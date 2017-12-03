#include "Raven_Bot.h"
#include "misc/Cgdi.h"
#include "misc/utils.h"
#include "2D/Transformations.h"
#include "2D/Geometry.h"
#include "lua/Raven_Scriptor.h"
#include "Raven_Game.h"
#include "navigation/Raven_PathPlanner.h"
#include "Raven_SteeringBehaviors.h"
#include "Raven_UserOptions.h"
#include "time/Regulator.h"
#include "Raven_WeaponSystem.h"
#include "Raven_SensoryMemory.h"
#include "armory/Raven_Weapon.h"

#include "Messaging/Telegram.h"
#include "Raven_Messages.h"
#include "Messaging/MessageDispatcher.h"

#include "goals/Raven_Goal_Types.h"
#include "goals/Goal_Think.h"


#include "Debug/DebugConsole.h"

//-------------------------- ctor ---------------------------------------------
Raven_Bot::Raven_Bot(Raven_Game* world,Vector2D pos):

  MovingEntity(pos,
               script->GetDouble("Bot_Scale"),
               Vector2D(0,0),
               script->GetDouble("Bot_MaxSpeed"),
               Vector2D(1,0),
               script->GetDouble("Bot_Mass"),
               Vector2D(script->GetDouble("Bot_Scale"),script->GetDouble("Bot_Scale")),
               script->GetDouble("Bot_MaxHeadTurnRate"),
               script->GetDouble("Bot_MaxForce")),
                 
                 m_iMaxHealth(script->GetInt("Bot_MaxHealth")),
                 m_iHealth(script->GetInt("Bot_MaxHealth")),
                 m_pPathPlanner(NULL),
                 m_pSteering(NULL),
                 m_pWorld(world),
                 m_pBrain(NULL),
                 m_iNumUpdatesHitPersistant((int)(FrameRate * script->GetDouble("HitFlashTime"))),
                 m_bHit(false),
                 m_iScore(0),
                 m_Status(spawning),
                 m_bPossessed(false),
                 m_dFieldOfView(DegsToRads(script->GetDouble("Bot_FOV")))
           
{
  SetEntityType(type_bot);

  SetUpVertexBuffer();
  
  //a bot starts off facing in the direction it is heading
  m_vFacing = m_vHeading;

  //create the navigation module
  m_pPathPlanner = new Raven_PathPlanner(this);

  //create the steering behavior class
  m_pSteering = new Raven_Steering(world, this);

  //create the regulators
  m_pWeaponSelectionRegulator = new Regulator(script->GetDouble("Bot_WeaponSelectionFrequency"));
  m_pGoalArbitrationRegulator =  new Regulator(script->GetDouble("Bot_GoalAppraisalUpdateFreq"));
  m_pTargetSelectionRegulator = new Regulator(script->GetDouble("Bot_TargetingUpdateFreq"));
  m_pTriggerTestRegulator = new Regulator(script->GetDouble("Bot_TriggerUpdateFreq"));
  m_pVisionUpdateRegulator = new Regulator(script->GetDouble("Bot_VisionUpdateFreq"));

  //create the goal queue
  m_pBrain = new Goal_Think(this);

  //create the targeting system
  m_pTargSys = new Raven_TargetingSystem(this);

  m_pWeaponSys = new Raven_WeaponSystem(this,
                                        script->GetDouble("Bot_ReactionTime"),
                                        script->GetDouble("Bot_AimAccuracy"),
                                        script->GetDouble("Bot_AimPersistance"));

  m_pSensoryMem = new Raven_SensoryMemory(this, script->GetDouble("Bot_MemorySpan"));
  InitializeFuzzyModule();

  isHumanPlayer = false;
}

//-------------------------------- dtor ---------------------------------------
//-----------------------------------------------------------------------------
Raven_Bot::~Raven_Bot()
{
  debug_con << "deleting raven bot (id = " << ID() << ")" << "";
  
  delete m_pBrain;
  delete m_pPathPlanner;
  delete m_pSteering;
  delete m_pWeaponSelectionRegulator;
  delete m_pTargSys;
  delete m_pGoalArbitrationRegulator;
  delete m_pTargetSelectionRegulator;
  delete m_pTriggerTestRegulator;
  delete m_pVisionUpdateRegulator;
  delete m_pWeaponSys;
  delete m_pSensoryMem;
}

//------------------------------- Spawn ---------------------------------------
//
//  spawns the bot at the given position
//-----------------------------------------------------------------------------
void Raven_Bot::Spawn(Vector2D pos)
{
    SetAlive();
    m_pBrain->RemoveAllSubgoals();
    m_pTargSys->ClearTarget();
    SetPos(pos);
    m_pWeaponSys->Initialize();
    RestoreHealthToMaximum();

	if (this->IsHumanPlayer())
		this->TurnIntoPlayable();
}

//------------------------------- DropITems -----------------------------------
//
//  Drop his items in his teamBase if team behavior is activate
//-----------------------------------------------------------------------------
void Raven_Bot::DropItems()
{
	if (GetWorld()->IsUsingTeam()) {
		Raven_Weapon* weapon;
		for (auto& w : GetWeaponSys()->GetAllWeapons()) {
			weapon = w.second;
			if (weapon != NULL) { GetWorld()->GetMap()->AddWeapon_Giver(weapon->GetType(), this); }
		}
	}
}

//-------------------------------- Update -------------------------------------
//
void Raven_Bot::Update()
{
  //process the currently active goal. Note this is required even if the bot
  //is under user control. This is because a goal is created whenever a user 
  //clicks on an area of the map that necessitates a path planning request.
  m_pBrain->Process();
  
  //Calculate the steering force and update the bot's velocity and position
  UpdateMovement();

  //if the bot is under AI control but not scripted
  if (!isPossessed())
  {
	  if (!m_pWorld->IsUsingTeam())
	  {
		  //examine all the opponents in the bots sensory memory and select one
		  //to be the current target
		  if (m_pTargetSelectionRegulator->isReady())
		  {
			  m_pTargSys->Update();
		  }

		  //appraise and arbitrate between all possible high level goals
		  if (m_pGoalArbitrationRegulator->isReady())
		  {
			  m_pBrain->Arbitrate();
		  }

		  //update the sensory memory with any visual stimulus
		  if (m_pVisionUpdateRegulator->isReady())
		  {
			  m_pSensoryMem->UpdateVision();
		  }

		  //select the appropriate weapon to use from the weapons currently in
		  //the inventory
		  if (m_pWeaponSelectionRegulator->isReady())
		  {
			  m_pWeaponSys->SelectWeapon();
		  }

		  //this method aims the bot's current weapon at the current target
		  //and takes a shot if a shot is possible
		  m_pWeaponSys->TakeAimAndShoot();
	  }
	  else
	  {
		  //examine all the opponents in the bots sensory memory and select one
		  //to be the current target
		  if (m_pTargetSelectionRegulator->isReady())
		  {
			  m_pTargSys->Update();
		  }

		  //appraise and arbitrate between all possible high level goals
		  if (m_pGoalArbitrationRegulator->isReady())
		  {
			  m_pBrain->Arbitrate();
		  }

		  //update the sensory memory with any visual stimulus
		  if (m_pVisionUpdateRegulator->isReady())
		  {
			  m_pSensoryMem->UpdateVision();
		  }

		  //select the appropriate weapon to use from the weapons currently in
		  //the inventory
		  if (m_pWeaponSelectionRegulator->isReady())
		  {
			  m_pWeaponSys->SelectWeapon();
		  }

		  //this method aims the bot's current weapon at the current target
		  //and takes a shot if a shot is possible
		  m_pWeaponSys->TakeAimAndShoot();
	  }
  }

}


//------------------------- UpdateMovement ------------------------------------
//
//  this method is called from the update method. It calculates and applies
//  the steering force for this time-step.
//-----------------------------------------------------------------------------
void Raven_Bot::UpdateMovement()
{
  //calculate the combined steering force
  Vector2D force = m_pSteering->Calculate();

  //if no steering force is produced decelerate the player by applying a
  //braking force
  if (m_pSteering->Force().isZero())
  {
    const double BrakingRate = 0.8; 

    m_vVelocity = m_vVelocity * BrakingRate;                                     
  }

  //calculate the acceleration
  Vector2D accel = force / m_dMass;

  //update the velocity
  m_vVelocity += accel;

  //make sure vehicle does not exceed maximum velocity
  m_vVelocity.Truncate(m_dMaxSpeed);

  //update the position
  m_vPosition += m_vVelocity;

  //if the vehicle has a non zero velocity the heading and side vectors must 
  //be updated
  if (!m_vVelocity.isZero())
  {    
    m_vHeading = Vec2DNormalize(m_vVelocity);

    m_vSide = m_vHeading.Perp();
  }
}
//---------------------------- isReadyForTriggerUpdate ------------------------
//
//  returns true if the bot is ready to be tested against the world triggers
//-----------------------------------------------------------------------------
bool Raven_Bot::isReadyForTriggerUpdate()const
{
  return m_pTriggerTestRegulator->isReady();
}

//--------------------------- HandleMessage -----------------------------------
//-----------------------------------------------------------------------------
bool Raven_Bot::HandleMessage(const Telegram& msg)
{
  //first see if the current goal accepts the message
  if (GetBrain()->HandleMessage(msg)) return true;
 
  //handle any messages not handles by the goals
  switch(msg.Msg)
  {
  case Msg_TakeThatMF:

    //just return if already dead or spawning
    if (isDead() || isSpawning()) return true;

    //the extra info field of the telegram carries the amount of damage
    ReduceHealth(DereferenceToType<int>(msg.ExtraInfo));

    //if this bot is now dead let the shooter know
    if (isDead())
    {
      Dispatcher->DispatchMsg(SEND_MSG_IMMEDIATELY,
                              ID(),
                              msg.Sender,
                              Msg_YouGotMeYouSOB,
                              NO_ADDITIONAL_INFO);
    }

    return true;

  case Msg_YouGotMeYouSOB:
    
    IncrementScore();
    
    //the bot this bot has just killed should be removed as the target
    m_pTargSys->ClearTarget();

    return true;

  case Msg_GunshotSound:

    //add the source of this sound to the bot's percepts
    GetSensoryMem()->UpdateWithSoundSource((Raven_Bot*)msg.ExtraInfo);

    return true;

  case Msg_UserHasRemovedBot:
    {

      Raven_Bot* pRemovedBot = (Raven_Bot*)msg.ExtraInfo;

      GetSensoryMem()->RemoveBotFromMemory(pRemovedBot);

      //if the removed bot is the target, make sure the target is cleared
      if (pRemovedBot == GetTargetSys()->GetTarget())
      {
        GetTargetSys()->ClearTarget();
      }

      return true;
    }


  default: return false;
  }
}

//------------------ RotateFacingTowardPosition -------------------------------
//
//  given a target position, this method rotates the bot's facing vector
//  by an amount not greater than m_dMaxTurnRate until it
//  directly faces the target.
//
//  returns true when the heading is facing in the desired direction
//----------------------------------------------------------------------------
bool Raven_Bot::RotateFacingTowardPosition(Vector2D target)
{
  Vector2D toTarget = Vec2DNormalize(target - m_vPosition);

  double dot = m_vFacing.Dot(toTarget);

  //clamp to rectify any rounding errors
  Clamp(dot, -1, 1);

  //determine the angle between the heading vector and the target
  double angle = acos(dot);

  //return true if the bot's facing is within WeaponAimTolerance degs of
  //facing the target
  const double WeaponAimTolerance = 0.01; //2 degs approx

  if (angle < WeaponAimTolerance)
  {
    m_vFacing = toTarget;
    return true;
  }

  //clamp the amount to turn to the max turn rate
  if (angle > m_dMaxTurnRate) angle = m_dMaxTurnRate;
  
  //The next few lines use a rotation matrix to rotate the player's facing
  //vector accordingly
  C2DMatrix RotationMatrix;
  
  //notice how the direction of rotation has to be determined when creating
  //the rotation matrix
  RotationMatrix.Rotate(angle * m_vFacing.Sign(toTarget));	
  RotationMatrix.TransformVector2Ds(m_vFacing);

  return false;
}




//--------------------------------- ReduceHealth ----------------------------
void Raven_Bot::ReduceHealth(unsigned int val)
{
  m_iHealth -= val;

  if (m_iHealth <= 0)
  {
    SetDead();
	DropItems();
  }

  m_bHit = true;

  m_iNumUpdatesHitPersistant = (int)(FrameRate * script->GetDouble("HitFlashTime"));
}

//--------------------------- Possess -----------------------------------------
//
//  this is called to allow a human player to control the bot
//-----------------------------------------------------------------------------
void Raven_Bot::TakePossession()
{
  if ( !(isSpawning() || isDead()))
  {
    m_bPossessed = true;

    debug_con << "Player Possesses bot " << this->ID() << "";
  }
}
//------------------------------- Exorcise ------------------------------------
//
//  called when a human is exorcised from this bot and the AI takes control
//-----------------------------------------------------------------------------
void Raven_Bot::Exorcise()
{
  m_bPossessed = false;

  //when the player is exorcised then the bot should resume normal service
  m_pBrain->AddGoal_Explore();
  
  debug_con << "Player is exorcised from bot " << this->ID() << "";
}
//------------------------------- TurnIntoPlayable ------------------------------------
//
//  called to modify a normal bot and make it a playable one
//-----------------------------------------------------------------------------
void Raven_Bot::TurnIntoPlayable()
{
	debug_con << "Player is now in game in bot " << this->ID() << "";
	double size = script->GetDouble("Bot_Scale") *2;
	//this->SetScale(Vector2D(size, size));
	this->GetWorld()->setPlayer(this);
	this->SetHumanPlayer(true);
	this->GetBrain()->RemoveAllSubgoals();
	this->TakePossession();

	this->GetSteering()->HumanControlOn();
}


//----------------------- ChangeWeapon ----------------------------------------
void Raven_Bot::ChangeWeapon(unsigned int type)
{
  m_pWeaponSys->ChangeWeapon(type);
}


//----------------------- getEnnemyTeam ----------------------------------------
std::list<Raven_Bot*> Raven_Bot::getEnnemyTeam()
{
	std::list<Raven_Bot*> myTeam = m_pWorld->getTeam(m_sTeamMembership);
	std::list<Raven_Bot*> ennemies = m_pWorld->GetAllBots();

	for (std::list<Raven_Bot*>::iterator it = myTeam.begin(); it != myTeam.end(); it++){
		ennemies.remove(*it);
	}

	return ennemies;
}
  

//---------------------------- FireWeapon -------------------------------------
//
//  fires the current weapon at the given position
//-----------------------------------------------------------------------------
void Raven_Bot::FireWeapon(Vector2D pos)
{
  m_pWeaponSys->ShootAt(pos);
}

//----------------- CalculateExpectedTimeToReachPosition ----------------------
//
//  returns a value indicating the time in seconds it will take the bot
//  to reach the given position at its current speed.
//-----------------------------------------------------------------------------
double Raven_Bot::CalculateTimeToReachPosition(Vector2D pos)const
{
  return Vec2DDistance(Pos(), pos) / (MaxSpeed() * FrameRate);
}

//------------------------ isAtPosition ---------------------------------------
//
//  returns true if the bot is close to the given position
//-----------------------------------------------------------------------------
bool Raven_Bot::isAtPosition(Vector2D pos)const
{
  const static double tolerance = 10.0;
  
  return Vec2DDistanceSq(Pos(), pos) < tolerance * tolerance;
}

//------------------------- hasLOSt0 ------------------------------------------
//
//  returns true if the bot has line of sight to the given position.
//-----------------------------------------------------------------------------
bool Raven_Bot::hasLOSto(Vector2D pos)const
{
  return m_pWorld->isLOSOkay(Pos(), pos);
}

//returns true if this bot can move directly to the given position
//without bumping into any walls
bool Raven_Bot::canWalkTo(Vector2D pos)const
{
  return !m_pWorld->isPathObstructed(Pos(), pos, BRadius());
}

//similar to above. Returns true if the bot can move between the two
//given positions without bumping into any walls
bool Raven_Bot::canWalkBetween(Vector2D from, Vector2D to)const
{
 return !m_pWorld->isPathObstructed(from, to, BRadius());
}

//--------------------------- canStep Methods ---------------------------------
//
//  returns true if there is space enough to step in the indicated direction
//  If true PositionOfStep will be assigned the offset position
//-----------------------------------------------------------------------------
bool Raven_Bot::canStepLeft(Vector2D& PositionOfStep)const
{
  static const double StepDistance = BRadius() * 2;

  PositionOfStep = Pos() - Facing().Perp() * StepDistance - Facing().Perp() * BRadius();

  return canWalkTo(PositionOfStep);
}

bool Raven_Bot::canStepRight(Vector2D& PositionOfStep)const
{
  static const double StepDistance = BRadius() * 2;

  PositionOfStep = Pos() + Facing().Perp() * StepDistance + Facing().Perp() * BRadius();

  return canWalkTo(PositionOfStep);
}

bool Raven_Bot::canStepForward(Vector2D& PositionOfStep)const
{
  static const double StepDistance = BRadius() * 2;

  PositionOfStep = Pos() + Facing() * StepDistance + Facing() * BRadius();

  return canWalkTo(PositionOfStep);
}

bool Raven_Bot::canStepBackward(Vector2D& PositionOfStep)const
{
  static const double StepDistance = BRadius() * 2;

  PositionOfStep = Pos() - Facing() * StepDistance - Facing() * BRadius();

  return canWalkTo(PositionOfStep);
}

//--------------------------- Render -------------------------------------
//
//------------------------------------------------------------------------
void Raven_Bot::Render()                                         
{
  //when a bot is hit by a projectile this value is set to a constant user
  //defined value which dictates how long the bot should have a thick red
  //circle drawn around it (to indicate it's been hit) The circle is drawn
  //as long as this value is positive. (see Render)
  m_iNumUpdatesHitPersistant--;


  if (isDead() || isSpawning()) return;
  
  gdi->BluePen();
  
  m_vecBotVBTrans = WorldTransform(m_vecBotVB,
                                   Pos(),
                                   Facing(),
                                   Facing().Perp(),
                                   Scale());

  gdi->ClosedShape(m_vecBotVBTrans);
  
  //draw the head
  if (this->IsHumanPlayer()) {
	  gdi->RedBrush();
  }
  else {
	  gdi->BrownBrush();
  }

  gdi->Circle(Pos(), 6.0 * Scale().x);



  //render the bot's weapon
  m_pWeaponSys->RenderCurrentWeapon();

  //render a thick red circle if the bot gets hit by a weapon
  if (m_bHit)
  {
    gdi->ThickRedPen();
    gdi->HollowBrush();
    gdi->Circle(m_vPosition, BRadius()+1);

    if (m_iNumUpdatesHitPersistant <= 0)
    {
      m_bHit = false;
    }
  }

  gdi->TransparentText();
  gdi->TextColor(0,255,0);

  if (UserOptions->m_bShowBotIDs)
  {
    gdi->TextAtPos(Pos().x -10, Pos().y-40, ttos(ID()));
  }

  if (UserOptions->m_bShowBotHealth)
  {
    gdi->TextAtPos(Pos().x-40, Pos().y-5, "H:"+ ttos(Health()));
  }

  if (UserOptions->m_bShowBotTeam)
  {
	  gdi->TextAtPos(Pos().x - 40, Pos().y - 20, m_sTeamMembership);
  }

  if (UserOptions->m_bShowScore)
  {
    gdi->TextAtPos(Pos().x-40, Pos().y+10, "Scr:"+ ttos(Score()));
  }    
}

//------------------------- SetUpVertexBuffer ---------------------------------
//-----------------------------------------------------------------------------
void Raven_Bot::SetUpVertexBuffer()
{
  //setup the vertex buffers and calculate the bounding radius
  const int NumBotVerts = 4;
  const Vector2D bot[NumBotVerts] = {Vector2D(-3, 8),
                                     Vector2D(3,10),
                                     Vector2D(3,-10),
                                     Vector2D(-3,-8)};

  m_dBoundingRadius = 0.0;
  double scale = script->GetDouble("Bot_Scale");
  
  for (int vtx=0; vtx<NumBotVerts; ++vtx)
  {
    m_vecBotVB.push_back(bot[vtx]);

    //set the bounding radius to the length of the 
    //greatest extent
    if (abs(bot[vtx].x)*scale > m_dBoundingRadius)
    {
      m_dBoundingRadius = abs(bot[vtx].x*scale);
    }

    if (abs(bot[vtx].y)*scale > m_dBoundingRadius)
    {
      m_dBoundingRadius = abs(bot[vtx].y)*scale;
    }
  }
}



void Raven_Bot::RestoreHealthToMaximum(){m_iHealth = m_iMaxHealth;}

void Raven_Bot::IncreaseHealth(unsigned int val)
{
  m_iHealth+=val; 
  Clamp(m_iHealth, 0, m_iMaxHealth);
}

//---------------------------- Accuracy -----------------------------------
//
//-----------------------------------------------------------------------------
double Raven_Bot::GetAccuracy()
{
		//fuzzify inputs
		m_FuzzyModule.Fuzzify("AimFuzzy_DistToTarget", Vec2DDistance(Pos(), GetTargetSys()->GetTarget()->Pos()));
		m_FuzzyModule.Fuzzify("AimFuzzy_OwnerVelocity", Velocity().Length());
		m_FuzzyModule.Fuzzify("AimFuzzy_TimeTargetHasBeenVisible", GetTargetSys()->GetTimeTargetHasBeenVisible());

		// Ouput Accuracy as double
		return m_FuzzyModule.DeFuzzify("AimFuzzy_Accuracy", FuzzyModule::max_av);
}

//-------------------------  InitializeFuzzyModuleForAiming -------------------
//
//  set up some fuzzy variables and rules
//-----------------------------------------------------------------------------
void Raven_Bot::InitializeFuzzyModule() {

	FuzzyVariable& AimFuzzy_DistToTarget = m_FuzzyModule.CreateFLV("AimFuzzy_DistToTarget");

	FzSet& Target_Close = AimFuzzy_DistToTarget.AddLeftShoulderSet("Target_Close", 0, 25, 50);
	FzSet& Target_Medium = AimFuzzy_DistToTarget.AddTriangularSet("Target_Medium", 25, 50, 300);
	FzSet& Target_Far = AimFuzzy_DistToTarget.AddRightShoulderSet("Target_Far", 450, 600, 1000);

	FuzzyVariable& AimFuzzy_OwnerVelocity = m_FuzzyModule.CreateFLV("AimFuzzy_OwnerVelocity");

	FzSet& LowVelocity = AimFuzzy_OwnerVelocity.AddLeftShoulderSet("LowVelocity", 0, 25, 50);
	FzSet& AverageVelocity = AimFuzzy_OwnerVelocity.AddTriangularSet("AverageVelocity", 25, 50, 75);
	FzSet& HighVelocity = AimFuzzy_OwnerVelocity.AddRightShoulderSet("HighVelocity", 50, 75, 100);

	FuzzyVariable& AimFuzzy_TimeTargetHasBeenVisible = m_FuzzyModule.CreateFLV("AimFuzzy_TimeTargetHasBeenVisible");

	FzSet& HighTime = AimFuzzy_TimeTargetHasBeenVisible.AddRightShoulderSet("HighTime", 1, 2, 100);
	FzSet& AverageTime = AimFuzzy_TimeTargetHasBeenVisible.AddTriangularSet("AverageTime", .5, 1, 2);
	FzSet& ShortTime = AimFuzzy_TimeTargetHasBeenVisible.AddTriangularSet("ShortTime", 0, .5, 1);

	// Need to Adjuste those variable
	FuzzyVariable& AimFuzzy_Accuracy = m_FuzzyModule.CreateFLV("AimFuzzy_Accuracy");

	FzSet& HighAccuracy = AimFuzzy_Accuracy.AddLeftShoulderSet("HighAccuracy", 0, .1, .2); 
	FzSet& AverageAccuracy = AimFuzzy_Accuracy.AddTriangularSet("AverageAccuracy", .1, .2, .3);
	FzSet& LowAccuracy = AimFuzzy_Accuracy.AddRightShoulderSet("LowAccuracy", .2, .3, .4);

	m_FuzzyModule.AddRule(FzAND(Target_Close, LowVelocity, HighTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Close, LowVelocity, AverageTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Close, LowVelocity, ShortTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Close, AverageVelocity, HighTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Close, AverageVelocity, AverageTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Close, AverageVelocity, ShortTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Close, HighVelocity, HighTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Close, HighVelocity, AverageTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Close, HighVelocity, ShortTime), AverageAccuracy);

	m_FuzzyModule.AddRule(FzAND(Target_Medium, LowVelocity, HighTime), HighAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Medium, LowVelocity, AverageTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Medium, LowVelocity, ShortTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Medium, AverageVelocity, HighTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Medium, AverageVelocity, AverageTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Medium, AverageVelocity, ShortTime), LowAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Medium, HighVelocity, HighTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Medium, HighVelocity, AverageTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Medium, HighVelocity, ShortTime), LowAccuracy);

	m_FuzzyModule.AddRule(FzAND(Target_Far, LowVelocity, HighTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Far, LowVelocity, AverageTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Far, LowVelocity, ShortTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Far, AverageVelocity, HighTime), AverageAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Far, AverageVelocity, AverageTime), LowAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Far, AverageVelocity, ShortTime), LowAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Far, HighVelocity, HighTime), LowAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Far, HighVelocity, AverageTime), LowAccuracy);
	m_FuzzyModule.AddRule(FzAND(Target_Far, HighVelocity, ShortTime), LowAccuracy);
}