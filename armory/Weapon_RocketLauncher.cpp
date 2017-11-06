#include "Weapon_RocketLauncher.h"
#include "../Raven_Bot.h"
#include "misc/Cgdi.h"
#include "../Raven_Game.h"
#include "../Raven_Map.h"
#include "../lua/Raven_Scriptor.h"
#include "fuzzy/FuzzyOperators.h"


//--------------------------- ctor --------------------------------------------
//-----------------------------------------------------------------------------
RocketLauncher::RocketLauncher(Raven_Bot*   owner):

                      Raven_Weapon(type_rocket_launcher,
                                   script->GetInt("RocketLauncher_DefaultRounds"),
                                   script->GetInt("RocketLauncher_MaxRoundsCarried"),
                                   script->GetDouble("RocketLauncher_FiringFreq"),
                                   script->GetDouble("RocketLauncher_IdealRange"),
                                   script->GetDouble("Rocket_MaxSpeed"),
                                   owner)
{
    //setup the vertex buffer
  const int NumWeaponVerts = 8;
  const Vector2D weapon[NumWeaponVerts] = {Vector2D(0, -3),
                                           Vector2D(6, -3),
                                           Vector2D(6, -1),
                                           Vector2D(15, -1),
                                           Vector2D(15, 1),
                                           Vector2D(6, 1),
                                           Vector2D(6, 3),
                                           Vector2D(0, 3)
                                           };
  for (int vtx=0; vtx<NumWeaponVerts; ++vtx)
  {
    m_vecWeaponVB.push_back(weapon[vtx]);
  }

  //setup the fuzzy module
  InitializeFuzzyModule();

}


//------------------------------ ShootAt --------------------------------------
//-----------------------------------------------------------------------------
inline void RocketLauncher::ShootAt(Vector2D pos)
{ 
  if (NumRoundsRemaining() > 0 && isReadyForNextShot())
  {
    //fire off a rocket!
    m_pOwner->GetWorld()->AddRocket(m_pOwner, pos);

    m_iNumRoundsLeft--;

    UpdateTimeWeaponIsNextAvailable();

    //add a trigger to the game so that the other bots can hear this shot
    //(provided they are within range)
    m_pOwner->GetWorld()->GetMap()->AddSoundTrigger(m_pOwner, script->GetDouble("RocketLauncher_SoundRange"));
  }
}

//---------------------------- Desirability -----------------------------------
//
//-----------------------------------------------------------------------------
double RocketLauncher::GetDesirability(double DistToTarget)
{
  if (m_iNumRoundsLeft == 0)
  {
    m_dLastDesirabilityScore = 0;
  }
  else
  {
    //fuzzify distance and amount of ammo
    m_FuzzyModule.Fuzzify("DistToTarget", DistToTarget);
    m_FuzzyModule.Fuzzify("AmmoStatus", (double)m_iNumRoundsLeft);

    m_dLastDesirabilityScore = m_FuzzyModule.DeFuzzify("Desirability", FuzzyModule::max_av);
  }

  return m_dLastDesirabilityScore;
}

//-------------------------  InitializeFuzzyModule ----------------------------
//
//  set up some fuzzy variables and rules
//-----------------------------------------------------------------------------
void RocketLauncher::InitializeFuzzyModule()
{
  FuzzyVariable& DistToTarget = m_FuzzyModule.CreateFLV("DistToTarget");

  FzSet& Target_Melee = DistToTarget.AddLeftShoulderSet("Target_Close",0,25,50);
  FzSet& Target_Close = DistToTarget.AddTriangularSet("Target_Medium",25,50,300);
  FzSet& Target_Medium = DistToTarget.AddTriangularSet("Target_Far",50,300,450);
  FzSet& Target_Far = DistToTarget.AddTriangularSet("Target_Far", 300, 450, 600);
  FzSet& Target_VeryFar = DistToTarget.AddRightShoulderSet("Target_Far", 450, 600, 1000);

  FuzzyVariable& Desirability = m_FuzzyModule.CreateFLV("Desirability"); 

  FzSet& VeryHighDesirability = Desirability.AddRightShoulderSet("VeryHighDesirability", 70, 85, 100);
  FzSet& HighDesirability = Desirability.AddTriangularSet("HighDesirability", 50, 70, 85);
  FzSet& AverageDesirability = Desirability.AddTriangularSet("AverageDesirability", 30, 50, 70);
  FzSet& LowDesirability = Desirability.AddTriangularSet("LowDesirability", 15, 30, 50 );
  FzSet& VeryLowDesirability = Desirability.AddLeftShoulderSet("VeryLowDesirability", 0, 15, 30);

  FuzzyVariable& AmmoStatus = m_FuzzyModule.CreateFLV("AmmoStatus");

  FzSet& Ammo_VeryHigh = AmmoStatus.AddRightShoulderSet("Ammo_VeryHigh", 90, 95, 100); // Nearly full of ammo
  FzSet& Ammo_High = AmmoStatus.AddTriangularSet("Ammo_High", 45, 65, 30);
  FzSet& Ammo_Okay = AmmoStatus.AddTriangularSet("Ammo_Okay", 25, 45, 65);
  FzSet& Ammo_Low = AmmoStatus.AddTriangularSet("Ammo_Low", 5, 30, 50);
  FzSet& Ammo_VeryLow = AmmoStatus.AddTriangularSet("Ammo_VeryLow", 0, 5, 10); // Nearly empty


  m_FuzzyModule.AddRule(FzAND(Target_Melee, Ammo_VeryHigh), VeryLowDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Melee, Ammo_High), VeryLowDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Melee, Ammo_Okay), VeryLowDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Melee, Ammo_Low), VeryLowDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Melee, Ammo_VeryLow), VeryLowDesirability);

  m_FuzzyModule.AddRule(FzAND(Target_Close, Ammo_VeryHigh), VeryHighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Close, Ammo_High), VeryHighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Close, Ammo_Okay), VeryHighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Close, Ammo_Low), VeryHighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Close, Ammo_VeryLow), LowDesirability);

  m_FuzzyModule.AddRule(FzAND(Target_Medium, Ammo_VeryHigh), VeryHighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Medium, Ammo_High), VeryHighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Medium, Ammo_Okay), VeryHighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Medium, Ammo_Low), VeryHighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Medium, Ammo_VeryLow), LowDesirability);

  m_FuzzyModule.AddRule(FzAND(Target_Far, Ammo_VeryHigh), HighDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Far, Ammo_High), AverageDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Far, Ammo_Okay), AverageDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Far, Ammo_Low), AverageDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_Far, Ammo_VeryLow), VeryLowDesirability);

  m_FuzzyModule.AddRule(FzAND(Target_VeryFar, Ammo_VeryHigh), AverageDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_VeryFar, Ammo_High), AverageDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_VeryFar, Ammo_Okay), LowDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_VeryFar, Ammo_Low), LowDesirability);
  m_FuzzyModule.AddRule(FzAND(Target_VeryFar, Ammo_VeryLow), VeryLowDesirability);
}


//-------------------------------- Render -------------------------------------
//-----------------------------------------------------------------------------
void RocketLauncher::Render()
{
    m_vecWeaponVBTrans = WorldTransform(m_vecWeaponVB,
                                   m_pOwner->Pos(),
                                   m_pOwner->Facing(),
                                   m_pOwner->Facing().Perp(),
                                   m_pOwner->Scale());

  gdi->RedPen();

  gdi->ClosedShape(m_vecWeaponVBTrans);
}