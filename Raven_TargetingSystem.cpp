#include "Raven_TargetingSystem.h"
#include "Raven_Bot.h"
#include "Raven_Game.h"
#include "Raven_SensoryMemory.h"
#include "Debug/DebugConsole.h"
#include "Raven_WeaponSystem.h"
#include "armory/Raven_Weapon.h"




//-------------------------------- ctor ---------------------------------------
//-----------------------------------------------------------------------------
Raven_TargetingSystem::Raven_TargetingSystem(Raven_Bot* owner):m_pOwner(owner),
                                                               m_pCurrentTarget(0)
{
}



//----------------------------- Update ----------------------------------------

//-----------------------------------------------------------------------------
void Raven_TargetingSystem::Update()
{
  double ClosestDistSoFar = MaxDouble;
  double distance = MaxDouble;
  double angle = MaxDouble;

  m_pCurrentTarget = 0;

  //grab a list of all the opponents the owner can sense
  std::list<Raven_Bot*> SensedBots;
  SensedBots = m_pOwner->GetSensoryMem()->GetListOfRecentlySensedOpponents();
  
  std::list<Raven_Bot*>::const_iterator curBot = SensedBots.begin();
  for (curBot; curBot != SensedBots.end(); ++curBot)
  {
    //make sure the bot is alive and that it is not the owner
    if ((*curBot)->isAlive() && (*curBot != m_pOwner) )
    {
		double cur_distance = Vec2DDistanceSq((*curBot)->Pos(), m_pOwner->Pos());

		if (useNN) {
			Vector2D forward = m_pOwner->Facing();
			Vector2D PtoB = (*curBot)->Pos() - m_pOwner->Pos();

			double cur_angle = acos(forward.Dot(PtoB) / (Vec2DLength(forward) * Vec2DLength(PtoB)));

			fann_type *calc_out;
			fann_type input[8];

			input[0] = angle;
			input[1] = distance;

			input[2] = cur_angle;
			input[3] = cur_distance;

			input[4] = m_pOwner->GetWeaponSys()->GetCurrentWeapon()->GetIdealRange();
			input[5] = m_pOwner->GetWeaponSys()->GetCurrentWeapon()->GetType();

			input[6] = (*curBot)->GetWeaponSys()->GetCurrentWeapon()->GetIdealRange();
			input[7] = (*curBot)->GetWeaponSys()->GetCurrentWeapon()->GetType();

			calc_out = fann_run(m_pOwner->GetWorld()->GetNeuralNet(), input);

			if (*calc_out > 0.5) {
				distance = cur_distance;
				angle = cur_angle;
				m_pCurrentTarget = *curBot;
			}
		}
	  else if (cur_distance < ClosestDistSoFar)
      {
		ClosestDistSoFar = cur_distance;
		m_pCurrentTarget = *curBot;
      }
    }
  }
}




bool Raven_TargetingSystem::isTargetWithinFOV()const
{
  return m_pOwner->GetSensoryMem()->isOpponentWithinFOV(m_pCurrentTarget);
}

bool Raven_TargetingSystem::isTargetShootable()const
{
  return m_pOwner->GetSensoryMem()->isOpponentShootable(m_pCurrentTarget);
}

Vector2D Raven_TargetingSystem::GetLastRecordedPosition()const
{
  return m_pOwner->GetSensoryMem()->GetLastRecordedPositionOfOpponent(m_pCurrentTarget);
}

double Raven_TargetingSystem::GetTimeTargetHasBeenVisible()const
{
  return m_pOwner->GetSensoryMem()->GetTimeOpponentHasBeenVisible(m_pCurrentTarget);
}

double Raven_TargetingSystem::GetTimeTargetHasBeenOutOfView()const
{
  return m_pOwner->GetSensoryMem()->GetTimeOpponentHasBeenOutOfView(m_pCurrentTarget);
}
