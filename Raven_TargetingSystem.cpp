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
  double aimProb = 0.5;

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
			fann_type *calc_out;
			fann_type input[4];

			input[0] = (*curBot)->Health();
			input[1] = cur_distance;

			input[2] = m_pOwner->GetWeaponSys()->GetCurrentWeapon()->GetIdealRange();
			input[3] = m_pOwner->GetWeaponSys()->GetCurrentWeapon()->GetType();

			calc_out = fann_run(m_pOwner->GetWorld()->GetNeuralNet(), input);

			if (*calc_out > aimProb) {
				m_pCurrentTarget = *curBot;
				aimProb = *calc_out;
				ClosestDistSoFar = cur_distance;
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
