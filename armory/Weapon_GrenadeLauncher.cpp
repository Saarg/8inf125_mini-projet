#include "Weapon_GrenadeLauncher.h"
#include "../Raven_Bot.h"
#include "misc/Cgdi.h"
#include "../Raven_Game.h"
#include "../Raven_Map.h"
#include "../lua/Raven_Scriptor.h"
#include "fuzzy/FuzzyOperators.h"

#include "Debug/DebugConsole.h"

//--------------------------- ctor --------------------------------------------
//-----------------------------------------------------------------------------
GrenadeLauncher::GrenadeLauncher(Raven_Bot*   owner) :

	Raven_Weapon(type_grenade_launcher,
		script->GetInt("GrenadeLauncher_DefaultRounds"),
		script->GetInt("GrenadeLauncher_MaxRoundsCarried"),
		script->GetDouble("GrenadeLauncher_FiringFreq"),
		script->GetDouble("GrenadeLauncher_IdealRange"),
		script->GetDouble("Grenade_MaxSpeed"),
		owner)
{
	//setup the vertex buffer
	const int NumWeaponVerts = 8;
	const Vector2D weapon[NumWeaponVerts] = { Vector2D(0, -3),
											  Vector2D(6, -3),
											  Vector2D(6, -1),
											  Vector2D(15, -1),
											  Vector2D(15, 1),
											  Vector2D(6, 1),
											  Vector2D(6, 3),
  											  Vector2D(0, 3)
	};

	for (int vtx = 0; vtx<NumWeaponVerts; ++vtx)
	{
		m_vecWeaponVB.push_back(weapon[vtx]);
	}

	//setup the fuzzy module
	InitializeFuzzyModule();
}


//------------------------------ ShootAt --------------------------------------

inline void GrenadeLauncher::ShootAt(Vector2D pos)
{
	if (isReadyForNextShot())
	{
		double maxDist = script->GetDouble("Grenade_MaxDistance");

		Vector2D diff = pos - m_pOwner->Pos();
		diff.Truncate(maxDist);

		Vector2D target = m_pOwner->Pos() + diff;

		//fire!
		m_pOwner->GetWorld()->AddGrenade(m_pOwner, target);

		UpdateTimeWeaponIsNextAvailable();

		//add a trigger to the game so that the other bots can hear this shot
		//(provided they are within range)
		m_pOwner->GetWorld()->GetMap()->AddSoundTrigger(m_pOwner, script->GetDouble("GrenadeLauncher_SoundRange"));
	}
}



//---------------------------- Desirability -----------------------------------
//
//-----------------------------------------------------------------------------
double GrenadeLauncher::GetDesirability(double DistToTarget)
{
	//fuzzify distance and amount of ammo
	m_FuzzyModule.Fuzzify("DistToTarget", DistToTarget);

	m_dLastDesirabilityScore = m_FuzzyModule.DeFuzzify("Desirability", FuzzyModule::max_av);

	return m_dLastDesirabilityScore;
}

//----------------------- InitializeFuzzyModule -------------------------------
//
//  set up some fuzzy variables and rules
//-----------------------------------------------------------------------------
void GrenadeLauncher::InitializeFuzzyModule()
{
	FuzzyVariable& DistToTarget = m_FuzzyModule.CreateFLV("DistToTarget");

	FzSet& Target_Close = DistToTarget.AddLeftShoulderSet("Target_Close", 0, 25, 150);
	FzSet& Target_Medium = DistToTarget.AddTriangularSet("Target_Medium", 25, 150, 300);
	FzSet& Target_Far = DistToTarget.AddRightShoulderSet("Target_Far", 150, 300, 1000);

	FuzzyVariable& Desirability = m_FuzzyModule.CreateFLV("Desirability");
	FzSet& VeryDesirable = Desirability.AddRightShoulderSet("VeryDesirable", 50, 75, 100);
	FzSet& Desirable = Desirability.AddTriangularSet("Desirable", 25, 50, 75);
	FzSet& Undesirable = Desirability.AddLeftShoulderSet("Undesirable", 0, 25, 50);

	m_FuzzyModule.AddRule(Target_Close, VeryDesirable);
	m_FuzzyModule.AddRule(Target_Medium, FzVery(Undesirable));
	m_FuzzyModule.AddRule(Target_Far, FzVery(Undesirable));
}


//-------------------------------- Render -------------------------------------
//-----------------------------------------------------------------------------
void GrenadeLauncher::Render()
{
	m_vecWeaponVBTrans = WorldTransform(m_vecWeaponVB,
		m_pOwner->Pos(),
		m_pOwner->Facing(),
		m_pOwner->Facing().Perp(),
		m_pOwner->Scale());

	gdi->PinkPen();

	gdi->ClosedShape(m_vecWeaponVBTrans);
}