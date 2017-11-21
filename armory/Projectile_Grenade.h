#ifndef GRENADE_H
#define GRENADE_H
#pragma warning (disable:4786)
//-----------------------------------------------------------------------------
//
//  Name:   Grenade.h
//
//  Author: Mat Buckland (ai-junkie.com)
//
//  Desc:   class to implement a rocket
//
//-----------------------------------------------------------------------------

#include "Raven_Projectile.h"

class Raven_Bot;

class Grenade : public Raven_Projectile
{
private:

	//the radius of damage, once the grenade has impacted
	double    m_dBlastRadius;

	//this is used to render the splash when the grenade impacts
	double    m_dCurrentBlastRadius;

	//If the grenade has impacted we test all bots to see if they are within the 
	//blast radius and reduce their health accordingly
	void InflictDamageOnBotsWithinBlastRadius();

	//tests the trajectory of the shell for an impact
	void TestForImpact();

public:

	Grenade(Raven_Bot* shooter, Vector2D target);

	void Render();

	void Update();

};


#endif