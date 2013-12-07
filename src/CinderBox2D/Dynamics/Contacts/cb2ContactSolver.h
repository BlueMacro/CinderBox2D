/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CB2_CONTACT_SOLVER_H
#define CB2_CONTACT_SOLVER_H

#include <CinderBox2D/Common/cb2Math.h>
#include <CinderBox2D/Collision/cb2Collision.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>

class cb2Contact;
class cb2Body;
class cb2StackAllocator;
struct cb2ContactPositionConstraint;

struct cb2VelocityConstraintPoint
{
	ci::Vec2f rA;
	ci::Vec2f rB;
	float normalImpulse;
	float tangentImpulse;
	float normalMass;
	float tangentMass;
	float velocityBias;
};

struct cb2ContactVelocityConstraint
{
	cb2VelocityConstraintPoint points[cb2_maxManifoldPoints];
	ci::Vec2f normal;
	ci::Matrix22f normalMass;
	ci::Matrix22f K;
	int indexA;
	int indexB;
	float invMassA, invMassB;
	float invIA, invIB;
	float friction;
	float restitution;
	float tangentSpeed;
	int pointCount;
	int contactIndex;
};

struct cb2ContactSolverDef
{
	cb2TimeStep step;
	cb2Contact** contacts;
	int count;
	cb2Position* positions;
	cb2Velocity* velocities;
	cb2StackAllocator* allocator;
};

class cb2ContactSolver
{
public:
	cb2ContactSolver(cb2ContactSolverDef* def);
	~cb2ContactSolver();

	void InitializeVelocityConstraints();

	void WarmStart();
	void SolveVelocityConstraints();
	void StoreImpulses();

	bool SolvePositionConstraints();
	bool SolveTOIPositionConstraints(int toiIndexA, int toiIndexB);

	cb2TimeStep m_step;
	cb2Position* m_positions;
	cb2Velocity* m_velocities;
	cb2StackAllocator* m_allocator;
	cb2ContactPositionConstraint* m_positionConstraints;
	cb2ContactVelocityConstraint* m_velocityConstraints;
	cb2Contact** m_contacts;
	int m_count;
};

#endif

