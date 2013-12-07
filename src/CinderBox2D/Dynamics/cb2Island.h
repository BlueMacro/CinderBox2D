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

#ifndef CB2_ISLAND_H
#define CB2_ISLAND_H

#include <CinderBox2D/Common/cb2Math.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>

class cb2Contact;
class cb2Joint;
class cb2StackAllocator;
class cb2ContactListener;
struct cb2ContactVelocityConstraint;
struct cb2Profile;

/// This is an internal class.
class cb2Island
{
public:
	cb2Island(int bodyCapacity, int contactCapacity, int jointCapacity,
			cb2StackAllocator* allocator, cb2ContactListener* listener);
	~cb2Island();

	void Clear()
	{
		m_bodyCount = 0;
		m_contactCount = 0;
		m_jointCount = 0;
	}

	void Solve(cb2Profile* profile, const cb2TimeStep& step, const ci::Vec2f& gravity, bool allowSleep);

	void SolveTOI(const cb2TimeStep& subStep, int toiIndexA, int toiIndexB);

	void Add(cb2Body* body)
	{
		cb2Assert(m_bodyCount < m_bodyCapacity);
		body->m_islandIndex = m_bodyCount;
		m_bodies[m_bodyCount] = body;
		++m_bodyCount;
	}

	void Add(cb2Contact* contact)
	{
		cb2Assert(m_contactCount < m_contactCapacity);
		m_contacts[m_contactCount++] = contact;
	}

	void Add(cb2Joint* joint)
	{
		cb2Assert(m_jointCount < m_jointCapacity);
		m_joints[m_jointCount++] = joint;
	}

	void Report(const cb2ContactVelocityConstraint* constraints);

	cb2StackAllocator* m_allocator;
	cb2ContactListener* m_listener;

	cb2Body** m_bodies;
	cb2Contact** m_contacts;
	cb2Joint** m_joints;

	cb2Position* m_positions;
	cb2Velocity* m_velocities;

	int m_bodyCount;
	int m_jointCount;
	int m_contactCount;

	int m_bodyCapacity;
	int m_contactCapacity;
	int m_jointCapacity;
};

#endif
