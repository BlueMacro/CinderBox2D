/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2Fixture.h>
#include <CinderBox2D/Dynamics/cb2World.h>
#include <CinderBox2D/Dynamics/Contacts/cb2Contact.h>
#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>

cb2Body::cb2Body(const cb2BodyDef* bd, cb2World* world)
{
	cb2Assert(cb2::isValid(bd->position));
	cb2Assert(cb2::isValid(bd->linearVelocity));
	cb2Assert(cb2::isValid(bd->angle));
	cb2Assert(cb2::isValid(bd->angularVelocity));
	cb2Assert(cb2::isValid(bd->angularDamping) && bd->angularDamping >= 0.0f);
	cb2Assert(cb2::isValid(bd->linearDamping) && bd->linearDamping >= 0.0f);

	m_flags = 0;

	if (bd->bullet)
	{
		m_flags |= e_bulletFlag;
	}
	if (bd->fixedRotation)
	{
		m_flags |= e_fixedRotationFlag;
	}
	if (bd->allowSleep)
	{
		m_flags |= e_autoSleepFlag;
	}
	if (bd->awake)
	{
		m_flags |= e_awakeFlag;
	}
	if (bd->active)
	{
		m_flags |= e_activeFlag;
	}

	m_world = world;

	m_xf.p = bd->position;
	m_xf.q.set(bd->angle);

	m_sweep.c0 = m_xf.p;
	m_sweep.c = m_xf.p;
	m_sweep.a0 = bd->angle;
	m_sweep.a = bd->angle;
	m_sweep.alpha0 = 0.0f;

	m_jointList = NULL;
	m_contactList = NULL;
	m_prev = NULL;
	m_next = NULL;

	m_linearVelocity = bd->linearVelocity;
	m_angularVelocity = bd->angularVelocity;

	m_linearDamping = bd->linearDamping;
	m_angularDamping = bd->angularDamping;
	m_gravityScale = bd->gravityScale;

	m_torque = 0.0f;

	m_sleepTime = 0.0f;

	m_type = bd->type;

	if (m_type == cb2_dynamicBody)
	{
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}
	else
	{
		m_mass = 0.0f;
		m_invMass = 0.0f;
	}

	m_I = 0.0f;
	m_invI = 0.0f;

	m_userData = bd->userData;

	m_fixtureList = NULL;
	m_fixtureCount = 0;
}

cb2Body::~cb2Body()
{
	// shapes and joints are destroyed in cb2World::Destroy
}

void cb2Body::SetType(cb2BodyType type)
{
	cb2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	if (m_type == type)
	{
		return;
	}

	m_type = type;

	ResetMassData();

	if (m_type == cb2_staticBody)
	{
		cb2::setZero(m_linearVelocity);
		m_angularVelocity = 0.0f;
		m_sweep.a0 = m_sweep.a;
		m_sweep.c0 = m_sweep.c;
		SynchronizeFixtures();
	}

	SetAwake(true);

	cb2::setZero(m_force);
	m_torque = 0.0f;

	// Delete the attached contacts.
	cb2ContactEdge* ce = m_contactList;
	while (ce)
	{
		cb2ContactEdge* ce0 = ce;
		ce = ce->next;
		m_world->m_contactManager.Destroy(ce0->contact);
	}
	m_contactList = NULL;

	// Touch the proxies so that new contacts will be created (when appropriate)
	cb2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (cb2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		int proxyCount = f->m_proxyCount;
		for (int i = 0; i < proxyCount; ++i)
		{
			broadPhase->TouchProxy(f->m_proxies[i].proxyId);
		}
	}
}

cb2Fixture* cb2Body::CreateFixture(const cb2FixtureDef* def)
{
	cb2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return NULL;
	}

	cb2BlockAllocator* allocator = &m_world->m_blockAllocator;

	void* memory = allocator->Allocate(sizeof(cb2Fixture));
	cb2Fixture* fixture = new (memory) cb2Fixture;
	fixture->Create(allocator, this, def);

	if (m_flags & e_activeFlag)
	{
		cb2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		fixture->CreateProxies(broadPhase, m_xf);
	}

	fixture->m_next = m_fixtureList;
	m_fixtureList = fixture;
	++m_fixtureCount;

	fixture->m_body = this;

	// Adjust mass properties if needed.
	if (fixture->m_density > 0.0f)
	{
		ResetMassData();
	}

	// Let the world know we have a new fixture. This will cause new contacts
	// to be created at the beginning of the next time step.
	m_world->m_flags |= cb2World::e_newFixture;

	return fixture;
}

cb2Fixture* cb2Body::CreateFixture(const cb2Shape* shape, float density)
{
	cb2FixtureDef def;
	def.shape = shape;
	def.density = density;

	return CreateFixture(&def);
}

void cb2Body::DestroyFixture(cb2Fixture* fixture)
{
	cb2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	cb2Assert(fixture->m_body == this);

	// Remove the fixture from this body's singly linked list.
	cb2Assert(m_fixtureCount > 0);
	cb2Fixture** node = &m_fixtureList;
	bool found = false;
	while (*node != NULL)
	{
		if (*node == fixture)
		{
			*node = fixture->m_next;
			found = true;
			break;
		}

		node = &(*node)->m_next;
	}

	// You tried to remove a shape that is not attached to this body.
	cb2Assert(found);

	// Destroy any contacts associated with the fixture.
	cb2ContactEdge* edge = m_contactList;
	while (edge)
	{
		cb2Contact* c = edge->contact;
		edge = edge->next;

		cb2Fixture* fixtureA = c->GetFixtureA();
		cb2Fixture* fixtureB = c->GetFixtureB();

		if (fixture == fixtureA || fixture == fixtureB)
		{
			// This destroys the contact and removes it from
			// this body's contact list.
			m_world->m_contactManager.Destroy(c);
		}
	}

	cb2BlockAllocator* allocator = &m_world->m_blockAllocator;

	if (m_flags & e_activeFlag)
	{
		cb2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		fixture->DestroyProxies(broadPhase);
	}

	fixture->Destroy(allocator);
	fixture->m_body = NULL;
	fixture->m_next = NULL;
	fixture->~cb2Fixture();
	allocator->Free(fixture, sizeof(cb2Fixture));

	--m_fixtureCount;

	// Reset the mass data.
	ResetMassData();
}

void cb2Body::ResetMassData()
{
	// Compute mass data from shapes. Each shape has its own density.
	m_mass = 0.0f;
	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;
	cb2::setZero(m_sweep.localCenter);

	// Static and kinematic bodies have zero mass.
	if (m_type == cb2_staticBody || m_type == cb2_kinematicBody)
	{
		m_sweep.c0 = m_xf.p;
		m_sweep.c = m_xf.p;
		m_sweep.a0 = m_sweep.a;
		return;
	}

	cb2Assert(m_type == cb2_dynamicBody);

	// Accumulate mass over all fixtures.
	ci::Vec2f localCenter = ci::Vec2f::zero();
	for (cb2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		if (f->m_density == 0.0f)
		{
			continue;
		}

		cb2MassData massData;
		f->GetMassData(&massData);
		m_mass += massData.mass;
		localCenter += massData.mass * massData.center;
		m_I += massData.I;
	}

	// Compute center of mass.
	if (m_mass > 0.0f)
	{
		m_invMass = 1.0f / m_mass;
		localCenter *= m_invMass;
	}
	else
	{
		// Force all dynamic bodies to have a positive mass.
		m_mass = 1.0f;
		m_invMass = 1.0f;
	}

	if (m_I > 0.0f && (m_flags & e_fixedRotationFlag) == 0)
	{
		// Center the inertia about the center of mass.
		m_I -= m_mass * cb2Dot(localCenter, localCenter);
		cb2Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;

	}
	else
	{
		m_I = 0.0f;
		m_invI = 0.0f;
	}

	// Move center of mass.
	ci::Vec2f oldCenter = m_sweep.c;
	m_sweep.localCenter = localCenter;
	m_sweep.c0 = m_sweep.c = cb2Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += cb2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

void cb2Body::SetMassData(const cb2MassData* massData)
{
	cb2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	if (m_type != cb2_dynamicBody)
	{
		return;
	}

	m_invMass = 0.0f;
	m_I = 0.0f;
	m_invI = 0.0f;

	m_mass = massData->mass;
	if (m_mass <= 0.0f)
	{
		m_mass = 1.0f;
	}

	m_invMass = 1.0f / m_mass;

	if (massData->I > 0.0f && (m_flags & cb2Body::e_fixedRotationFlag) == 0)
	{
		m_I = massData->I - m_mass * cb2Dot(massData->center, massData->center);
		cb2Assert(m_I > 0.0f);
		m_invI = 1.0f / m_I;
	}

	// Move center of mass.
	ci::Vec2f oldCenter = m_sweep.c;
	m_sweep.localCenter =  massData->center;
	m_sweep.c0 = m_sweep.c = cb2Mul(m_xf, m_sweep.localCenter);

	// Update center of mass velocity.
	m_linearVelocity += cb2Cross(m_angularVelocity, m_sweep.c - oldCenter);
}

bool cb2Body::ShouldCollide(const cb2Body* other) const
{
	// At least one body should be dynamic.
	if (m_type != cb2_dynamicBody && other->m_type != cb2_dynamicBody)
	{
		return false;
	}

	// Does a joint prevent collision?
	for (cb2JointEdge* jn = m_jointList; jn; jn = jn->next)
	{
		if (jn->other == other)
		{
			if (jn->joint->m_collideConnected == false)
			{
				return false;
			}
		}
	}

	return true;
}

void cb2Body::SetTransform(const ci::Vec2f& position, float angle)
{
	cb2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	m_xf.q.set(angle);
	m_xf.p = position;

	m_sweep.c = cb2Mul(m_xf, m_sweep.localCenter);
	m_sweep.a = angle;

	m_sweep.c0 = m_sweep.c;
	m_sweep.a0 = angle;

	cb2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (cb2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, m_xf, m_xf);
	}
}

void cb2Body::SetPosition(ci::Vec2f& position)
{
	cb2Assert(m_world->IsLocked() == false);
	if (m_world->IsLocked() == true)
	{
		return;
	}

	m_xf.p     = position;
	m_sweep.c  = cb2Mul(m_xf, m_sweep.localCenter);
	m_sweep.c0 = m_sweep.c;

	cb2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (cb2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, m_xf, m_xf);
	}
}

void cb2Body::SynchronizeFixtures()
{
	cb2Transform xf1;
	xf1.q.set(m_sweep.a0);
	xf1.p = m_sweep.c0 - cb2Mul(xf1.q, m_sweep.localCenter);

	cb2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
	for (cb2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		f->Synchronize(broadPhase, xf1, m_xf);
	}
}

void cb2Body::SetActive(bool flag)
{
	cb2Assert(m_world->IsLocked() == false);

	if (flag == IsActive())
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_activeFlag;

		// Create all proxies.
		cb2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		for (cb2Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->CreateProxies(broadPhase, m_xf);
		}

		// Contacts are created the next time step.
	}
	else
	{
		m_flags &= ~e_activeFlag;

		// Destroy all proxies.
		cb2BroadPhase* broadPhase = &m_world->m_contactManager.m_broadPhase;
		for (cb2Fixture* f = m_fixtureList; f; f = f->m_next)
		{
			f->DestroyProxies(broadPhase);
		}

		// Destroy the attached contacts.
		cb2ContactEdge* ce = m_contactList;
		while (ce)
		{
			cb2ContactEdge* ce0 = ce;
			ce = ce->next;
			m_world->m_contactManager.Destroy(ce0->contact);
		}
		m_contactList = NULL;
	}
}

void cb2Body::SetFixedRotation(bool flag)
{
	bool status = (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
	if (status == flag)
	{
		return;
	}

	if (flag)
	{
		m_flags |= e_fixedRotationFlag;
	}
	else
	{
		m_flags &= ~e_fixedRotationFlag;
	}

	m_angularVelocity = 0.0f;

	ResetMassData();
}

void cb2Body::Dump()
{
	int bodyIndex = m_islandIndex;

	cb2Log("{\n");
	cb2Log("  cb2BodyDef bd;\n");
	cb2Log("  bd.type = cb2BodyType(%d);\n", m_type);
	cb2Log("  bd.position.set(%.15lef, %.15lef);\n", m_xf.p.x, m_xf.p.y);
	cb2Log("  bd.angle = %.15lef;\n", m_sweep.a);
	cb2Log("  bd.linearVelocity.set(%.15lef, %.15lef);\n", m_linearVelocity.x, m_linearVelocity.y);
	cb2Log("  bd.angularVelocity = %.15lef;\n", m_angularVelocity);
	cb2Log("  bd.linearDamping = %.15lef;\n", m_linearDamping);
	cb2Log("  bd.angularDamping = %.15lef;\n", m_angularDamping);
	cb2Log("  bd.allowSleep = bool(%d);\n", m_flags & e_autoSleepFlag);
	cb2Log("  bd.awake = bool(%d);\n", m_flags & e_awakeFlag);
	cb2Log("  bd.fixedRotation = bool(%d);\n", m_flags & e_fixedRotationFlag);
	cb2Log("  bd.bullet = bool(%d);\n", m_flags & e_bulletFlag);
	cb2Log("  bd.active = bool(%d);\n", m_flags & e_activeFlag);
	cb2Log("  bd.gravityScale = %.15lef;\n", m_gravityScale);
	cb2Log("  bodies[%d] = m_world->CreateBody(&bd);\n", m_islandIndex);
	cb2Log("\n");
	for (cb2Fixture* f = m_fixtureList; f; f = f->m_next)
	{
		cb2Log("  {\n");
		f->Dump(bodyIndex);
		cb2Log("  }\n");
	}
	cb2Log("}\n");
}