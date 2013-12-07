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

#include <CinderBox2D/Dynamics/Contacts/cb2CircleContact.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2Fixture.h>
#include <CinderBox2D/Dynamics/cb2WorldCallbacks.h>
#include <CinderBox2D/Common/cb2BlockAllocator.h>
#include <CinderBox2D/Collision/cb2TimeOfImpact.h>

#include <new>

cb2Contact* cb2CircleContact::Create(cb2Fixture* fixtureA, int, cb2Fixture* fixtureB, int, cb2BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(cb2CircleContact));
	return new (mem) cb2CircleContact(fixtureA, fixtureB);
}

void cb2CircleContact::Destroy(cb2Contact* contact, cb2BlockAllocator* allocator)
{
	((cb2CircleContact*)contact)->~cb2CircleContact();
	allocator->Free(contact, sizeof(cb2CircleContact));
}

cb2CircleContact::cb2CircleContact(cb2Fixture* fixtureA, cb2Fixture* fixtureB)
	: cb2Contact(fixtureA, 0, fixtureB, 0)
{
	cb2Assert(m_fixtureA->GetType() == cb2Shape::e_circle);
	cb2Assert(m_fixtureB->GetType() == cb2Shape::e_circle);
}

void cb2CircleContact::Evaluate(cb2Manifold* manifold, const cb2Transform& xfA, const cb2Transform& xfB)
{
	cb2CollideCircles(manifold,
					(cb2CircleShape*)m_fixtureA->GetShape(), xfA,
					(cb2CircleShape*)m_fixtureB->GetShape(), xfB);
}
