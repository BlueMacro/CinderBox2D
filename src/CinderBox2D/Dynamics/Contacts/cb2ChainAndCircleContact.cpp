/*
* Copyright (c) 2006-2010 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Dynamics/Contacts/cb2ChainAndCircleContact.h>
#include <CinderBox2D/Common/cb2BlockAllocator.h>
#include <CinderBox2D/Dynamics/cb2Fixture.h>
#include <CinderBox2D/Collision/Shapes/cb2ChainShape.h>
#include <CinderBox2D/Collision/Shapes/cb2EdgeShape.h>

#include <new>

cb2Contact* cb2ChainAndCircleContact::Create(cb2Fixture* fixtureA, int indexA, cb2Fixture* fixtureB, int indexB, cb2BlockAllocator* allocator)
{
	void* mem = allocator->Allocate(sizeof(cb2ChainAndCircleContact));
	return new (mem) cb2ChainAndCircleContact(fixtureA, indexA, fixtureB, indexB);
}

void cb2ChainAndCircleContact::Destroy(cb2Contact* contact, cb2BlockAllocator* allocator)
{
	((cb2ChainAndCircleContact*)contact)->~cb2ChainAndCircleContact();
	allocator->Free(contact, sizeof(cb2ChainAndCircleContact));
}

cb2ChainAndCircleContact::cb2ChainAndCircleContact(cb2Fixture* fixtureA, int indexA, cb2Fixture* fixtureB, int indexB)
: cb2Contact(fixtureA, indexA, fixtureB, indexB)
{
	cb2Assert(m_fixtureA->GetType() == cb2Shape::e_chain);
	cb2Assert(m_fixtureB->GetType() == cb2Shape::e_circle);
}

void cb2ChainAndCircleContact::Evaluate(cb2Manifold* manifold, const cb2Transform& xfA, const cb2Transform& xfB)
{
	cb2ChainShape* chain = (cb2ChainShape*)m_fixtureA->GetShape();
	cb2EdgeShape edge;
	chain->GetChildEdge(&edge, m_indexA);
	cb2CollideEdgeAndCircle(	manifold, &edge, xfA,
							(cb2CircleShape*)m_fixtureB->GetShape(), xfB);
}
