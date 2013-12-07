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

#include <CinderBox2D/Dynamics/cb2Fixture.h>
#include <CinderBox2D/Dynamics/Contacts/cb2Contact.h>
#include <CinderBox2D/Dynamics/cb2World.h>
#include <CinderBox2D/Collision/Shapes/cb2CircleShape.h>
#include <CinderBox2D/Collision/Shapes/cb2EdgeShape.h>
#include <CinderBox2D/Collision/Shapes/cb2PolygonShape.h>
#include <CinderBox2D/Collision/Shapes/cb2ChainShape.h>
#include <CinderBox2D/Collision/cb2BroadPhase.h>
#include <CinderBox2D/Collision/cb2Collision.h>
#include <CinderBox2D/Common/cb2BlockAllocator.h>

cb2Fixture::cb2Fixture()
{
	m_userData = NULL;
	m_body = NULL;
	m_next = NULL;
	m_proxies = NULL;
	m_proxyCount = 0;
	m_shape = NULL;
	m_density = 0.0f;
}

void cb2Fixture::Create(cb2BlockAllocator* allocator, cb2Body* body, const cb2FixtureDef* def)
{
	m_userData = def->userData;
	m_friction = def->friction;
	m_restitution = def->restitution;

	m_body = body;
	m_next = NULL;

	m_filter = def->filter;

	m_isSensor = def->isSensor;

	m_shape = def->shape->Clone(allocator);

	// Reserve proxy space
	int childCount = m_shape->GetChildCount();
	m_proxies = (cb2FixtureProxy*)allocator->Allocate(childCount * sizeof(cb2FixtureProxy));
	for (int i = 0; i < childCount; ++i)
	{
		m_proxies[i].fixture = NULL;
		m_proxies[i].proxyId = cb2BroadPhase::e_nullProxy;
	}
	m_proxyCount = 0;

	m_density = def->density;
}

void cb2Fixture::Destroy(cb2BlockAllocator* allocator)
{
	// The proxies must be destroyed before calling this.
	cb2Assert(m_proxyCount == 0);

	// Free the proxy array.
	int childCount = m_shape->GetChildCount();
	allocator->Free(m_proxies, childCount * sizeof(cb2FixtureProxy));
	m_proxies = NULL;

	// Free the child shape.
	switch (m_shape->m_type)
	{
	case cb2Shape::e_circle:
		{
			cb2CircleShape* s = (cb2CircleShape*)m_shape;
			s->~cb2CircleShape();
			allocator->Free(s, sizeof(cb2CircleShape));
		}
		break;

	case cb2Shape::e_edge:
		{
			cb2EdgeShape* s = (cb2EdgeShape*)m_shape;
			s->~cb2EdgeShape();
			allocator->Free(s, sizeof(cb2EdgeShape));
		}
		break;

	case cb2Shape::e_polygon:
		{
			cb2PolygonShape* s = (cb2PolygonShape*)m_shape;
			s->~cb2PolygonShape();
			allocator->Free(s, sizeof(cb2PolygonShape));
		}
		break;

	case cb2Shape::e_chain:
		{
			cb2ChainShape* s = (cb2ChainShape*)m_shape;
			s->~cb2ChainShape();
			allocator->Free(s, sizeof(cb2ChainShape));
		}
		break;

	default:
		cb2Assert(false);
		break;
	}

	m_shape = NULL;
}

void cb2Fixture::CreateProxies(cb2BroadPhase* broadPhase, const cb2Transform& xf)
{
	cb2Assert(m_proxyCount == 0);

	// Create proxies in the broad-phase.
	m_proxyCount = m_shape->GetChildCount();

	for (int i = 0; i < m_proxyCount; ++i)
	{
		cb2FixtureProxy* proxy = m_proxies + i;
		m_shape->ComputeAABB(&proxy->aabb, xf, i);
		proxy->proxyId = broadPhase->CreateProxy(proxy->aabb, proxy);
		proxy->fixture = this;
		proxy->childIndex = i;
	}
}

void cb2Fixture::DestroyProxies(cb2BroadPhase* broadPhase)
{
	// Destroy proxies in the broad-phase.
	for (int i = 0; i < m_proxyCount; ++i)
	{
		cb2FixtureProxy* proxy = m_proxies + i;
		broadPhase->DestroyProxy(proxy->proxyId);
		proxy->proxyId = cb2BroadPhase::e_nullProxy;
	}

	m_proxyCount = 0;
}

void cb2Fixture::Synchronize(cb2BroadPhase* broadPhase, const cb2Transform& transform1, const cb2Transform& transform2)
{
	if (m_proxyCount == 0)
	{	
		return;
	}

	for (int i = 0; i < m_proxyCount; ++i)
	{
		cb2FixtureProxy* proxy = m_proxies + i;

		// Compute an AABB that covers the swept shape (may miss some rotation effect).
		cb2AABB aabb1, aabb2;
		m_shape->ComputeAABB(&aabb1, transform1, proxy->childIndex);
		m_shape->ComputeAABB(&aabb2, transform2, proxy->childIndex);
	
		proxy->aabb.Combine(aabb1, aabb2);

		ci::Vec2f displacement = transform2.p - transform1.p;

		broadPhase->MoveProxy(proxy->proxyId, proxy->aabb, displacement);
	}
}

void cb2Fixture::SetFilterData(const cb2Filter& filter)
{
	m_filter = filter;

	Refilter();
}

void cb2Fixture::Refilter()
{
	if (m_body == NULL)
	{
		return;
	}

	// Flag associated contacts for filtering.
	cb2ContactEdge* edge = m_body->GetContactList();
	while (edge)
	{
		cb2Contact* contact = edge->contact;
		cb2Fixture* fixtureA = contact->GetFixtureA();
		cb2Fixture* fixtureB = contact->GetFixtureB();
		if (fixtureA == this || fixtureB == this)
		{
			contact->FlagForFiltering();
		}

		edge = edge->next;
	}

	cb2World* world = m_body->GetWorld();

	if (world == NULL)
	{
		return;
	}

	// Touch each proxy so that new pairs may be created
	cb2BroadPhase* broadPhase = &world->m_contactManager.m_broadPhase;
	for (int i = 0; i < m_proxyCount; ++i)
	{
		broadPhase->TouchProxy(m_proxies[i].proxyId);
	}
}

void cb2Fixture::SetSensor(bool sensor)
{
	if (sensor != m_isSensor)
	{
		m_body->SetAwake(true);
		m_isSensor = sensor;
	}
}

void cb2Fixture::Dump(int bodyIndex)
{
	cb2Log("    cb2FixtureDef fd;\n");
	cb2Log("    fd.friction = %.15lef;\n", m_friction);
	cb2Log("    fd.restitution = %.15lef;\n", m_restitution);
	cb2Log("    fd.density = %.15lef;\n", m_density);
	cb2Log("    fd.isSensor = bool(%d);\n", m_isSensor);
	cb2Log("    fd.filter.categoryBits = unsigned short(%d);\n", m_filter.categoryBits);
	cb2Log("    fd.filter.maskBits = unsigned short(%d);\n", m_filter.maskBits);
	cb2Log("    fd.filter.groupIndex = short(%d);\n", m_filter.groupIndex);

	switch (m_shape->m_type)
	{
	case cb2Shape::e_circle:
		{
			cb2CircleShape* s = (cb2CircleShape*)m_shape;
			cb2Log("    cb2CircleShape shape;\n");
			cb2Log("    shape.m_radius = %.15lef;\n", s->m_radius);
			cb2Log("    shape.m_p.set(%.15lef, %.15lef);\n", s->m_p.x, s->m_p.y);
		}
		break;

	case cb2Shape::e_edge:
		{
			cb2EdgeShape* s = (cb2EdgeShape*)m_shape;
			cb2Log("    cb2EdgeShape shape;\n");
			cb2Log("    shape.m_radius = %.15lef;\n", s->m_radius);
			cb2Log("    shape.m_vertex0.set(%.15lef, %.15lef);\n", s->m_vertex0.x, s->m_vertex0.y);
			cb2Log("    shape.m_vertex1.set(%.15lef, %.15lef);\n", s->m_vertex1.x, s->m_vertex1.y);
			cb2Log("    shape.m_vertex2.set(%.15lef, %.15lef);\n", s->m_vertex2.x, s->m_vertex2.y);
			cb2Log("    shape.m_vertex3.set(%.15lef, %.15lef);\n", s->m_vertex3.x, s->m_vertex3.y);
			cb2Log("    shape.m_hasVertex0 = bool(%d);\n", s->m_hasVertex0);
			cb2Log("    shape.m_hasVertex3 = bool(%d);\n", s->m_hasVertex3);
		}
		break;

	case cb2Shape::e_polygon:
		{
			cb2PolygonShape* s = (cb2PolygonShape*)m_shape;
			cb2Log("    cb2PolygonShape shape;\n");
			cb2Log("    ci::Vec2f vs[%d];\n", cb2_maxPolygonVertices);
			for (int i = 0; i < s->m_count; ++i)
			{
				cb2Log("    vs[%d].set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
			}
			cb2Log("    shape.set(vs, %d);\n", s->m_count);
		}
		break;

	case cb2Shape::e_chain:
		{
			cb2ChainShape* s = (cb2ChainShape*)m_shape;
			cb2Log("    cb2ChainShape shape;\n");
			cb2Log("    ci::Vec2f vs[%d];\n", s->m_count);
			for (int i = 0; i < s->m_count; ++i)
			{
				cb2Log("    vs[%d].set(%.15lef, %.15lef);\n", i, s->m_vertices[i].x, s->m_vertices[i].y);
			}
			cb2Log("    shape.CreateChain(vs, %d);\n", s->m_count);
			cb2Log("    shape.m_prevVertex.set(%.15lef, %.15lef);\n", s->m_prevVertex.x, s->m_prevVertex.y);
			cb2Log("    shape.m_nextVertex.set(%.15lef, %.15lef);\n", s->m_nextVertex.x, s->m_nextVertex.y);
			cb2Log("    shape.m_hasPrevVertex = bool(%d);\n", s->m_hasPrevVertex);
			cb2Log("    shape.m_hasNextVertex = bool(%d);\n", s->m_hasNextVertex);
		}
		break;

	default:
		return;
	}

	cb2Log("\n");
	cb2Log("    fd.shape = &shape;\n");
	cb2Log("\n");
	cb2Log("    bodies[%d]->CreateFixture(&fd);\n", bodyIndex);
}
