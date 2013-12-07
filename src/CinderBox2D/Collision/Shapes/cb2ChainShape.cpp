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

#include <CinderBox2D/Collision/Shapes/cb2ChainShape.h>
#include <CinderBox2D/Collision/Shapes/cb2EdgeShape.h>
#include <new>
#include <memory.h>

cb2ChainShape::~cb2ChainShape()
{
	cb2Free(m_vertices);
	m_vertices = NULL;
	m_count = 0;
}

void cb2ChainShape::CreateLoop(const ci::Vec2f* vertices, int count)
{
	cb2Assert(m_vertices == NULL && m_count == 0);
	cb2Assert(count >= 3);
	for (int i = 1; i < count; ++i)
	{
		ci::Vec2f v1 = vertices[i-1];
		ci::Vec2f v2 = vertices[i];
		// If the code crashes here, it means your vertices are too close together.
		cb2Assert(cb2DistanceSquared(v1, v2) > cb2_linearSlop * cb2_linearSlop);
	}

	m_count = count + 1;
	m_vertices = (ci::Vec2f*)cb2Alloc(m_count * sizeof(ci::Vec2f));
	memcpy(m_vertices, vertices, count * sizeof(ci::Vec2f));
	m_vertices[count] = m_vertices[0];
	m_prevVertex = m_vertices[m_count - 2];
	m_nextVertex = m_vertices[1];
	m_hasPrevVertex = true;
	m_hasNextVertex = true;
}

void cb2ChainShape::CreateChain(const ci::Vec2f* vertices, int count)
{
	cb2Assert(m_vertices == NULL && m_count == 0);
	cb2Assert(count >= 2);
	for (int i = 1; i < count; ++i)
	{
		ci::Vec2f v1 = vertices[i-1];
		ci::Vec2f v2 = vertices[i];
		// If the code crashes here, it means your vertices are too close together.
		cb2Assert(cb2DistanceSquared(v1, v2) > cb2_linearSlop * cb2_linearSlop);
	}

	m_count = count;
	m_vertices = (ci::Vec2f*)cb2Alloc(count * sizeof(ci::Vec2f));
	memcpy(m_vertices, vertices, m_count * sizeof(ci::Vec2f));

	m_hasPrevVertex = false;
	m_hasNextVertex = false;

	cb2::setZero(m_prevVertex);
	cb2::setZero(m_nextVertex);
}

void cb2ChainShape::SetPrevVertex(const ci::Vec2f& prevVertex)
{
	m_prevVertex = prevVertex;
	m_hasPrevVertex = true;
}

void cb2ChainShape::SetNextVertex(const ci::Vec2f& nextVertex)
{
	m_nextVertex = nextVertex;
	m_hasNextVertex = true;
}

cb2Shape* cb2ChainShape::Clone(cb2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(cb2ChainShape));
	cb2ChainShape* clone = new (mem) cb2ChainShape;
	clone->CreateChain(m_vertices, m_count);
	clone->m_prevVertex = m_prevVertex;
	clone->m_nextVertex = m_nextVertex;
	clone->m_hasPrevVertex = m_hasPrevVertex;
	clone->m_hasNextVertex = m_hasNextVertex;
	return clone;
}

int cb2ChainShape::GetChildCount() const
{
	// edge count = vertex count - 1
	return m_count - 1;
}

void cb2ChainShape::GetChildEdge(cb2EdgeShape* edge, int index) const
{
	cb2Assert(0 <= index && index < m_count - 1);
	edge->m_type = cb2Shape::e_edge;
	edge->m_radius = m_radius;

	edge->m_vertex1 = m_vertices[index + 0];
	edge->m_vertex2 = m_vertices[index + 1];

	if (index > 0)
	{
		edge->m_vertex0 = m_vertices[index - 1];
		edge->m_hasVertex0 = true;
	}
	else
	{
		edge->m_vertex0 = m_prevVertex;
		edge->m_hasVertex0 = m_hasPrevVertex;
	}

	if (index < m_count - 2)
	{
		edge->m_vertex3 = m_vertices[index + 2];
		edge->m_hasVertex3 = true;
	}
	else
	{
		edge->m_vertex3 = m_nextVertex;
		edge->m_hasVertex3 = m_hasNextVertex;
	}
}

bool cb2ChainShape::TestPoint(const cb2Transform& xf, const ci::Vec2f& p) const
{
	CB2_NOT_USED(xf);
	CB2_NOT_USED(p);
	return false;
}

bool cb2ChainShape::RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input,
							const cb2Transform& xf, int childIndex) const
{
	cb2Assert(childIndex < m_count);

	cb2EdgeShape edgeShape;

	int i1 = childIndex;
	int i2 = childIndex + 1;
	if (i2 == m_count)
	{
		i2 = 0;
	}

	edgeShape.m_vertex1 = m_vertices[i1];
	edgeShape.m_vertex2 = m_vertices[i2];

	return edgeShape.RayCast(output, input, xf, 0);
}

void cb2ChainShape::ComputeAABB(cb2AABB* aabb, const cb2Transform& xf, int childIndex) const
{
	cb2Assert(childIndex < m_count);

	int i1 = childIndex;
	int i2 = childIndex + 1;
	if (i2 == m_count)
	{
		i2 = 0;
	}

	ci::Vec2f v1 = cb2Mul(xf, m_vertices[i1]);
	ci::Vec2f v2 = cb2Mul(xf, m_vertices[i2]);

	aabb->lowerBound = cb2Min(v1, v2);
	aabb->upperBound = cb2Max(v1, v2);
}

void cb2ChainShape::ComputeMass(cb2MassData* massData, float density) const
{
	CB2_NOT_USED(density);

	massData->mass = 0.0f;
	cb2::setZero(massData->center);
	massData->I = 0.0f;
}
