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

#include <CinderBox2D/Collision/Shapes/cb2EdgeShape.h>
#include <new>

void cb2EdgeShape::Set(const ci::Vec2f& v1, const ci::Vec2f& v2)
{
	m_vertex1 = v1;
	m_vertex2 = v2;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}

cb2Shape* cb2EdgeShape::Clone(cb2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(cb2EdgeShape));
	cb2EdgeShape* clone = new (mem) cb2EdgeShape;
	*clone = *this;
	return clone;
}

int cb2EdgeShape::GetChildCount() const
{
	return 1;
}

bool cb2EdgeShape::TestPoint(const cb2Transform& xf, const ci::Vec2f& p) const
{
	CB2_NOT_USED(xf);
	CB2_NOT_USED(p);
	return false;
}

// p = p1 + t * d
// v = v1 + s * e
// p1 + t * d = v1 + s * e
// s * e - t * d = p1 - v1
bool cb2EdgeShape::RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input,
							const cb2Transform& xf, int childIndex) const
{
	CB2_NOT_USED(childIndex);

	// Put the ray into the edge's frame of reference.
	ci::Vec2f p1 = cb2MulT(xf.q, input.p1 - xf.p);
	ci::Vec2f p2 = cb2MulT(xf.q, input.p2 - xf.p);
	ci::Vec2f d = p2 - p1;

	ci::Vec2f v1 = m_vertex1;
	ci::Vec2f v2 = m_vertex2;
	ci::Vec2f e = v2 - v1;
	ci::Vec2f normal(e.y, -e.x);
	normal.normalize();

	// q = p1 + t * d
	// dot(normal, q - v1) = 0
	// dot(normal, p1 - v1) + t * dot(normal, d) = 0
	float numerator = cb2Dot(normal, v1 - p1);
	float denominator = cb2Dot(normal, d);

	if (denominator == 0.0f)
	{
		return false;
	}

	float t = numerator / denominator;
	if (t < 0.0f || input.maxFraction < t)
	{
		return false;
	}

	ci::Vec2f q = p1 + t * d;

	// q = v1 + s * r
	// s = dot(q - v1, r) / dot(r, r)
	ci::Vec2f r = v2 - v1;
	float rr = cb2Dot(r, r);
	if (rr == 0.0f)
	{
		return false;
	}

	float s = cb2Dot(q - v1, r) / rr;
	if (s < 0.0f || 1.0f < s)
	{
		return false;
	}

	output->fraction = t;
	if (numerator > 0.0f)
	{
		output->normal = -cb2Mul(xf.q, normal);
	}
	else
	{
		output->normal = cb2Mul(xf.q, normal);
	}
	return true;
}

void cb2EdgeShape::ComputeAABB(cb2AABB* aabb, const cb2Transform& xf, int childIndex) const
{
	CB2_NOT_USED(childIndex);

	ci::Vec2f v1 = cb2Mul(xf, m_vertex1);
	ci::Vec2f v2 = cb2Mul(xf, m_vertex2);

	ci::Vec2f lower = cb2Min(v1, v2);
	ci::Vec2f upper = cb2Max(v1, v2);

	ci::Vec2f r(m_radius, m_radius);
	aabb->lowerBound = lower - r;
	aabb->upperBound = upper + r;
}

void cb2EdgeShape::ComputeMass(cb2MassData* massData, float density) const
{
	CB2_NOT_USED(density);

	massData->mass = 0.0f;
	massData->center = 0.5f * (m_vertex1 + m_vertex2);
	massData->I = 0.0f;
}
