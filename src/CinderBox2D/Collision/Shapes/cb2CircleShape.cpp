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

#include <CinderBox2D/Collision/Shapes/cb2CircleShape.h>
#include <new>

cb2Shape* cb2CircleShape::Clone(cb2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(cb2CircleShape));
	cb2CircleShape* clone = new (mem) cb2CircleShape;
	*clone = *this;
	return clone;
}

int cb2CircleShape::GetChildCount() const
{
	return 1;
}

bool cb2CircleShape::TestPoint(const cb2Transform& transform, const ci::Vec2f& p) const
{
	ci::Vec2f center = transform.p + cb2Mul(transform.q, m_p);
	ci::Vec2f d = p - center;
	return cb2Dot(d, d) <= m_radius * m_radius;
}

// Collision Detection in Interactive 3D Environments by Gino van den Bergen
// From Section 3.1.2
// x = s + a * r
// norm(x) = radius
bool cb2CircleShape::RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input,
							const cb2Transform& transform, int childIndex) const
{
	CB2_NOT_USED(childIndex);

	ci::Vec2f position = transform.p + cb2Mul(transform.q, m_p);
	ci::Vec2f s = input.p1 - position;
	float b = cb2Dot(s, s) - m_radius * m_radius;

	// Solve quadratic equation.
	ci::Vec2f r = input.p2 - input.p1;
	float c =  cb2Dot(s, r);
	float rr = cb2Dot(r, r);
	float sigma = c * c - rr * b;

	// Check for negative discriminant and short segment.
	if (sigma < 0.0f || rr < cb2_epsilon)
	{
		return false;
	}

	// Find the point of intersection of the line with the circle.
	float a = -(c + cb2Sqrt(sigma));

	// Is the intersection point on the segment?
	if (0.0f <= a && a <= input.maxFraction * rr)
	{
		a /= rr;
		output->fraction = a;
		output->normal = s + a * r;
		output->normal.normalize();
		return true;
	}

	return false;
}

void cb2CircleShape::ComputeAABB(cb2AABB* aabb, const cb2Transform& transform, int childIndex) const
{
	CB2_NOT_USED(childIndex);

	ci::Vec2f p = transform.p + cb2Mul(transform.q, m_p);
	aabb->lowerBound.set(p.x - m_radius, p.y - m_radius);
	aabb->upperBound.set(p.x + m_radius, p.y + m_radius);
}

void cb2CircleShape::ComputeMass(cb2MassData* massData, float density) const
{
	massData->mass = density * cb2_pi * m_radius * m_radius;
	massData->center = m_p;

	// inertia about the local origin
	massData->I = massData->mass * (0.5f * m_radius * m_radius + cb2Dot(m_p, m_p));
}
