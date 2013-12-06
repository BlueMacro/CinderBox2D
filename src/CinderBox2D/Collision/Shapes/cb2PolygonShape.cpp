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

#include <CinderBox2D/Collision/Shapes/cb2PolygonShape.h>
#include <new>

b2Shape* b2PolygonShape::Clone(b2BlockAllocator* allocator) const
{
	void* mem = allocator->Allocate(sizeof(b2PolygonShape));
	b2PolygonShape* clone = new (mem) b2PolygonShape;
	*clone = *this;
	return clone;
}

void b2PolygonShape::SetAsBox(float hx, float hy)
{
	m_vertexCount = 4;
	m_vertices[0].set(-hx, -hy);
	m_vertices[1].set( hx, -hy);
	m_vertices[2].set( hx,  hy);
	m_vertices[3].set(-hx,  hy);
	m_normals[0].set(0.0f, -1.0f);
	m_normals[1].set(1.0f, 0.0f);
	m_normals[2].set(0.0f, 1.0f);
	m_normals[3].set(-1.0f, 0.0f);
	cb2::setZero(m_centroid);
}

void b2PolygonShape::SetAsBox(float hx, float hy, const ci::Vec2f& center, float angle)
{
	m_vertexCount = 4;
	m_vertices[0].set(-hx, -hy);
	m_vertices[1].set( hx, -hy);
	m_vertices[2].set( hx,  hy);
	m_vertices[3].set(-hx,  hy);
	m_normals[0].set(0.0f, -1.0f);
	m_normals[1].set(1.0f, 0.0f);
	m_normals[2].set(0.0f, 1.0f);
	m_normals[3].set(-1.0f, 0.0f);
	m_centroid = center;

	b2Transform xf;
	xf.p = center;
	xf.q.set(angle);

	// Transform vertices and normals.
	for (int i = 0; i < m_vertexCount; ++i)
	{
		m_vertices[i] = b2Mul(xf, m_vertices[i]);
		m_normals[i] = b2Mul(xf.q, m_normals[i]);
	}
}

int b2PolygonShape::GetChildCount() const
{
	return 1;
}

static ci::Vec2f ComputeCentroid(const ci::Vec2f* vs, int count)
{
	b2Assert(count >= 3);

	ci::Vec2f c; c.set(0.0f, 0.0f);
	float area = 0.0f;

	// pRef is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	ci::Vec2f pRef(0.0f, 0.0f);
#if 0
	// This code would put the reference point inside the polygon.
	for (int i = 0; i < count; ++i)
	{
		pRef += vs[i];
	}
	pRef *= 1.0f / count;
#endif

	const float inv3 = 1.0f / 3.0f;

	for (int i = 0; i < count; ++i)
	{
		// Triangle vertices.
		ci::Vec2f p1 = pRef;
		ci::Vec2f p2 = vs[i];
		ci::Vec2f p3 = i + 1 < count ? vs[i+1] : vs[0];

		ci::Vec2f e1 = p2 - p1;
		ci::Vec2f e2 = p3 - p1;

		float D = b2Cross(e1, e2);

		float triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		c += triangleArea * inv3 * (p1 + p2 + p3);
	}

	// Centroid
	b2Assert(area > b2_epsilon);
	c *= 1.0f / area;
	return c;
}

void b2PolygonShape::set(const ci::Vec2f* vertices, int count)
{
	b2Assert(3 <= count && count <= b2_maxPolygonVertices);
	m_vertexCount = count;

	// Copy vertices.
	for (int i = 0; i < m_vertexCount; ++i)
	{
		m_vertices[i] = vertices[i];
	}

	// Compute normals. Ensure the edges have non-zero length.
	for (int i = 0; i < m_vertexCount; ++i)
	{
		int i1 = i;
		int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
		ci::Vec2f edge = m_vertices[i2] - m_vertices[i1];
		b2Assert(edge.lengthSquared() > b2_epsilon * b2_epsilon);
		m_normals[i] = b2Cross(edge, 1.0f);
		m_normals[i].normalize();
	}

#ifdef _DEBUG
	// Ensure the polygon is convex and the interior
	// is to the left of each edge.
	for (int i = 0; i < m_vertexCount; ++i)
	{
		int i1 = i;
		int i2 = i + 1 < m_vertexCount ? i + 1 : 0;
		ci::Vec2f edge = m_vertices[i2] - m_vertices[i1];

		for (int j = 0; j < m_vertexCount; ++j)
		{
			// Don't check vertices on the current edge.
			if (j == i1 || j == i2)
			{
				continue;
			}
			
			ci::Vec2f r = m_vertices[j] - m_vertices[i1];

			// If this crashes, your polygon is non-convex, has colinear edges,
			// or the winding order is wrong.
			float s = b2Cross(edge, r);
			b2Assert(s > 0.0f && "ERROR: Please ensure your polygon is convex and has a CCW winding order");
		}
	}
#endif

	// Compute the polygon centroid.
	m_centroid = ComputeCentroid(m_vertices, m_vertexCount);
}

bool b2PolygonShape::TestPoint(const b2Transform& xf, const ci::Vec2f& p) const
{
	ci::Vec2f pLocal = b2MulT(xf.q, p - xf.p);

	for (int i = 0; i < m_vertexCount; ++i)
	{
		float dot = b2Dot(m_normals[i], pLocal - m_vertices[i]);
		if (dot > 0.0f)
		{
			return false;
		}
	}

	return true;
}

bool b2PolygonShape::RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
								const b2Transform& xf, int childIndex) const
{
	B2_NOT_USED(childIndex);

	// Put the ray into the polygon's frame of reference.
	ci::Vec2f p1 = b2MulT(xf.q, input.p1 - xf.p);
	ci::Vec2f p2 = b2MulT(xf.q, input.p2 - xf.p);
	ci::Vec2f d = p2 - p1;

	float lower = 0.0f, upper = input.maxFraction;

	int index = -1;

	for (int i = 0; i < m_vertexCount; ++i)
	{
		// p = p1 + a * d
		// dot(normal, p - v) = 0
		// dot(normal, p1 - v) + a * dot(normal, d) = 0
		float numerator = b2Dot(m_normals[i], m_vertices[i] - p1);
		float denominator = b2Dot(m_normals[i], d);

		if (denominator == 0.0f)
		{	
			if (numerator < 0.0f)
			{
				return false;
			}
		}
		else
		{
			// Note: we want this predicate without division:
			// lower < numerator / denominator, where denominator < 0
			// Since denominator < 0, we have to flip the inequality:
			// lower < numerator / denominator <==> denominator * lower > numerator.
			if (denominator < 0.0f && numerator < lower * denominator)
			{
				// Increase lower.
				// The segment enters this half-space.
				lower = numerator / denominator;
				index = i;
			}
			else if (denominator > 0.0f && numerator < upper * denominator)
			{
				// Decrease upper.
				// The segment exits this half-space.
				upper = numerator / denominator;
			}
		}

		// The use of epsilon here causes the assert on lower to trip
		// in some cases. Apparently the use of epsilon was to make edge
		// shapes work, but now those are handled separately.
		//if (upper < lower - b2_epsilon)
		if (upper < lower)
		{
			return false;
		}
	}

	b2Assert(0.0f <= lower && lower <= input.maxFraction);

	if (index >= 0)
	{
		output->fraction = lower;
		output->normal = b2Mul(xf.q, m_normals[index]);
		return true;
	}

	return false;
}

void b2PolygonShape::ComputeAABB(b2AABB* aabb, const b2Transform& xf, int childIndex) const
{
	B2_NOT_USED(childIndex);

	ci::Vec2f lower = b2Mul(xf, m_vertices[0]);
	ci::Vec2f upper = lower;

	for (int i = 1; i < m_vertexCount; ++i)
	{
		ci::Vec2f v = b2Mul(xf, m_vertices[i]);
		lower = b2Min(lower, v);
		upper = b2Max(upper, v);
	}

	ci::Vec2f r(m_radius, m_radius);
	aabb->lowerBound = lower - r;
	aabb->upperBound = upper + r;
}

void b2PolygonShape::ComputeMass(b2MassData* massData, float density) const
{
	// Polygon mass, centroid, and inertia.
	// Let rho be the polygon density in mass per unit area.
	// Then:
	// mass = rho * int(dA)
	// centroid.x = (1/mass) * rho * int(x * dA)
	// centroid.y = (1/mass) * rho * int(y * dA)
	// I = rho * int((x*x + y*y) * dA)
	//
	// We can compute these integrals by summing all the integrals
	// for each triangle of the polygon. To evaluate the integral
	// for a single triangle, we make a change of variables to
	// the (u,v) coordinates of the triangle:
	// x = x0 + e1x * u + e2x * v
	// y = y0 + e1y * u + e2y * v
	// where 0 <= u && 0 <= v && u + v <= 1.
	//
	// We integrate u from [0,1-v] and then v from [0,1].
	// We also need to use the Jacobian of the transformation:
	// D = cross(e1, e2)
	//
	// Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
	//
	// The rest of the derivation is handled by computer algebra.

	b2Assert(m_vertexCount >= 3);

	ci::Vec2f center; center.set(0.0f, 0.0f);
	float area = 0.0f;
	float I = 0.0f;

	// s is the reference point for forming triangles.
	// It's location doesn't change the result (except for rounding error).
	ci::Vec2f s(0.0f, 0.0f);

	// This code would put the reference point inside the polygon.
	for (int i = 0; i < m_vertexCount; ++i)
	{
		s += m_vertices[i];
	}
	s *= 1.0f / m_vertexCount;

	const float k_inv3 = 1.0f / 3.0f;

	for (int i = 0; i < m_vertexCount; ++i)
	{
		// Triangle vertices.
		ci::Vec2f e1 = m_vertices[i] - s;
		ci::Vec2f e2 = i + 1 < m_vertexCount ? m_vertices[i+1] - s : m_vertices[0] - s;

		float D = b2Cross(e1, e2);

		float triangleArea = 0.5f * D;
		area += triangleArea;

		// Area weighted centroid
		center += triangleArea * k_inv3 * (e1 + e2);

		float ex1 = e1.x, ey1 = e1.y;
		float ex2 = e2.x, ey2 = e2.y;

		float intx2 = ex1*ex1 + ex2*ex1 + ex2*ex2;
		float inty2 = ey1*ey1 + ey2*ey1 + ey2*ey2;

		I += (0.25f * k_inv3 * D) * (intx2 + inty2);
	}

	// Total mass
	massData->mass = density * area;

	// Center of mass
	b2Assert(area > b2_epsilon);
	center *= 1.0f / area;
	massData->center = center + s;

	// Inertia tensor relative to the local origin (point s).
	massData->I = density * I;
	
	// Shift to center of mass then to original body origin.
	massData->I += massData->mass * (b2Dot(massData->center, massData->center) - b2Dot(center, center));
}
