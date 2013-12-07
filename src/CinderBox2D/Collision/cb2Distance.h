
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

#ifndef CB2_DISTANCE_H
#define CB2_DISTANCE_H

#include <CinderBox2D/Common/cb2Math.h>

class cb2Shape;

/// A distance proxy is used by the GJK algorithm.
/// It encapsulates any shape.
struct cb2DistanceProxy
{
	cb2DistanceProxy() : m_vertices(NULL), m_count(0), m_radius(0.0f) {}

	/// Initialize the proxy using the given shape. The shape
	/// must remain in scope while the proxy is in use.
	void set(const cb2Shape* shape, int index);

	/// Get the supporting vertex index in the given direction.
	int GetSupport(const ci::Vec2f& d) const;

	/// Get the supporting vertex in the given direction.
	const ci::Vec2f& GetSupportVertex(const ci::Vec2f& d) const;

	/// Get the vertex count.
	int GetVertexCount() const;

	/// Get a vertex by index. Used by cb2Distance.
	const ci::Vec2f& GetVertex(int index) const;

	ci::Vec2f m_buffer[2];
	const ci::Vec2f* m_vertices;
	int m_count;
	float m_radius;
};

/// Used to warm start cb2Distance.
/// set count to zero on first call.
struct cb2SimplexCache
{
	float metric;		///< length or area
	unsigned short count;
	unsigned char indexA[3];	///< vertices on shape A
	unsigned char indexB[3];	///< vertices on shape B
};

/// Input for cb2Distance.
/// You have to option to use the shape radii
/// in the computation. Even 
struct cb2DistanceInput
{
	cb2DistanceProxy proxyA;
	cb2DistanceProxy proxyB;
	cb2Transform transformA;
	cb2Transform transformB;
	bool useRadii;
};

/// Output for cb2Distance.
struct cb2DistanceOutput
{
	ci::Vec2f pointA;		///< closest point on shapeA
	ci::Vec2f pointB;		///< closest point on shapeB
	float distance;
	int iterations;	///< number of GJK iterations used
};

/// Compute the closest points between two shapes. Supports any combination of:
/// cb2CircleShape, cb2PolygonShape, cb2EdgeShape. The simplex cache is input/output.
/// On the first call set cb2SimplexCache.count to zero.
void cb2Distance(cb2DistanceOutput* output,
				cb2SimplexCache* cache, 
				const cb2DistanceInput* input);


//////////////////////////////////////////////////////////////////////////

inline int cb2DistanceProxy::GetVertexCount() const
{
	return m_count;
}

inline const ci::Vec2f& cb2DistanceProxy::GetVertex(int index) const
{
	cb2Assert(0 <= index && index < m_count);
	return m_vertices[index];
}

inline int cb2DistanceProxy::GetSupport(const ci::Vec2f& d) const
{
	int bestIndex = 0;
	float bestValue = cb2Dot(m_vertices[0], d);
	for (int i = 1; i < m_count; ++i)
	{
		float value = cb2Dot(m_vertices[i], d);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return bestIndex;
}

inline const ci::Vec2f& cb2DistanceProxy::GetSupportVertex(const ci::Vec2f& d) const
{
	int bestIndex = 0;
	float bestValue = cb2Dot(m_vertices[0], d);
	for (int i = 1; i < m_count; ++i)
	{
		float value = cb2Dot(m_vertices[i], d);
		if (value > bestValue)
		{
			bestIndex = i;
			bestValue = value;
		}
	}

	return m_vertices[bestIndex];
}

#endif
