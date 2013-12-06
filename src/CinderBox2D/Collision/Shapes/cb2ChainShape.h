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

#ifndef B2_CHAIN_SHAPE_H
#define B2_CHAIN_SHAPE_H

#include <CinderBox2D/Collision/Shapes/cb2Shape.h>

class b2EdgeShape;

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using b2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
class b2ChainShape : public b2Shape
{
public:
	b2ChainShape();

	/// The destructor frees the vertices using b2Free.
	~b2ChainShape();

	/// Create a loop. This automatically adjusts connectivity.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	void CreateLoop(const ci::Vec2f* vertices, int count);

	/// Create a chain with isolated end vertices.
	/// @param vertices an array of vertices, these are copied
	/// @param count the vertex count
	void CreateChain(const ci::Vec2f* vertices, int count);

	/// Establish connectivity to a vertex that precedes the first vertex.
	/// Don't call this for loops.
	void SetPrevVertex(const ci::Vec2f& prevVertex);

	/// Establish connectivity to a vertex that follows the last vertex.
	/// Don't call this for loops.
	void SetNextVertex(const ci::Vec2f& nextVertex);

	/// Implement b2Shape. Vertices are cloned using b2Alloc.
	b2Shape* Clone(b2BlockAllocator* allocator) const;

	/// @see b2Shape::GetChildCount
	int GetChildCount() const;

	/// Get a child edge.
	void GetChildEdge(b2EdgeShape* edge, int index) const;

	/// This always return false.
	/// @see b2Shape::TestPoint
	bool TestPoint(const b2Transform& transform, const ci::Vec2f& p) const;

	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
					const b2Transform& transform, int childIndex) const;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int childIndex) const;

	/// Chains have zero mass.
	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float density) const;

	/// The vertices. Owned by this class.
	ci::Vec2f* m_vertices;

	/// The vertex count.
	int m_count;

	ci::Vec2f m_prevVertex, m_nextVertex;
	bool m_hasPrevVertex, m_hasNextVertex;
};

inline b2ChainShape::b2ChainShape()
{
	m_type = e_chain;
	m_radius = b2_polygonRadius;
	m_vertices = NULL;
	m_count = 0;
	m_hasPrevVertex = false;
	m_hasNextVertex = false;
}

#endif
