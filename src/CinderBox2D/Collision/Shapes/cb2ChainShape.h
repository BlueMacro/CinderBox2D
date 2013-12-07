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

#ifndef CB2_CHAIN_SHAPE_H
#define CB2_CHAIN_SHAPE_H

#include <CinderBox2D/Collision/Shapes/cb2Shape.h>

class cb2EdgeShape;

/// A chain shape is a free form sequence of line segments.
/// The chain has two-sided collision, so you can use inside and outside collision.
/// Therefore, you may use any winding order.
/// Since there may be many vertices, they are allocated using cb2Alloc.
/// Connectivity information is used to create smooth collisions.
/// WARNING: The chain will not collide properly if there are self-intersections.
class cb2ChainShape : public cb2Shape
{
public:
	cb2ChainShape();

	/// The destructor frees the vertices using cb2Free.
	~cb2ChainShape();

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

	/// Implement cb2Shape. Vertices are cloned using cb2Alloc.
	cb2Shape* Clone(cb2BlockAllocator* allocator) const;

	/// @see cb2Shape::GetChildCount
	int GetChildCount() const;

	/// Get a child edge.
	void GetChildEdge(cb2EdgeShape* edge, int index) const;

	/// This always return false.
	/// @see cb2Shape::TestPoint
	bool TestPoint(const cb2Transform& transform, const ci::Vec2f& p) const;

	/// Implement cb2Shape.
	bool RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input,
					const cb2Transform& transform, int childIndex) const;

	/// @see cb2Shape::ComputeAABB
	void ComputeAABB(cb2AABB* aabb, const cb2Transform& transform, int childIndex) const;

	/// Chains have zero mass.
	/// @see cb2Shape::ComputeMass
	void ComputeMass(cb2MassData* massData, float density) const;

	/// The vertices. Owned by this class.
	ci::Vec2f* m_vertices;

	/// The vertex count.
	int m_count;

	ci::Vec2f m_prevVertex, m_nextVertex;
	bool m_hasPrevVertex, m_hasNextVertex;
};

inline cb2ChainShape::cb2ChainShape()
{
	m_type = e_chain;
	m_radius = cb2_polygonRadius;
	m_vertices = NULL;
	m_count = 0;
	m_hasPrevVertex = false;
	m_hasNextVertex = false;
}

#endif
