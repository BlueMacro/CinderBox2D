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

#ifndef CB2_EDGE_SHAPE_H
#define CB2_EDGE_SHAPE_H

#include <CinderBox2D/Collision/Shapes/cb2Shape.h>

/// A line segment (edge) shape. These can be connected in chains or loops
/// to other edge shapes. The connectivity information is used to ensure
/// correct contact normals.
class cb2EdgeShape : public cb2Shape
{
public:
	cb2EdgeShape();

	/// Set this as an isolated edge.
	void Set(const ci::Vec2f& v1, const ci::Vec2f& v2);

	/// Implement cb2Shape.
	cb2Shape* Clone(cb2BlockAllocator* allocator) const;

	/// @see cb2Shape::GetChildCount
	int GetChildCount() const;

	/// @see cb2Shape::TestPoint
	bool TestPoint(const cb2Transform& transform, const ci::Vec2f& p) const;

	/// Implement cb2Shape.
	bool RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input,
				const cb2Transform& transform, int childIndex) const;

	/// @see cb2Shape::ComputeAABB
	void ComputeAABB(cb2AABB* aabb, const cb2Transform& transform, int childIndex) const;

	/// @see cb2Shape::ComputeMass
	void ComputeMass(cb2MassData* massData, float density) const;
	
	/// These are the edge vertices
	ci::Vec2f m_vertex1, m_vertex2;

	/// Optional adjacent vertices. These are used for smooth collision.
	ci::Vec2f m_vertex0, m_vertex3;
	bool m_hasVertex0, m_hasVertex3;
};

inline cb2EdgeShape::cb2EdgeShape()
{
	m_type = e_edge;
	m_radius = cb2_polygonRadius;
	m_vertex0.x = 0.0f;
	m_vertex0.y = 0.0f;
	m_vertex3.x = 0.0f;
	m_vertex3.y = 0.0f;
	m_hasVertex0 = false;
	m_hasVertex3 = false;
}

#endif
