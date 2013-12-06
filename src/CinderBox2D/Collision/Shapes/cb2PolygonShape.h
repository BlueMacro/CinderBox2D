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

#ifndef B2_POLYGON_SHAPE_H
#define B2_POLYGON_SHAPE_H

#include <CinderBox2D/Collision/Shapes/cb2Shape.h>

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class b2PolygonShape : public b2Shape
{
public:
	b2PolygonShape();

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const;

	/// @see b2Shape::GetChildCount
	int GetChildCount() const;

	/// Copy vertices. This assumes the vertices define a convex polygon.
	/// It is assumed that the exterior is the the right of each edge.
	/// The count must be in the range [3, b2_maxPolygonVertices].
	void set(const ci::Vec2f* vertices, int vertexCount);

	/// Build vertices to represent an axis-aligned box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	void SetAsBox(float hx, float hy);

	/// Build vertices to represent an oriented box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param center the center of the box in local coordinates.
	/// @param angle the rotation of the box in local coordinates.
	void SetAsBox(float hx, float hy, const ci::Vec2f& center, float angle);

	/// @see b2Shape::TestPoint
	bool TestPoint(const b2Transform& transform, const ci::Vec2f& p) const;

	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
					const b2Transform& transform, int childIndex) const;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int childIndex) const;

	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float density) const;

	/// Get the vertex count.
	int GetVertexCount() const { return m_vertexCount; }

	/// Get a vertex by index.
	const ci::Vec2f& GetVertex(int index) const;

	ci::Vec2f m_centroid;
	ci::Vec2f m_vertices[b2_maxPolygonVertices];
	ci::Vec2f m_normals[b2_maxPolygonVertices];
	int m_vertexCount;
};

inline b2PolygonShape::b2PolygonShape()
{
	m_type = e_polygon;
	m_radius = b2_polygonRadius;
	m_vertexCount = 0;
	cb2::setZero(m_centroid);
}

inline const ci::Vec2f& b2PolygonShape::GetVertex(int index) const
{
	b2Assert(0 <= index && index < m_vertexCount);
	return m_vertices[index];
}

#endif
