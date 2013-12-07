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

#ifndef CB2_POLYGON_SHAPE_H
#define CB2_POLYGON_SHAPE_H

#include <CinderBox2D/Collision/Shapes/cb2Shape.h>

/// A convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to cb2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
class cb2PolygonShape : public cb2Shape
{
public:
	cb2PolygonShape();

	/// Implement cb2Shape.
	cb2Shape* Clone(cb2BlockAllocator* allocator) const;

	/// @see cb2Shape::GetChildCount
	int GetChildCount() const;

	/// Create a convex hull from the given array of local points.
	/// The count must be in the range [3, cb2_maxPolygonVertices].
	/// @warning the points may be re-ordered, even if they form a convex polygon
	/// @warning collinear points are handled but not removed. Collinear points
	/// may lead to poor stacking behavior.
	void set(const ci::Vec2f* points, int count);

	/// Build vertices to represent an axis-aligned box centered on the local origin.
	/// @param hx the half-width.
	/// @param hy the half-height.
	void SetAsBox(float hx, float hy);

	/// Build vertices to represent an oriented box.
	/// @param hx the half-width.
	/// @param hy the half-height.
	/// @param center the center of the box in local coordinates.
	/// @param angle the rotation of the box in local coordinates.
	void SetAsBox(float hx, float hy, const ci::Vec2f& center, float angle);

	/// @see cb2Shape::TestPoint
	bool TestPoint(const cb2Transform& transform, const ci::Vec2f& p) const;

	/// Implement cb2Shape.
	bool RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input,
					const cb2Transform& transform, int childIndex) const;

	/// @see cb2Shape::ComputeAABB
	void ComputeAABB(cb2AABB* aabb, const cb2Transform& transform, int childIndex) const;

	/// @see cb2Shape::ComputeMass
	void ComputeMass(cb2MassData* massData, float density) const;

	/// Get the vertex count.
	int GetVertexCount() const { return m_count; }

	/// Get a vertex by index.
	const ci::Vec2f& GetVertex(int index) const;

	/// Validate convexity. This is a very time consuming operation.
	/// @returns true if valid
	bool Validate() const;

	ci::Vec2f m_centroid;
	ci::Vec2f m_vertices[cb2_maxPolygonVertices];
	ci::Vec2f m_normals[cb2_maxPolygonVertices];
	int m_count;
};

inline cb2PolygonShape::cb2PolygonShape()
{
	m_type = e_polygon;
	m_radius = cb2_polygonRadius;
	m_count = 0;
}

inline const ci::Vec2f& cb2PolygonShape::GetVertex(int index) const
{
	cb2Assert(0 <= index && index < m_count);
	return m_vertices[index];
}

#endif
