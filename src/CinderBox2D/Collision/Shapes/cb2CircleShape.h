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

#ifndef B2_CIRCLE_SHAPE_H
#define B2_CIRCLE_SHAPE_H

#include <CinderBox2D/Collision/Shapes/cb2Shape.h>

/// A circle shape.
class b2CircleShape : public b2Shape
{
public:
	b2CircleShape();

	/// Implement b2Shape.
	b2Shape* Clone(b2BlockAllocator* allocator) const;

	/// @see b2Shape::GetChildCount
	int GetChildCount() const;

	/// Implement b2Shape.
	bool TestPoint(const b2Transform& transform, const ci::Vec2f& p) const;

	/// Implement b2Shape.
	bool RayCast(b2RayCastOutput* output, const b2RayCastInput& input,
				const b2Transform& transform, int childIndex) const;

	/// @see b2Shape::ComputeAABB
	void ComputeAABB(b2AABB* aabb, const b2Transform& transform, int childIndex) const;

	/// @see b2Shape::ComputeMass
	void ComputeMass(b2MassData* massData, float density) const;

	/// Get the supporting vertex index in the given direction.
	int GetSupport(const ci::Vec2f& d) const;

	/// Get the supporting vertex in the given direction.
	const ci::Vec2f& GetSupportVertex(const ci::Vec2f& d) const;

	/// Get the vertex count.
	int GetVertexCount() const { return 1; }

	/// Get a vertex by index. Used by b2Distance.
	const ci::Vec2f& GetVertex(int index) const;

	/// Position
	ci::Vec2f m_p;
};

inline b2CircleShape::b2CircleShape()
{
	m_type = e_circle;
	m_radius = 0.0f;
	cb2::setZero(m_p);
}

inline int b2CircleShape::GetSupport(const ci::Vec2f &d) const
{
	B2_NOT_USED(d);
	return 0;
}

inline const ci::Vec2f& b2CircleShape::GetSupportVertex(const ci::Vec2f &d) const
{
	B2_NOT_USED(d);
	return m_p;
}

inline const ci::Vec2f& b2CircleShape::GetVertex(int index) const
{
	B2_NOT_USED(index);
	b2Assert(index == 0);
	return m_p;
}

#endif
