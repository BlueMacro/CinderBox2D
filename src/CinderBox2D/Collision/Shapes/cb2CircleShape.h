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

#ifndef CB2_CIRCLE_SHAPE_H
#define CB2_CIRCLE_SHAPE_H

#include <CinderBox2D/Collision/Shapes/cb2Shape.h>

/// A circle shape.
class cb2CircleShape : public cb2Shape
{
public:
	cb2CircleShape();

	/// Implement cb2Shape.
	cb2Shape* Clone(cb2BlockAllocator* allocator) const;

	/// @see cb2Shape::GetChildCount
	int GetChildCount() const;

	/// Implement cb2Shape.
	bool TestPoint(const cb2Transform& transform, const ci::Vec2f& p) const;

	/// Implement cb2Shape.
	bool RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input,
				const cb2Transform& transform, int childIndex) const;

	/// @see cb2Shape::ComputeAABB
	void ComputeAABB(cb2AABB* aabb, const cb2Transform& transform, int childIndex) const;

	/// @see cb2Shape::ComputeMass
	void ComputeMass(cb2MassData* massData, float density) const;

	/// Get the supporting vertex index in the given direction.
	int GetSupport(const ci::Vec2f& d) const;

	/// Get the supporting vertex in the given direction.
	const ci::Vec2f& GetSupportVertex(const ci::Vec2f& d) const;

	/// Get the vertex count.
	int GetVertexCount() const { return 1; }

	/// Get a vertex by index. Used by cb2Distance.
	const ci::Vec2f& GetVertex(int index) const;

	/// Position
	ci::Vec2f m_p;
};

inline cb2CircleShape::cb2CircleShape()
{
	m_type = e_circle;
	m_radius = 0.0f;
	cb2::setZero(m_p);
}

inline int cb2CircleShape::GetSupport(const ci::Vec2f &d) const
{
	CB2_NOT_USED(d);
	return 0;
}

inline const ci::Vec2f& cb2CircleShape::GetSupportVertex(const ci::Vec2f &d) const
{
	CB2_NOT_USED(d);
	return m_p;
}

inline const ci::Vec2f& cb2CircleShape::GetVertex(int index) const
{
	CB2_NOT_USED(index);
	cb2Assert(index == 0);
	return m_p;
}

#endif
