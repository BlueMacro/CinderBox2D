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

#ifndef CB2_COLLISION_H
#define CB2_COLLISION_H

#include <CinderBox2D/Common/cb2Math.h>
#include <climits>

/// @file
/// Structures and functions used for computing contact points, distance
/// queries, and TOI queries.

class cb2Shape;
class cb2CircleShape;
class cb2EdgeShape;
class cb2PolygonShape;

const unsigned char cb2_nullFeature = UCHAR_MAX;

/// The features that intersect to form the contact point
/// This must be 4 bytes or less.
struct cb2ContactFeature
{
	enum Type
	{
		e_vertex = 0,
		e_face = 1
	};

	unsigned char indexA;		///< Feature index on shapeA
	unsigned char indexB;		///< Feature index on shapeB
	unsigned char typeA;		///< The feature type on shapeA
	unsigned char typeB;		///< The feature type on shapeB
};

/// Contact ids to facilitate warm starting.
union cb2ContactID
{
	cb2ContactFeature cf;
	unsigned int key;					///< Used to quickly compare contact ids.
};

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleB
/// -e_faceA: the local center of cirlceB or the clip point of polygonB
/// -e_faceB: the clip point of polygonA
/// This structure is stored across time steps, so we keep it small.
/// Note: the impulses are used for internal caching and may not
/// provide reliable contact forces, especially for high speed collisions.
struct cb2ManifoldPoint
{
	ci::Vec2f localPoint;		///< usage depends on manifold type
	float normalImpulse;	///< the non-penetration impulse
	float tangentImpulse;	///< the friction impulse
	cb2ContactID id;			///< uniquely identifies a contact point between two shapes
};

/// A manifold for two touching convex shapes.
/// Box2D supports multiple types of contact:
/// - clip point versus plane with radius
/// - point versus point with radius (circles)
/// The local point usage depends on the manifold type:
/// -e_circles: the local center of circleA
/// -e_faceA: the center of faceA
/// -e_faceB: the center of faceB
/// Similarly the local normal usage:
/// -e_circles: not used
/// -e_faceA: the normal on polygonA
/// -e_faceB: the normal on polygonB
/// We store contacts in this way so that position correction can
/// account for movement, which is critical for continuous physics.
/// All contact scenarios must be expressed in one of these types.
/// This structure is stored across time steps, so we keep it small.
struct cb2Manifold
{
	enum Type
	{
		e_circles,
		e_faceA,
		e_faceB
	};

	cb2ManifoldPoint points[cb2_maxManifoldPoints];	///< the points of contact
	ci::Vec2f localNormal;								///< not use for Type::e_points
	ci::Vec2f localPoint;								///< usage depends on manifold type
	Type type;
	int pointCount;								///< the number of manifold points
};

/// This is used to compute the current state of a contact manifold.
struct cb2WorldManifold
{
	/// Evaluate the manifold with supplied transforms. This assumes
	/// modest motion from the original state. This does not change the
	/// point count, impulses, etc. The radii must come from the shapes
	/// that generated the manifold.
	void Initialize(const cb2Manifold* manifold,
					const cb2Transform& xfA, float radiusA,
					const cb2Transform& xfB, float radiusB);

	ci::Vec2f normal;							///< world vector pointing from A to B
	ci::Vec2f points[cb2_maxManifoldPoints];	///< world contact point (point of intersection)
	float separations[cb2_maxManifoldPoints];	///< a negative value indicates overlap, in meters
};

/// This is used for determining the state of contact points.
enum cb2PointState
{
	cb2_nullState,		///< point does not exist
	cb2_addState,		///< point was added in the update
	cb2_persistState,	///< point persisted across the update
	cb2_removeState		///< point was removed in the update
};

/// Compute the point states given two manifolds. The states pertain to the transition from manifold1
/// to manifold2. So state1 is either persist or remove while state2 is either add or persist.
void cb2GetPointStates(cb2PointState state1[cb2_maxManifoldPoints], cb2PointState state2[cb2_maxManifoldPoints],
					  const cb2Manifold* manifold1, const cb2Manifold* manifold2);

/// Used for computing contact manifolds.
struct cb2ClipVertex
{
	ci::Vec2f v;
	cb2ContactID id;
};

/// Ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
struct cb2RayCastInput
{
	ci::Vec2f p1, p2;
	float maxFraction;
};

/// Ray-cast output data. The ray hits at p1 + fraction * (p2 - p1), where p1 and p2
/// come from cb2RayCastInput.
struct cb2RayCastOutput
{
	ci::Vec2f normal;
	float fraction;
};

/// An axis aligned bounding box.
struct cb2AABB
{
	/// Verify that the bounds are sorted.
	bool IsValid() const;

	/// Get the center of the AABB.
	ci::Vec2f GetCenter() const
	{
		return 0.5f * (lowerBound + upperBound);
	}

	/// Get the extents of the AABB (half-widths).
	ci::Vec2f GetExtents() const
	{
		return 0.5f * (upperBound - lowerBound);
	}

	/// Get the perimeter length
	float GetPerimeter() const
	{
		float wx = upperBound.x - lowerBound.x;
		float wy = upperBound.y - lowerBound.y;
		return 2.0f * (wx + wy);
	}

	/// Combine an AABB into this one.
	void Combine(const cb2AABB& aabb)
	{
		lowerBound = cb2Min(lowerBound, aabb.lowerBound);
		upperBound = cb2Max(upperBound, aabb.upperBound);
	}

	/// Combine two AABBs into this one.
	void Combine(const cb2AABB& aabb1, const cb2AABB& aabb2)
	{
		lowerBound = cb2Min(aabb1.lowerBound, aabb2.lowerBound);
		upperBound = cb2Max(aabb1.upperBound, aabb2.upperBound);
	}

	/// Does this aabb contain the provided AABB.
	bool Contains(const cb2AABB& aabb) const
	{
		bool result = true;
		result = result && lowerBound.x <= aabb.lowerBound.x;
		result = result && lowerBound.y <= aabb.lowerBound.y;
		result = result && aabb.upperBound.x <= upperBound.x;
		result = result && aabb.upperBound.y <= upperBound.y;
		return result;
	}

	bool RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input) const;

	ci::Vec2f lowerBound;	///< the lower vertex
	ci::Vec2f upperBound;	///< the upper vertex
};

/// Compute the collision manifold between two circles.
void cb2CollideCircles(cb2Manifold* manifold,
					  const cb2CircleShape* circleA, const cb2Transform& xfA,
					  const cb2CircleShape* circleB, const cb2Transform& xfB);

/// Compute the collision manifold between a polygon and a circle.
void cb2CollidePolygonAndCircle(cb2Manifold* manifold,
							   const cb2PolygonShape* polygonA, const cb2Transform& xfA,
							   const cb2CircleShape* circleB, const cb2Transform& xfB);

/// Compute the collision manifold between two polygons.
void cb2CollidePolygons(cb2Manifold* manifold,
					   const cb2PolygonShape* polygonA, const cb2Transform& xfA,
					   const cb2PolygonShape* polygonB, const cb2Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
void cb2CollideEdgeAndCircle(cb2Manifold* manifold,
							   const cb2EdgeShape* polygonA, const cb2Transform& xfA,
							   const cb2CircleShape* circleB, const cb2Transform& xfB);

/// Compute the collision manifold between an edge and a circle.
void cb2CollideEdgeAndPolygon(cb2Manifold* manifold,
							   const cb2EdgeShape* edgeA, const cb2Transform& xfA,
							   const cb2PolygonShape* circleB, const cb2Transform& xfB);

/// Clipping for contact manifolds.
int cb2ClipSegmentToLine(cb2ClipVertex vOut[2], const cb2ClipVertex vIn[2],
							const ci::Vec2f& normal, float offset, int vertexIndexA);

/// Determine if two generic shapes overlap.
bool cb2TestOverlap(	const cb2Shape* shapeA, int indexA,
					const cb2Shape* shapeB, int indexB,
					const cb2Transform& xfA, const cb2Transform& xfB);

// ---------------- Inline Functions ------------------------------------------

inline bool cb2AABB::IsValid() const
{
	ci::Vec2f d = upperBound - lowerBound;
	bool valid = d.x >= 0.0f && d.y >= 0.0f;
	valid = valid && cb2::isValid(lowerBound) && cb2::isValid(upperBound);
	return valid;
}

inline bool cb2TestOverlap(const cb2AABB& a, const cb2AABB& b)
{
	ci::Vec2f d1, d2;
	d1 = b.lowerBound - a.upperBound;
	d2 = a.lowerBound - b.upperBound;

	if (d1.x > 0.0f || d1.y > 0.0f)
		return false;

	if (d2.x > 0.0f || d2.y > 0.0f)
		return false;

	return true;
}

#endif
