/*
 * Copyright (c) 2007-2009 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Collision/cb2Collision.h>
#include <CinderBox2D/Collision/Shapes/cb2CircleShape.h>
#include <CinderBox2D/Collision/Shapes/cb2EdgeShape.h>
#include <CinderBox2D/Collision/Shapes/cb2PolygonShape.h>


// Compute contact points for edge versus circle.
// This accounts for edge connectivity.
void cb2CollideEdgeAndCircle(cb2Manifold* manifold,
							const cb2EdgeShape* edgeA, const cb2Transform& xfA,
							const cb2CircleShape* circleB, const cb2Transform& xfB)
{
	manifold->pointCount = 0;
	
	// Compute circle in frame of edge
	ci::Vec2f Q = cb2MulT(xfA, cb2Mul(xfB, circleB->m_p));
	
	ci::Vec2f A = edgeA->m_vertex1, B = edgeA->m_vertex2;
	ci::Vec2f e = B - A;
	
	// Barycentric coordinates
	float u = cb2Dot(e, B - Q);
	float v = cb2Dot(e, Q - A);
	
	float radius = edgeA->m_radius + circleB->m_radius;
	
	cb2ContactFeature cf;
	cf.indexB = 0;
	cf.typeB = cb2ContactFeature::e_vertex;
	
	// Region A
	if (v <= 0.0f)
	{
		ci::Vec2f P = A;
		ci::Vec2f d = Q - P;
		float dd = cb2Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}
		
		// Is there an edge connected to A?
		if (edgeA->m_hasVertex0)
		{
			ci::Vec2f A1 = edgeA->m_vertex0;
			ci::Vec2f B1 = A;
			ci::Vec2f e1 = B1 - A1;
			float u1 = cb2Dot(e1, B1 - Q);
			
			// Is the circle in Region AB of the previous edge?
			if (u1 > 0.0f)
			{
				return;
			}
		}
		
		cf.indexA = 0;
		cf.typeA = cb2ContactFeature::e_vertex;
		manifold->pointCount = 1;
		manifold->type = cb2Manifold::e_circles;
		cb2::setZero(manifold->localNormal);
		manifold->localPoint = P;
		manifold->points[0].id.key = 0;
		manifold->points[0].id.cf = cf;
		manifold->points[0].localPoint = circleB->m_p;
		return;
	}
	
	// Region B
	if (u <= 0.0f)
	{
		ci::Vec2f P = B;
		ci::Vec2f d = Q - P;
		float dd = cb2Dot(d, d);
		if (dd > radius * radius)
		{
			return;
		}
		
		// Is there an edge connected to B?
		if (edgeA->m_hasVertex3)
		{
			ci::Vec2f CB2 = edgeA->m_vertex3;
			ci::Vec2f A2 = B;
			ci::Vec2f e2 = CB2 - A2;
			float v2 = cb2Dot(e2, Q - A2);
			
			// Is the circle in Region AB of the next edge?
			if (v2 > 0.0f)
			{
				return;
			}
		}
		
		cf.indexA = 1;
		cf.typeA = cb2ContactFeature::e_vertex;
		manifold->pointCount = 1;
		manifold->type = cb2Manifold::e_circles;
		cb2::setZero(manifold->localNormal);
		manifold->localPoint = P;
		manifold->points[0].id.key = 0;
		manifold->points[0].id.cf = cf;
		manifold->points[0].localPoint = circleB->m_p;
		return;
	}
	
	// Region AB
	float den = cb2Dot(e, e);
	cb2Assert(den > 0.0f);
	ci::Vec2f P = (1.0f / den) * (u * A + v * B);
	ci::Vec2f d = Q - P;
	float dd = cb2Dot(d, d);
	if (dd > radius * radius)
	{
		return;
	}
	
	ci::Vec2f n(-e.y, e.x);
	if (cb2Dot(n, Q - A) < 0.0f)
	{
		n.set(-n.x, -n.y);
	}
	n.normalize();
	
	cf.indexA = 0;
	cf.typeA = cb2ContactFeature::e_face;
	manifold->pointCount = 1;
	manifold->type = cb2Manifold::e_faceA;
	manifold->localNormal = n;
	manifold->localPoint = A;
	manifold->points[0].id.key = 0;
	manifold->points[0].id.cf = cf;
	manifold->points[0].localPoint = circleB->m_p;
}

// This structure is used to keep track of the best separating axis.
struct cb2EPAxis
{
	enum Type
	{
		e_unknown,
		e_edgeA,
		e_edgeB
	};
	
	Type type;
	int index;
	float separation;
};

// This holds polygon B expressed in frame A.
struct cb2TempPolygon
{
	ci::Vec2f vertices[cb2_maxPolygonVertices];
	ci::Vec2f normals[cb2_maxPolygonVertices];
	int count;
};

// Reference face used for clipping
struct cb2ReferenceFace
{
	int i1, i2;
	
	ci::Vec2f v1, v2;
	
	ci::Vec2f normal;
	
	ci::Vec2f sideNormal1;
	float sideOffset1;
	
	ci::Vec2f sideNormal2;
	float sideOffset2;
};

// This class collides and edge and a polygon, taking into account edge adjacency.
struct cb2EPCollider
{
	void Collide(cb2Manifold* manifold, const cb2EdgeShape* edgeA, const cb2Transform& xfA,
				 const cb2PolygonShape* polygonB, const cb2Transform& xfB);
	cb2EPAxis ComputeEdgeSeparation();
	cb2EPAxis ComputePolygonSeparation();
	
	enum VertexType
	{
		e_isolated,
		e_concave,
		e_convex
	};
	
	cb2TempPolygon m_polygonB;
	
	cb2Transform m_xf;
	ci::Vec2f m_centroidB;
	ci::Vec2f m_v0, m_v1, m_v2, m_v3;
	ci::Vec2f m_normal0, m_normal1, m_normal2;
	ci::Vec2f m_normal;
	VertexType m_type1, m_type2;
	ci::Vec2f m_lowerLimit, m_upperLimit;
	float m_radius;
	bool m_front;
};

// Algorithm:
// 1. Classify v1 and v2
// 2. Classify polygon centroid as front or back
// 3. Flip normal if necessary
// 4. Initialize normal range to [-pi, pi] about face normal
// 5. Adjust normal range according to adjacent edges
// 6. Visit each separating axes, only accept axes within the range
// 7. Return if _any_ axis indicates separation
// 8. Clip
void cb2EPCollider::Collide(cb2Manifold* manifold, const cb2EdgeShape* edgeA, const cb2Transform& xfA,
						   const cb2PolygonShape* polygonB, const cb2Transform& xfB)
{
	m_xf = cb2MulT(xfA, xfB);
	
	m_centroidB = cb2Mul(m_xf, polygonB->m_centroid);
	
	m_v0 = edgeA->m_vertex0;
	m_v1 = edgeA->m_vertex1;
	m_v2 = edgeA->m_vertex2;
	m_v3 = edgeA->m_vertex3;
	
	bool hasVertex0 = edgeA->m_hasVertex0;
	bool hasVertex3 = edgeA->m_hasVertex3;
	
	ci::Vec2f edge1 = m_v2 - m_v1;
	edge1.normalize();
	m_normal1.set(edge1.y, -edge1.x);
	float offset1 = cb2Dot(m_normal1, m_centroidB - m_v1);
	float offset0 = 0.0f, offset2 = 0.0f;
	bool convex1 = false, convex2 = false;
	
	// Is there a preceding edge?
	if (hasVertex0)
	{
		ci::Vec2f edge0 = m_v1 - m_v0;
		edge0.normalize();
		m_normal0.set(edge0.y, -edge0.x);
		convex1 = cb2Cross(edge0, edge1) >= 0.0f;
		offset0 = cb2Dot(m_normal0, m_centroidB - m_v0);
	}
	
	// Is there a following edge?
	if (hasVertex3)
	{
		ci::Vec2f edge2 = m_v3 - m_v2;
		edge2.normalize();
		m_normal2.set(edge2.y, -edge2.x);
		convex2 = cb2Cross(edge1, edge2) > 0.0f;
		offset2 = cb2Dot(m_normal2, m_centroidB - m_v2);
	}
	
	// Determine front or back collision. Determine collision normal limits.
	if (hasVertex0 && hasVertex3)
	{
		if (convex1 && convex2)
		{
			m_front = offset0 >= 0.0f || offset1 >= 0.0f || offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else if (convex1)
		{
			m_front = offset0 >= 0.0f || (offset1 >= 0.0f && offset2 >= 0.0f);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = -m_normal1;
			}
		}
		else if (convex2)
		{
			m_front = offset2 >= 0.0f || (offset0 >= 0.0f && offset1 >= 0.0f);
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = -m_normal0;
			}
		}
		else
		{
			m_front = offset0 >= 0.0f && offset1 >= 0.0f && offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = -m_normal0;
			}
		}
	}
	else if (hasVertex0)
	{
		if (convex1)
		{
			m_front = offset0 >= 0.0f || offset1 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal0;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
		}
		else
		{
			m_front = offset0 >= 0.0f && offset1 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = m_normal1;
				m_upperLimit = -m_normal0;
			}
		}
	}
	else if (hasVertex3)
	{
		if (convex2)
		{
			m_front = offset1 >= 0.0f || offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal2;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
		}
		else
		{
			m_front = offset1 >= 0.0f && offset2 >= 0.0f;
			if (m_front)
			{
				m_normal = m_normal1;
				m_lowerLimit = -m_normal1;
				m_upperLimit = m_normal1;
			}
			else
			{
				m_normal = -m_normal1;
				m_lowerLimit = -m_normal2;
				m_upperLimit = m_normal1;
			}
		}		
	}
	else
	{
		m_front = offset1 >= 0.0f;
		if (m_front)
		{
			m_normal = m_normal1;
			m_lowerLimit = -m_normal1;
			m_upperLimit = -m_normal1;
		}
		else
		{
			m_normal = -m_normal1;
			m_lowerLimit = m_normal1;
			m_upperLimit = m_normal1;
		}
	}
	
	// Get polygonB in frameA
	m_polygonB.count = polygonB->m_count;
	for (int i = 0; i < polygonB->m_count; ++i)
	{
		m_polygonB.vertices[i] = cb2Mul(m_xf, polygonB->m_vertices[i]);
		m_polygonB.normals[i] = cb2Mul(m_xf.q, polygonB->m_normals[i]);
	}
	
	m_radius = 2.0f * cb2_polygonRadius;
	
	manifold->pointCount = 0;
	
	cb2EPAxis edgeAxis = ComputeEdgeSeparation();
	
	// If no valid normal can be found than this edge should not collide.
	if (edgeAxis.type == cb2EPAxis::e_unknown)
	{
		return;
	}
	
	if (edgeAxis.separation > m_radius)
	{
		return;
	}
	
	cb2EPAxis polygonAxis = ComputePolygonSeparation();
	if (polygonAxis.type != cb2EPAxis::e_unknown && polygonAxis.separation > m_radius)
	{
		return;
	}
	
	// Use hysteresis for jitter reduction.
	const float k_relativeTol = 0.98f;
	const float k_absoluteTol = 0.001f;
	
	cb2EPAxis primaryAxis;
	if (polygonAxis.type == cb2EPAxis::e_unknown)
	{
		primaryAxis = edgeAxis;
	}
	else if (polygonAxis.separation > k_relativeTol * edgeAxis.separation + k_absoluteTol)
	{
		primaryAxis = polygonAxis;
	}
	else
	{
		primaryAxis = edgeAxis;
	}
	
	cb2ClipVertex ie[2];
	cb2ReferenceFace rf;
	if (primaryAxis.type == cb2EPAxis::e_edgeA)
	{
		manifold->type = cb2Manifold::e_faceA;
		
		// Search for the polygon normal that is most anti-parallel to the edge normal.
		int bestIndex = 0;
		float bestValue = cb2Dot(m_normal, m_polygonB.normals[0]);
		for (int i = 1; i < m_polygonB.count; ++i)
		{
			float value = cb2Dot(m_normal, m_polygonB.normals[i]);
			if (value < bestValue)
			{
				bestValue = value;
				bestIndex = i;
			}
		}
		
		int i1 = bestIndex;
		int i2 = i1 + 1 < m_polygonB.count ? i1 + 1 : 0;
		
		ie[0].v = m_polygonB.vertices[i1];
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = static_cast<unsigned char>(i1);
		ie[0].id.cf.typeA = cb2ContactFeature::e_face;
		ie[0].id.cf.typeB = cb2ContactFeature::e_vertex;
		
		ie[1].v = m_polygonB.vertices[i2];
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = static_cast<unsigned char>(i2);
		ie[1].id.cf.typeA = cb2ContactFeature::e_face;
		ie[1].id.cf.typeB = cb2ContactFeature::e_vertex;
		
		if (m_front)
		{
			rf.i1 = 0;
			rf.i2 = 1;
			rf.v1 = m_v1;
			rf.v2 = m_v2;
			rf.normal = m_normal1;
		}
		else
		{
			rf.i1 = 1;
			rf.i2 = 0;
			rf.v1 = m_v2;
			rf.v2 = m_v1;
			rf.normal = -m_normal1;
		}		
	}
	else
	{
		manifold->type = cb2Manifold::e_faceB;
		
		ie[0].v = m_v1;
		ie[0].id.cf.indexA = 0;
		ie[0].id.cf.indexB = static_cast<unsigned char>(primaryAxis.index);
		ie[0].id.cf.typeA = cb2ContactFeature::e_vertex;
		ie[0].id.cf.typeB = cb2ContactFeature::e_face;
		
		ie[1].v = m_v2;
		ie[1].id.cf.indexA = 0;
		ie[1].id.cf.indexB = static_cast<unsigned char>(primaryAxis.index);		
		ie[1].id.cf.typeA = cb2ContactFeature::e_vertex;
		ie[1].id.cf.typeB = cb2ContactFeature::e_face;
		
		rf.i1 = primaryAxis.index;
		rf.i2 = rf.i1 + 1 < m_polygonB.count ? rf.i1 + 1 : 0;
		rf.v1 = m_polygonB.vertices[rf.i1];
		rf.v2 = m_polygonB.vertices[rf.i2];
		rf.normal = m_polygonB.normals[rf.i1];
	}
	
	rf.sideNormal1.set(rf.normal.y, -rf.normal.x);
	rf.sideNormal2 = -rf.sideNormal1;
	rf.sideOffset1 = cb2Dot(rf.sideNormal1, rf.v1);
	rf.sideOffset2 = cb2Dot(rf.sideNormal2, rf.v2);
	
	// Clip incident edge against extruded edge1 side edges.
	cb2ClipVertex clipPoints1[2];
	cb2ClipVertex clipPoints2[2];
	int np;
	
	// Clip to box side 1
	np = cb2ClipSegmentToLine(clipPoints1, ie, rf.sideNormal1, rf.sideOffset1, rf.i1);
	
	if (np < cb2_maxManifoldPoints)
	{
		return;
	}
	
	// Clip to negative box side 1
	np = cb2ClipSegmentToLine(clipPoints2, clipPoints1, rf.sideNormal2, rf.sideOffset2, rf.i2);
	
	if (np < cb2_maxManifoldPoints)
	{
		return;
	}
	
	// Now clipPoints2 contains the clipped points.
	if (primaryAxis.type == cb2EPAxis::e_edgeA)
	{
		manifold->localNormal = rf.normal;
		manifold->localPoint = rf.v1;
	}
	else
	{
		manifold->localNormal = polygonB->m_normals[rf.i1];
		manifold->localPoint = polygonB->m_vertices[rf.i1];
	}
	
	int pointCount = 0;
	for (int i = 0; i < cb2_maxManifoldPoints; ++i)
	{
		float separation;
		
		separation = cb2Dot(rf.normal, clipPoints2[i].v - rf.v1);
		
		if (separation <= m_radius)
		{
			cb2ManifoldPoint* cp = manifold->points + pointCount;
			
			if (primaryAxis.type == cb2EPAxis::e_edgeA)
			{
				cp->localPoint = cb2MulT(m_xf, clipPoints2[i].v);
				cp->id = clipPoints2[i].id;
			}
			else
			{
				cp->localPoint = clipPoints2[i].v;
				cp->id.cf.typeA = clipPoints2[i].id.cf.typeB;
				cp->id.cf.typeB = clipPoints2[i].id.cf.typeA;
				cp->id.cf.indexA = clipPoints2[i].id.cf.indexB;
				cp->id.cf.indexB = clipPoints2[i].id.cf.indexA;
			}
			
			++pointCount;
		}
	}
	
	manifold->pointCount = pointCount;
}

cb2EPAxis cb2EPCollider::ComputeEdgeSeparation()
{
	cb2EPAxis axis;
	axis.type = cb2EPAxis::e_edgeA;
	axis.index = m_front ? 0 : 1;
	axis.separation = FLT_MAX;
	
	for (int i = 0; i < m_polygonB.count; ++i)
	{
		float s = cb2Dot(m_normal, m_polygonB.vertices[i] - m_v1);
		if (s < axis.separation)
		{
			axis.separation = s;
		}
	}
	
	return axis;
}

cb2EPAxis cb2EPCollider::ComputePolygonSeparation()
{
	cb2EPAxis axis;
	axis.type = cb2EPAxis::e_unknown;
	axis.index = -1;
	axis.separation = -FLT_MAX;

	ci::Vec2f perp(-m_normal.y, m_normal.x);

	for (int i = 0; i < m_polygonB.count; ++i)
	{
		ci::Vec2f n = -m_polygonB.normals[i];
		
		float s1 = cb2Dot(n, m_polygonB.vertices[i] - m_v1);
		float s2 = cb2Dot(n, m_polygonB.vertices[i] - m_v2);
		float s = cb2Min(s1, s2);
		
		if (s > m_radius)
		{
			// No collision
			axis.type = cb2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
			return axis;
		}
		
		// Adjacency
		if (cb2Dot(n, perp) >= 0.0f)
		{
			if (cb2Dot(n - m_upperLimit, m_normal) < -cb2_angularSlop)
			{
				continue;
			}
		}
		else
		{
			if (cb2Dot(n - m_lowerLimit, m_normal) < -cb2_angularSlop)
			{
				continue;
			}
		}
		
		if (s > axis.separation)
		{
			axis.type = cb2EPAxis::e_edgeB;
			axis.index = i;
			axis.separation = s;
		}
	}
	
	return axis;
}

void cb2CollideEdgeAndPolygon(	cb2Manifold* manifold,
							 const cb2EdgeShape* edgeA, const cb2Transform& xfA,
							 const cb2PolygonShape* polygonB, const cb2Transform& xfB)
{
	cb2EPCollider collider;
	collider.Collide(manifold, edgeA, xfA, polygonB, xfB);
}
