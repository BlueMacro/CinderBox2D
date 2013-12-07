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
#include <CinderBox2D/Collision/Shapes/cb2PolygonShape.h>

void cb2CollideCircles(
	cb2Manifold* manifold,
	const cb2CircleShape* circleA, const cb2Transform& xfA,
	const cb2CircleShape* circleB, const cb2Transform& xfB)
{
	manifold->pointCount = 0;

	ci::Vec2f pA = cb2Mul(xfA, circleA->m_p);
	ci::Vec2f pB = cb2Mul(xfB, circleB->m_p);

	ci::Vec2f d = pB - pA;
	float distSqr = cb2Dot(d, d);
	float rA = circleA->m_radius, rB = circleB->m_radius;
	float radius = rA + rB;
	if (distSqr > radius * radius)
	{
		return;
	}

	manifold->type = cb2Manifold::e_circles;
	manifold->localPoint = circleA->m_p;
	cb2::setZero(manifold->localNormal);
	manifold->pointCount = 1;

	manifold->points[0].localPoint = circleB->m_p;
	manifold->points[0].id.key = 0;
}

void cb2CollidePolygonAndCircle(
	cb2Manifold* manifold,
	const cb2PolygonShape* polygonA, const cb2Transform& xfA,
	const cb2CircleShape* circleB, const cb2Transform& xfB)
{
	manifold->pointCount = 0;

	// Compute circle position in the frame of the polygon.
	ci::Vec2f c = cb2Mul(xfB, circleB->m_p);
	ci::Vec2f cLocal = cb2MulT(xfA, c);

	// Find the min separating edge.
	int normalIndex = 0;
	float separation = -cb2_maxFloat;
	float radius = polygonA->m_radius + circleB->m_radius;
	int vertexCount = polygonA->m_count;
	const ci::Vec2f* vertices = polygonA->m_vertices;
	const ci::Vec2f* normals = polygonA->m_normals;

	for (int i = 0; i < vertexCount; ++i)
	{
		float s = cb2Dot(normals[i], cLocal - vertices[i]);

		if (s > radius)
		{
			// Early out.
			return;
		}

		if (s > separation)
		{
			separation = s;
			normalIndex = i;
		}
	}

	// Vertices that subtend the incident face.
	int vertIndex1 = normalIndex;
	int vertIndex2 = vertIndex1 + 1 < vertexCount ? vertIndex1 + 1 : 0;
	ci::Vec2f v1 = vertices[vertIndex1];
	ci::Vec2f v2 = vertices[vertIndex2];

	// If the center is inside the polygon ...
	if (separation < cb2_epsilon)
	{
		manifold->pointCount = 1;
		manifold->type = cb2Manifold::e_faceA;
		manifold->localNormal = normals[normalIndex];
		manifold->localPoint = 0.5f * (v1 + v2);
		manifold->points[0].localPoint = circleB->m_p;
		manifold->points[0].id.key = 0;
		return;
	}

	// Compute barycentric coordinates
	float u1 = cb2Dot(cLocal - v1, v2 - v1);
	float u2 = cb2Dot(cLocal - v2, v1 - v2);
	if (u1 <= 0.0f)
	{
		if (cb2DistanceSquared(cLocal, v1) > radius * radius)
		{
			return;
		}

		manifold->pointCount = 1;
		manifold->type = cb2Manifold::e_faceA;
		manifold->localNormal = cLocal - v1;
		manifold->localNormal.normalize();
		manifold->localPoint = v1;
		manifold->points[0].localPoint = circleB->m_p;
		manifold->points[0].id.key = 0;
	}
	else if (u2 <= 0.0f)
	{
		if (cb2DistanceSquared(cLocal, v2) > radius * radius)
		{
			return;
		}

		manifold->pointCount = 1;
		manifold->type = cb2Manifold::e_faceA;
		manifold->localNormal = cLocal - v2;
		manifold->localNormal.normalize();
		manifold->localPoint = v2;
		manifold->points[0].localPoint = circleB->m_p;
		manifold->points[0].id.key = 0;
	}
	else
	{
		ci::Vec2f faceCenter = 0.5f * (v1 + v2);
		float separation = cb2Dot(cLocal - faceCenter, normals[vertIndex1]);
		if (separation > radius)
		{
			return;
		}

		manifold->pointCount = 1;
		manifold->type = cb2Manifold::e_faceA;
		manifold->localNormal = normals[vertIndex1];
		manifold->localPoint = faceCenter;
		manifold->points[0].localPoint = circleB->m_p;
		manifold->points[0].id.key = 0;
	}
}
