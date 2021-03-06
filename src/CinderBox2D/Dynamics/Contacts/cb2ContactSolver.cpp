/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Dynamics/Contacts/cb2ContactSolver.h>

#include <CinderBox2D/Dynamics/Contacts/cb2Contact.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2Fixture.h>
#include <CinderBox2D/Dynamics/cb2World.h>
#include <CinderBox2D/Common/cb2StackAllocator.h>

#define CB2_DEBUG_SOLVER 0

struct cb2ContactPositionConstraint
{
	ci::Vec2f localPoints[cb2_maxManifoldPoints];
	ci::Vec2f localNormal;
	ci::Vec2f localPoint;
	int indexA;
	int indexB;
	float invMassA, invMassB;
	ci::Vec2f localCenterA, localCenterB;
	float invIA, invIB;
	cb2Manifold::Type type;
	float radiusA, radiusB;
	int pointCount;
};

cb2ContactSolver::cb2ContactSolver(cb2ContactSolverDef* def)
{
	m_step = def->step;
	m_allocator = def->allocator;
	m_count = def->count;
	m_positionConstraints = (cb2ContactPositionConstraint*)m_allocator->Allocate(m_count * sizeof(cb2ContactPositionConstraint));
	m_velocityConstraints = (cb2ContactVelocityConstraint*)m_allocator->Allocate(m_count * sizeof(cb2ContactVelocityConstraint));
	m_positions = def->positions;
	m_velocities = def->velocities;
	m_contacts = def->contacts;

	// Initialize position independent portions of the constraints.
	for (int i = 0; i < m_count; ++i)
	{
		cb2Contact* contact = m_contacts[i];

		cb2Fixture* fixtureA = contact->m_fixtureA;
		cb2Fixture* fixtureB = contact->m_fixtureB;
		cb2Shape* shapeA = fixtureA->GetShape();
		cb2Shape* shapeB = fixtureB->GetShape();
		float radiusA = shapeA->m_radius;
		float radiusB = shapeB->m_radius;
		cb2Body* bodyA = fixtureA->GetBody();
		cb2Body* bodyB = fixtureB->GetBody();
		cb2Manifold* manifold = contact->GetManifold();

		int pointCount = manifold->pointCount;
		cb2Assert(pointCount > 0);

		cb2ContactVelocityConstraint* vc = m_velocityConstraints + i;
		vc->friction = contact->m_friction;
		vc->restitution = contact->m_restitution;
		vc->tangentSpeed = contact->m_tangentSpeed;
		vc->indexA = bodyA->m_islandIndex;
		vc->indexB = bodyB->m_islandIndex;
		vc->invMassA = bodyA->m_invMass;
		vc->invMassB = bodyB->m_invMass;
		vc->invIA = bodyA->m_invI;
		vc->invIB = bodyB->m_invI;
		vc->contactIndex = i;
		vc->pointCount = pointCount;
		cb2::setZero(vc->K);
		cb2::setZero(vc->normalMass);

		cb2ContactPositionConstraint* pc = m_positionConstraints + i;
		pc->indexA = bodyA->m_islandIndex;
		pc->indexB = bodyB->m_islandIndex;
		pc->invMassA = bodyA->m_invMass;
		pc->invMassB = bodyB->m_invMass;
		pc->localCenterA = bodyA->m_sweep.localCenter;
		pc->localCenterB = bodyB->m_sweep.localCenter;
		pc->invIA = bodyA->m_invI;
		pc->invIB = bodyB->m_invI;
		pc->localNormal = manifold->localNormal;
		pc->localPoint = manifold->localPoint;
		pc->pointCount = pointCount;
		pc->radiusA = radiusA;
		pc->radiusB = radiusB;
		pc->type = manifold->type;

		for (int j = 0; j < pointCount; ++j)
		{
			cb2ManifoldPoint* cp = manifold->points + j;
			cb2VelocityConstraintPoint* vcp = vc->points + j;
	
			if (m_step.warmStarting)
			{
				vcp->normalImpulse = m_step.dtRatio * cp->normalImpulse;
				vcp->tangentImpulse = m_step.dtRatio * cp->tangentImpulse;
			}
			else
			{
				vcp->normalImpulse = 0.0f;
				vcp->tangentImpulse = 0.0f;
			}

			cb2::setZero(vcp->rA);
			cb2::setZero(vcp->rB);
			vcp->normalMass = 0.0f;
			vcp->tangentMass = 0.0f;
			vcp->velocityBias = 0.0f;

			pc->localPoints[j] = cp->localPoint;
		}
	}
}

cb2ContactSolver::~cb2ContactSolver()
{
	m_allocator->Free(m_velocityConstraints);
	m_allocator->Free(m_positionConstraints);
}

// Initialize position dependent portions of the velocity constraints.
void cb2ContactSolver::InitializeVelocityConstraints()
{
	for (int i = 0; i < m_count; ++i)
	{
		cb2ContactVelocityConstraint* vc = m_velocityConstraints + i;
		cb2ContactPositionConstraint* pc = m_positionConstraints + i;

		float radiusA = pc->radiusA;
		float radiusB = pc->radiusB;
		cb2Manifold* manifold = m_contacts[vc->contactIndex]->GetManifold();

		int indexA = vc->indexA;
		int indexB = vc->indexB;

		float mA = vc->invMassA;
		float mB = vc->invMassB;
		float iA = vc->invIA;
		float iB = vc->invIB;
		ci::Vec2f localCenterA = pc->localCenterA;
		ci::Vec2f localCenterB = pc->localCenterB;

		ci::Vec2f cA = m_positions[indexA].c;
		float aA = m_positions[indexA].a;
		ci::Vec2f vA = m_velocities[indexA].v;
		float wA = m_velocities[indexA].w;

		ci::Vec2f cB = m_positions[indexB].c;
		float aB = m_positions[indexB].a;
		ci::Vec2f vB = m_velocities[indexB].v;
		float wB = m_velocities[indexB].w;

		cb2Assert(manifold->pointCount > 0);

		cb2Transform xfA, xfB;
		xfA.q.set(aA);
		xfB.q.set(aB);
		xfA.p = cA - cb2Mul(xfA.q, localCenterA);
		xfB.p = cB - cb2Mul(xfB.q, localCenterB);

		cb2WorldManifold worldManifold;
		worldManifold.Initialize(manifold, xfA, radiusA, xfB, radiusB);

		vc->normal = worldManifold.normal;

		int pointCount = vc->pointCount;
		for (int j = 0; j < pointCount; ++j)
		{
			cb2VelocityConstraintPoint* vcp = vc->points + j;

			vcp->rA = worldManifold.points[j] - cA;
			vcp->rB = worldManifold.points[j] - cB;

			float rnA = cb2Cross(vcp->rA, vc->normal);
			float rnB = cb2Cross(vcp->rB, vc->normal);

			float kNormal = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			vcp->normalMass = kNormal > 0.0f ? 1.0f / kNormal : 0.0f;

			ci::Vec2f tangent = cb2Cross(vc->normal, 1.0f);

			float rtA = cb2Cross(vcp->rA, tangent);
			float rtB = cb2Cross(vcp->rB, tangent);

			float kTangent = mA + mB + iA * rtA * rtA + iB * rtB * rtB;

			vcp->tangentMass = kTangent > 0.0f ? 1.0f /  kTangent : 0.0f;

			// Setup a velocity bias for restitution.
			vcp->velocityBias = 0.0f;
			float vRel = cb2Dot(vc->normal, vB + cb2Cross(wB, vcp->rB) - vA - cb2Cross(wA, vcp->rA));
			if (vRel < -cb2_velocityThreshold)
			{
				vcp->velocityBias = -vc->restitution * vRel;
			}
		}

		// If we have two points, then prepare the block solver.
		if (vc->pointCount == 2)
		{
			cb2VelocityConstraintPoint* vcp1 = vc->points + 0;
			cb2VelocityConstraintPoint* vcp2 = vc->points + 1;

			float rn1A = cb2Cross(vcp1->rA, vc->normal);
			float rn1B = cb2Cross(vcp1->rB, vc->normal);
			float rn2A = cb2Cross(vcp2->rA, vc->normal);
			float rn2B = cb2Cross(vcp2->rB, vc->normal);

			float k11 = mA + mB + iA * rn1A * rn1A + iB * rn1B * rn1B;
			float k22 = mA + mB + iA * rn2A * rn2A + iB * rn2B * rn2B;
			float k12 = mA + mB + iA * rn1A * rn2A + iB * rn1B * rn2B;

			// Ensure a reasonable condition number.
			const float k_maxConditionNumber = 1000.0f;
			if (k11 * k11 < k_maxConditionNumber * (k11 * k22 - k12 * k12))
			{
				// K is safe to invert.
				vc->K.set(k11, k12,
				          k12, k22);
				vc->normalMass = vc->K.inverted();
			}
			else
			{
				// The constraints are redundant, just use one.
				// TODO_ERIN use deepest?
				vc->pointCount = 1;
			}
		}
	}
}

void cb2ContactSolver::WarmStart()
{
	// Warm start.
	for (int i = 0; i < m_count; ++i)
	{
		cb2ContactVelocityConstraint* vc = m_velocityConstraints + i;

		int indexA = vc->indexA;
		int indexB = vc->indexB;
		float mA = vc->invMassA;
		float iA = vc->invIA;
		float mB = vc->invMassB;
		float iB = vc->invIB;
		int pointCount = vc->pointCount;

		ci::Vec2f vA = m_velocities[indexA].v;
		float wA = m_velocities[indexA].w;
		ci::Vec2f vB = m_velocities[indexB].v;
		float wB = m_velocities[indexB].w;

		ci::Vec2f normal = vc->normal;
		ci::Vec2f tangent = cb2Cross(normal, 1.0f);

		for (int j = 0; j < pointCount; ++j)
		{
			cb2VelocityConstraintPoint* vcp = vc->points + j;
			ci::Vec2f P = vcp->normalImpulse * normal + vcp->tangentImpulse * tangent;
			wA -= iA * cb2Cross(vcp->rA, P);
			vA -= mA * P;
			wB += iB * cb2Cross(vcp->rB, P);
			vB += mB * P;
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void cb2ContactSolver::SolveVelocityConstraints()
{
	for (int i = 0; i < m_count; ++i)
	{
		cb2ContactVelocityConstraint* vc = m_velocityConstraints + i;

		int indexA = vc->indexA;
		int indexB = vc->indexB;
		float mA = vc->invMassA;
		float iA = vc->invIA;
		float mB = vc->invMassB;
		float iB = vc->invIB;
		int pointCount = vc->pointCount;

		ci::Vec2f vA = m_velocities[indexA].v;
		float wA = m_velocities[indexA].w;
		ci::Vec2f vB = m_velocities[indexB].v;
		float wB = m_velocities[indexB].w;

		ci::Vec2f normal = vc->normal;
		ci::Vec2f tangent = cb2Cross(normal, 1.0f);
		float friction = vc->friction;

		cb2Assert(pointCount == 1 || pointCount == 2);

		// Solve tangent constraints first because non-penetration is more important
		// than friction.
		for (int j = 0; j < pointCount; ++j)
		{
			cb2VelocityConstraintPoint* vcp = vc->points + j;

			// Relative velocity at contact
			ci::Vec2f dv = vB + cb2Cross(wB, vcp->rB) - vA - cb2Cross(wA, vcp->rA);

			// Compute tangent force
			float vt = cb2Dot(dv, tangent) - vc->tangentSpeed;
			float lambda = vcp->tangentMass * (-vt);

			// cb2Clamp the accumulated force
			float maxFriction = friction * vcp->normalImpulse;
			float newImpulse = cb2Clamp(vcp->tangentImpulse + lambda, -maxFriction, maxFriction);
			lambda = newImpulse - vcp->tangentImpulse;
			vcp->tangentImpulse = newImpulse;

			// Apply contact impulse
			ci::Vec2f P = lambda * tangent;

			vA -= mA * P;
			wA -= iA * cb2Cross(vcp->rA, P);

			vB += mB * P;
			wB += iB * cb2Cross(vcp->rB, P);
		}

		// Solve normal constraints
		if (vc->pointCount == 1)
		{
			cb2VelocityConstraintPoint* vcp = vc->points + 0;

			// Relative velocity at contact
			ci::Vec2f dv = vB + cb2Cross(wB, vcp->rB) - vA - cb2Cross(wA, vcp->rA);

			// Compute normal impulse
			float vn = cb2Dot(dv, normal);
			float lambda = -vcp->normalMass * (vn - vcp->velocityBias);

			// cb2Clamp the accumulated impulse
			float newImpulse = cb2Max(vcp->normalImpulse + lambda, 0.0f);
			lambda = newImpulse - vcp->normalImpulse;
			vcp->normalImpulse = newImpulse;

			// Apply contact impulse
			ci::Vec2f P = lambda * normal;
			vA -= mA * P;
			wA -= iA * cb2Cross(vcp->rA, P);

			vB += mB * P;
			wB += iB * cb2Cross(vcp->rB, P);
		}
		else
		{
			// Block solver developed in collaboration with Dirk Gregorius (back in 01/07 on Box2D_Lite).
			// Build the mini LCP for this contact patch
			//
			// vn = A * x + b, vn >= 0, , vn >= 0, x >= 0 and vn_i * x_i = 0 with i = 1..2
			//
			// A = J * W * JT and J = ( -n, -r1 x n, n, r2 x n )
			// b = vn0 - velocityBias
			//
			// The system is solved using the "Total enumeration method" (s. Murty). The complementary constraint vn_i * x_i
			// implies that we must have in any solution either vn_i = 0 or x_i = 0. So for the 2D contact problem the cases
			// vn1 = 0 and vn2 = 0, x1 = 0 and x2 = 0, x1 = 0 and vn2 = 0, x2 = 0 and vn1 = 0 need to be tested. The first valid
			// solution that satisfies the problem is chosen.
			// 
			// In order to account of the accumulated impulse 'a' (because of the iterative nature of the solver which only requires
			// that the accumulated impulse is clamped and not the incremental impulse) we change the impulse variable (x_i).
			//
			// Substitute:
			// 
			// x = a + d
			// 
			// a := old total impulse
			// x := new total impulse
			// d := incremental impulse 
			//
			// For the current iteration we extend the formula for the incremental impulse
			// to compute the new total impulse:
			//
			// vn = A * d + b
			//    = A * (x - a) + b
			//    = A * x + b - A * a
			//    = A * x + b'
			// b' = b - A * a;

			cb2VelocityConstraintPoint* cp1 = vc->points + 0;
			cb2VelocityConstraintPoint* cp2 = vc->points + 1;

			ci::Vec2f a(cp1->normalImpulse, cp2->normalImpulse);
			cb2Assert(a.x >= 0.0f && a.y >= 0.0f);

			// Relative velocity at contact
			ci::Vec2f dv1 = vB + cb2Cross(wB, cp1->rB) - vA - cb2Cross(wA, cp1->rA);
			ci::Vec2f dv2 = vB + cb2Cross(wB, cp2->rB) - vA - cb2Cross(wA, cp2->rA);

			// Compute normal velocity
			float vn1 = cb2Dot(dv1, normal);
			float vn2 = cb2Dot(dv2, normal);

			ci::Vec2f b;
			b.x = vn1 - cp1->velocityBias;
			b.y = vn2 - cp2->velocityBias;

			// Compute b'
			b -= cb2Mul(vc->K, a);

			const float k_errorTol = 1e-3f;
			CB2_NOT_USED(k_errorTol);

			for (;;)
			{
				//
				// Case 1: vn = 0
				//
				// 0 = A * x + b'
				//
				// Solve for x:
				//
				// x = - inv(A) * b'
				//
				ci::Vec2f x = - cb2Mul(vc->normalMass, b);

				if (x.x >= 0.0f && x.y >= 0.0f)
				{
					// Get the incremental impulse
					ci::Vec2f d = x - a;

					// Apply incremental impulse
					ci::Vec2f P1 = d.x * normal;
					ci::Vec2f P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (cb2Cross(cp1->rA, P1) + cb2Cross(cp2->rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (cb2Cross(cp1->rB, P1) + cb2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if CB2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + cb2Cross(wB, cp1->rB) - vA - cb2Cross(wA, cp1->rA);
					dv2 = vB + cb2Cross(wB, cp2->rB) - vA - cb2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn1 = cb2Dot(dv1, normal);
					vn2 = cb2Dot(dv2, normal);

					cb2Assert(cb2Abs(vn1 - cp1->velocityBias) < k_errorTol);
					cb2Assert(cb2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 2: vn1 = 0 and x2 = 0
				//
				//   0 = a11 * x1 + a12 * 0 + b1' 
				// vn2 = a21 * x1 + a22 * 0 + cb2'
				//
				x.x = - cp1->normalMass * b.x;
				x.y = 0.0f;
				vn1 = 0.0f;
				vn2 = vc->K.m10 * x.x + b.y;

				if (x.x >= 0.0f && vn2 >= 0.0f)
				{
					// Get the incremental impulse
					ci::Vec2f d = x - a;

					// Apply incremental impulse
					ci::Vec2f P1 = d.x * normal;
					ci::Vec2f P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (cb2Cross(cp1->rA, P1) + cb2Cross(cp2->rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (cb2Cross(cp1->rB, P1) + cb2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if CB2_DEBUG_SOLVER == 1
					// Postconditions
					dv1 = vB + cb2Cross(wB, cp1->rB) - vA - cb2Cross(wA, cp1->rA);

					// Compute normal velocity
					vn1 = cb2Dot(dv1, normal);

					cb2Assert(cb2Abs(vn1 - cp1->velocityBias) < k_errorTol);
#endif
					break;
				}


				//
				// Case 3: vn2 = 0 and x1 = 0
				//
				// vn1 = a11 * 0 + a12 * x2 + b1' 
				//   0 = a21 * 0 + a22 * x2 + cb2'
				//
				x.x = 0.0f;
				x.y = - cp2->normalMass * b.y;
				vn1 = vc->K.m01 * x.y + b.x;
				vn2 = 0.0f;

				if (x.y >= 0.0f && vn1 >= 0.0f)
				{
					// Resubstitute for the incremental impulse
					ci::Vec2f d = x - a;

					// Apply incremental impulse
					ci::Vec2f P1 = d.x * normal;
					ci::Vec2f P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (cb2Cross(cp1->rA, P1) + cb2Cross(cp2->rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (cb2Cross(cp1->rB, P1) + cb2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

#if CB2_DEBUG_SOLVER == 1
					// Postconditions
					dv2 = vB + cb2Cross(wB, cp2->rB) - vA - cb2Cross(wA, cp2->rA);

					// Compute normal velocity
					vn2 = cb2Dot(dv2, normal);

					cb2Assert(cb2Abs(vn2 - cp2->velocityBias) < k_errorTol);
#endif
					break;
				}

				//
				// Case 4: x1 = 0 and x2 = 0
				// 
				// vn1 = b1
				// vn2 = cb2;
				x.x = 0.0f;
				x.y = 0.0f;
				vn1 = b.x;
				vn2 = b.y;

				if (vn1 >= 0.0f && vn2 >= 0.0f )
				{
					// Resubstitute for the incremental impulse
					ci::Vec2f d = x - a;

					// Apply incremental impulse
					ci::Vec2f P1 = d.x * normal;
					ci::Vec2f P2 = d.y * normal;
					vA -= mA * (P1 + P2);
					wA -= iA * (cb2Cross(cp1->rA, P1) + cb2Cross(cp2->rA, P2));

					vB += mB * (P1 + P2);
					wB += iB * (cb2Cross(cp1->rB, P1) + cb2Cross(cp2->rB, P2));

					// Accumulate
					cp1->normalImpulse = x.x;
					cp2->normalImpulse = x.y;

					break;
				}

				// No solution, give up. This is hit sometimes, but it doesn't seem to matter.
				break;
			}
		}

		m_velocities[indexA].v = vA;
		m_velocities[indexA].w = wA;
		m_velocities[indexB].v = vB;
		m_velocities[indexB].w = wB;
	}
}

void cb2ContactSolver::StoreImpulses()
{
	for (int i = 0; i < m_count; ++i)
	{
		cb2ContactVelocityConstraint* vc = m_velocityConstraints + i;
		cb2Manifold* manifold = m_contacts[vc->contactIndex]->GetManifold();

		for (int j = 0; j < vc->pointCount; ++j)
		{
			manifold->points[j].normalImpulse = vc->points[j].normalImpulse;
			manifold->points[j].tangentImpulse = vc->points[j].tangentImpulse;
		}
	}
}

struct cb2PositionSolverManifold
{
	void Initialize(cb2ContactPositionConstraint* pc, const cb2Transform& xfA, const cb2Transform& xfB, int index)
	{
		cb2Assert(pc->pointCount > 0);

		switch (pc->type)
		{
		case cb2Manifold::e_circles:
			{
				ci::Vec2f pointA = cb2Mul(xfA, pc->localPoint);
				ci::Vec2f pointB = cb2Mul(xfB, pc->localPoints[0]);
				normal = pointB - pointA;
				normal.normalize();
				point = 0.5f * (pointA + pointB);
				separation = cb2Dot(pointB - pointA, normal) - pc->radiusA - pc->radiusB;
			}
			break;

		case cb2Manifold::e_faceA:
			{
				normal = cb2Mul(xfA.q, pc->localNormal);
				ci::Vec2f planePoint = cb2Mul(xfA, pc->localPoint);

				ci::Vec2f clipPoint = cb2Mul(xfB, pc->localPoints[index]);
				separation = cb2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
				point = clipPoint;
			}
			break;

		case cb2Manifold::e_faceB:
			{
				normal = cb2Mul(xfB.q, pc->localNormal);
				ci::Vec2f planePoint = cb2Mul(xfB, pc->localPoint);

				ci::Vec2f clipPoint = cb2Mul(xfA, pc->localPoints[index]);
				separation = cb2Dot(clipPoint - planePoint, normal) - pc->radiusA - pc->radiusB;
				point = clipPoint;

				// Ensure normal points from A to B
				normal = -normal;
			}
			break;
		}
	}

	ci::Vec2f normal;
	ci::Vec2f point;
	float separation;
};

// Sequential solver.
bool cb2ContactSolver::SolvePositionConstraints()
{
	float minSeparation = 0.0f;

	for (int i = 0; i < m_count; ++i)
	{
		cb2ContactPositionConstraint* pc = m_positionConstraints + i;

		int indexA = pc->indexA;
		int indexB = pc->indexB;
		ci::Vec2f localCenterA = pc->localCenterA;
		float mA = pc->invMassA;
		float iA = pc->invIA;
		ci::Vec2f localCenterB = pc->localCenterB;
		float mB = pc->invMassB;
		float iB = pc->invIB;
		int pointCount = pc->pointCount;

		ci::Vec2f cA = m_positions[indexA].c;
		float aA = m_positions[indexA].a;

		ci::Vec2f cB = m_positions[indexB].c;
		float aB = m_positions[indexB].a;

		// Solve normal constraints
		for (int j = 0; j < pointCount; ++j)
		{
			cb2Transform xfA, xfB;
			xfA.q.set(aA);
			xfB.q.set(aB);
			xfA.p = cA - cb2Mul(xfA.q, localCenterA);
			xfB.p = cB - cb2Mul(xfB.q, localCenterB);

			cb2PositionSolverManifold psm;
			psm.Initialize(pc, xfA, xfB, j);
			ci::Vec2f normal = psm.normal;

			ci::Vec2f point = psm.point;
			float separation = psm.separation;

			ci::Vec2f rA = point - cA;
			ci::Vec2f rB = point - cB;

			// Track max constraint error.
			minSeparation = cb2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float C = cb2Clamp(cb2_baumgarte * (separation + cb2_linearSlop), -cb2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			float rnA = cb2Cross(rA, normal);
			float rnB = cb2Cross(rB, normal);
			float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float impulse = K > 0.0f ? - C / K : 0.0f;

			ci::Vec2f P = impulse * normal;

			cA -= mA * P;
			aA -= iA * cb2Cross(rA, P);

			cB += mB * P;
			aB += iB * cb2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;

		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -cb2_linearSlop because we don't
	// push the separation above -cb2_linearSlop.
	return minSeparation >= -3.0f * cb2_linearSlop;
}

// Sequential position solver for position constraints.
bool cb2ContactSolver::SolveTOIPositionConstraints(int toiIndexA, int toiIndexB)
{
	float minSeparation = 0.0f;

	for (int i = 0; i < m_count; ++i)
	{
		cb2ContactPositionConstraint* pc = m_positionConstraints + i;

		int indexA = pc->indexA;
		int indexB = pc->indexB;
		ci::Vec2f localCenterA = pc->localCenterA;
		ci::Vec2f localCenterB = pc->localCenterB;
		int pointCount = pc->pointCount;

		float mA = 0.0f;
		float iA = 0.0f;
		if (indexA == toiIndexA || indexA == toiIndexB)
		{
			mA = pc->invMassA;
			iA = pc->invIA;
		}

		float mB = 0.0f;
		float iB = 0.;
		if (indexB == toiIndexA || indexB == toiIndexB)
		{
			mB = pc->invMassB;
			iB = pc->invIB;
		}

		ci::Vec2f cA = m_positions[indexA].c;
		float aA = m_positions[indexA].a;

		ci::Vec2f cB = m_positions[indexB].c;
		float aB = m_positions[indexB].a;

		// Solve normal constraints
		for (int j = 0; j < pointCount; ++j)
		{
			cb2Transform xfA, xfB;
			xfA.q.set(aA);
			xfB.q.set(aB);
			xfA.p = cA - cb2Mul(xfA.q, localCenterA);
			xfB.p = cB - cb2Mul(xfB.q, localCenterB);

			cb2PositionSolverManifold psm;
			psm.Initialize(pc, xfA, xfB, j);
			ci::Vec2f normal = psm.normal;

			ci::Vec2f point = psm.point;
			float separation = psm.separation;

			ci::Vec2f rA = point - cA;
			ci::Vec2f rB = point - cB;

			// Track max constraint error.
			minSeparation = cb2Min(minSeparation, separation);

			// Prevent large corrections and allow slop.
			float C = cb2Clamp(cb2_toiBaugarte * (separation + cb2_linearSlop), -cb2_maxLinearCorrection, 0.0f);

			// Compute the effective mass.
			float rnA = cb2Cross(rA, normal);
			float rnB = cb2Cross(rB, normal);
			float K = mA + mB + iA * rnA * rnA + iB * rnB * rnB;

			// Compute normal impulse
			float impulse = K > 0.0f ? - C / K : 0.0f;

			ci::Vec2f P = impulse * normal;

			cA -= mA * P;
			aA -= iA * cb2Cross(rA, P);

			cB += mB * P;
			aB += iB * cb2Cross(rB, P);
		}

		m_positions[indexA].c = cA;
		m_positions[indexA].a = aA;

		m_positions[indexB].c = cB;
		m_positions[indexB].a = aB;
	}

	// We can't expect minSpeparation >= -cb2_linearSlop because we don't
	// push the separation above -cb2_linearSlop.
	return minSeparation >= -1.5f * cb2_linearSlop;
}
