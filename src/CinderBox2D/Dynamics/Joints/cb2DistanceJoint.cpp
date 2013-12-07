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

#include <CinderBox2D/Dynamics/Joints/cb2DistanceJoint.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>

// 1-D constrained system
// m (v2 - v1) = lambda
// v2 + (beta/h) * x1 + gamma * lambda = 0, gamma has units of inverse mass.
// x2 = x1 + h * v2

// 1-D mass-damper-spring system
// m (v2 - v1) + h * d * v2 + h * k * 

// C = norm(p2 - p1) - L
// u = (p2 - p1) / norm(p2 - p1)
// Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
// J = [-u -cross(r1, u) u cross(r2, u)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

void cb2DistanceJointDef::Initialize(cb2Body* b1, cb2Body* cb2,
									const ci::Vec2f& anchor1, const ci::Vec2f& anchor2)
{
	bodyA = b1;
	bodyB = cb2;
	localAnchorA = bodyA->GetLocalPoint(anchor1);
	localAnchorB = bodyB->GetLocalPoint(anchor2);
	ci::Vec2f d = anchor2 - anchor1;
	length = d.length();
}

cb2DistanceJoint::cb2DistanceJoint(const cb2DistanceJointDef* def)
: cb2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_length = def->length;
	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;
	m_impulse = 0.0f;
	m_gamma = 0.0f;
	m_bias = 0.0f;
}

void cb2DistanceJoint::InitVelocityConstraints(const cb2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;

	ci::Vec2f cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;

	ci::Vec2f cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	cb2Rot qA(aA), qB(aB);

	m_rA = cb2Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = cb2Mul(qB, m_localAnchorB - m_localCenterB);
	m_u = cB + m_rB - cA - m_rA;

	// Handle singularity.
	float length = m_u.length();
	if (length > cb2_linearSlop)
	{
		m_u *= 1.0f / length;
	}
	else
	{
		m_u.set(0.0f, 0.0f);
	}

	float crAu = cb2Cross(m_rA, m_u);
	float crBu = cb2Cross(m_rB, m_u);
	float invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu;

	// Compute the effective mass matrix.
	m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

	if (m_frequencyHz > 0.0f)
	{
		float C = length - m_length;

		// Frequency
		float omega = 2.0f * cb2_pi * m_frequencyHz;

		// Damping coefficient
		float d = 2.0f * m_mass * m_dampingRatio * omega;

		// Spring stiffness
		float k = m_mass * omega * omega;

		// magic formulas
		float h = data.step.dt;
		m_gamma = h * (d + h * k);
		m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
		m_bias = C * h * k * m_gamma;

		invMass += m_gamma;
		m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
	}
	else
	{
		m_gamma = 0.0f;
		m_bias = 0.0f;
	}

	if (data.step.warmStarting)
	{
		// Scale the impulse to support a variable time step.
		m_impulse *= data.step.dtRatio;

		ci::Vec2f P = m_impulse * m_u;
		vA -= m_invMassA * P;
		wA -= m_invIA * cb2Cross(m_rA, P);
		vB += m_invMassB * P;
		wB += m_invIB * cb2Cross(m_rB, P);
	}
	else
	{
		m_impulse = 0.0f;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void cb2DistanceJoint::SolveVelocityConstraints(const cb2SolverData& data)
{
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	// Cdot = dot(u, v + cross(w, r))
	ci::Vec2f vpA = vA + cb2Cross(wA, m_rA);
	ci::Vec2f vpB = vB + cb2Cross(wB, m_rB);
	float Cdot = cb2Dot(m_u, vpB - vpA);

	float impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse);
	m_impulse += impulse;

	ci::Vec2f P = impulse * m_u;
	vA -= m_invMassA * P;
	wA -= m_invIA * cb2Cross(m_rA, P);
	vB += m_invMassB * P;
	wB += m_invIB * cb2Cross(m_rB, P);

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool cb2DistanceJoint::SolvePositionConstraints(const cb2SolverData& data)
{
	if (m_frequencyHz > 0.0f)
	{
		// There is no position correction for soft distance constraints.
		return true;
	}

	ci::Vec2f cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	ci::Vec2f cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;

	cb2Rot qA(aA), qB(aB);

	ci::Vec2f rA = cb2Mul(qA, m_localAnchorA - m_localCenterA);
	ci::Vec2f rB = cb2Mul(qB, m_localAnchorB - m_localCenterB);
	ci::Vec2f u = cB + rB - cA - rA;
  
	float length = u.length();
  u /= length;
	float C = length - m_length;
	C = cb2Clamp(C, -cb2_maxLinearCorrection, cb2_maxLinearCorrection);

	float impulse = -m_mass * C;
	ci::Vec2f P = impulse * u;

	cA -= m_invMassA * P;
	aA -= m_invIA * cb2Cross(rA, P);
	cB += m_invMassB * P;
	aB += m_invIB * cb2Cross(rB, P);

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return cb2Abs(C) < cb2_linearSlop;
}

ci::Vec2f cb2DistanceJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

ci::Vec2f cb2DistanceJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

ci::Vec2f cb2DistanceJoint::GetReactionForce(float inv_dt) const
{
	ci::Vec2f F = (inv_dt * m_impulse) * m_u;
	return F;
}

float cb2DistanceJoint::GetReactionTorque(float inv_dt) const
{
	CB2_NOT_USED(inv_dt);
	return 0.0f;
}

void cb2DistanceJoint::Dump()
{
	int indexA = m_bodyA->m_islandIndex;
	int indexB = m_bodyB->m_islandIndex;

	cb2Log("  cb2DistanceJointDef jd;\n");
	cb2Log("  jd.bodyA = bodies[%d];\n", indexA);
	cb2Log("  jd.bodyB = bodies[%d];\n", indexB);
	cb2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	cb2Log("  jd.localAnchorA.set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	cb2Log("  jd.localAnchorB.set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	cb2Log("  jd.length = %.15lef;\n", m_length);
	cb2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
	cb2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
	cb2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
