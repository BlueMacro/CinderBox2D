/*
* Copyright (c) 2007-2011 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Dynamics/Joints/cb2RopeJoint.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>


// Limit:
// C = norm(pB - pA) - L
// u = (pB - pA) / norm(pB - pA)
// Cdot = dot(u, vB + cross(wB, rB) - vA - cross(wA, rA))
// J = [-u -cross(rA, u) u cross(rB, u)]
// K = J * invM * JT
//   = invMassA + invIA * cross(rA, u)^2 + invMassB + invIB * cross(rB, u)^2

cb2RopeJoint::cb2RopeJoint(const cb2RopeJointDef* def)
: cb2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;

	m_maxLength = def->maxLength;

	m_mass = 0.0f;
	m_impulse = 0.0f;
	m_state = e_inactiveLimit;
	m_length = 0.0f;
}

void cb2RopeJoint::InitVelocityConstraints(const cb2SolverData& data)
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

	m_length = m_u.length();

	float C = m_length - m_maxLength;
	if (C > 0.0f)
	{
		m_state = e_atUpperLimit;
	}
	else
	{
		m_state = e_inactiveLimit;
	}

	if (m_length > cb2_linearSlop)
	{
		m_u *= 1.0f / m_length;
	}
	else
	{
		cb2::setZero(m_u);
		m_mass = 0.0f;
		m_impulse = 0.0f;
		return;
	}

	// Compute effective mass.
	float crA = cb2Cross(m_rA, m_u);
	float crB = cb2Cross(m_rB, m_u);
	float invMass = m_invMassA + m_invIA * crA * crA + m_invMassB + m_invIB * crB * crB;

	m_mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;

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

void cb2RopeJoint::SolveVelocityConstraints(const cb2SolverData& data)
{
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	// Cdot = dot(u, v + cross(w, r))
	ci::Vec2f vpA = vA + cb2Cross(wA, m_rA);
	ci::Vec2f vpB = vB + cb2Cross(wB, m_rB);
	float C = m_length - m_maxLength;
	float Cdot = cb2Dot(m_u, vpB - vpA);

	// Predictive constraint.
	if (C < 0.0f)
	{
		Cdot += data.step.inv_dt * C;
	}

	float impulse = -m_mass * Cdot;
	float oldImpulse = m_impulse;
	m_impulse = cb2Min(0.0f, m_impulse + impulse);
	impulse = m_impulse - oldImpulse;

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

bool cb2RopeJoint::SolvePositionConstraints(const cb2SolverData& data)
{
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
	float C = length - m_maxLength;

	C = cb2Clamp(C, 0.0f, cb2_maxLinearCorrection);

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

	return length - m_maxLength < cb2_linearSlop;
}

ci::Vec2f cb2RopeJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

ci::Vec2f cb2RopeJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

ci::Vec2f cb2RopeJoint::GetReactionForce(float inv_dt) const
{
	ci::Vec2f F = (inv_dt * m_impulse) * m_u;
	return F;
}

float cb2RopeJoint::GetReactionTorque(float inv_dt) const
{
	CB2_NOT_USED(inv_dt);
	return 0.0f;
}

float cb2RopeJoint::GetMaxLength() const
{
	return m_maxLength;
}

cb2LimitState cb2RopeJoint::GetLimitState() const
{
	return m_state;
}

void cb2RopeJoint::Dump()
{
	int indexA = m_bodyA->m_islandIndex;
	int indexB = m_bodyB->m_islandIndex;

	cb2Log("  cb2RopeJointDef jd;\n");
	cb2Log("  jd.bodyA = bodies[%d];\n", indexA);
	cb2Log("  jd.bodyB = bodies[%d];\n", indexB);
	cb2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	cb2Log("  jd.localAnchorA.set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	cb2Log("  jd.localAnchorB.set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	cb2Log("  jd.maxLength = %.15lef;\n", m_maxLength);
	cb2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
