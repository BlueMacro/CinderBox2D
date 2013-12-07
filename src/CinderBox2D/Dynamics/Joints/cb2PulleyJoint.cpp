/*
* Copyright (c) 2007 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Dynamics/Joints/cb2PulleyJoint.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>

// Pulley:
// length1 = norm(p1 - s1)
// length2 = norm(p2 - s2)
// C0 = (length1 + ratio * length2)_initial
// C = C0 - (length1 + ratio * length2)
// u1 = (p1 - s1) / norm(p1 - s1)
// u2 = (p2 - s2) / norm(p2 - s2)
// Cdot = -dot(u1, v1 + cross(w1, r1)) - ratio * dot(u2, v2 + cross(w2, r2))
// J = -[u1 cross(r1, u1) ratio * u2  ratio * cross(r2, u2)]
// K = J * invM * JT
//   = invMass1 + invI1 * cross(r1, u1)^2 + ratio^2 * (invMass2 + invI2 * cross(r2, u2)^2)

void cb2PulleyJointDef::Initialize(cb2Body* bA, cb2Body* bB,
				const ci::Vec2f& groundA, const ci::Vec2f& groundB,
				const ci::Vec2f& anchorA, const ci::Vec2f& anchorB,
				float r)
{
	bodyA = bA;
	bodyB = bB;
	groundAnchorA = groundA;
	groundAnchorB = groundB;
	localAnchorA = bodyA->GetLocalPoint(anchorA);
	localAnchorB = bodyB->GetLocalPoint(anchorB);
	ci::Vec2f dA = anchorA - groundA;
	lengthA = dA.length();
	ci::Vec2f dB = anchorB - groundB;
	lengthB = dB.length();
	ratio = r;
	cb2Assert(ratio > cb2_epsilon);
}

cb2PulleyJoint::cb2PulleyJoint(const cb2PulleyJointDef* def)
: cb2Joint(def)
{
	m_groundAnchorA = def->groundAnchorA;
	m_groundAnchorB = def->groundAnchorB;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;

	m_lengthA = def->lengthA;
	m_lengthB = def->lengthB;

	cb2Assert(def->ratio != 0.0f);
	m_ratio = def->ratio;

	m_constant = def->lengthA + m_ratio * def->lengthB;

	m_impulse = 0.0f;
}

void cb2PulleyJoint::InitVelocityConstraints(const cb2SolverData& data)
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

	// Get the pulley axes.
	m_uA = cA + m_rA - m_groundAnchorA;
	m_uB = cB + m_rB - m_groundAnchorB;

	float lengthA = m_uA.length();
	float lengthB = m_uB.length();

	if (lengthA > 10.0f * cb2_linearSlop)
	{
		m_uA *= 1.0f / lengthA;
	}
	else
	{
		cb2::setZero(m_uA);
	}

	if (lengthB > 10.0f * cb2_linearSlop)
	{
		m_uB *= 1.0f / lengthB;
	}
	else
	{
		cb2::setZero(m_uB);
	}

	// Compute effective mass.
	float ruA = cb2Cross(m_rA, m_uA);
	float ruB = cb2Cross(m_rB, m_uB);

	float mA = m_invMassA + m_invIA * ruA * ruA;
	float mB = m_invMassB + m_invIB * ruB * ruB;

	m_mass = mA + m_ratio * m_ratio * mB;

	if (m_mass > 0.0f)
	{
		m_mass = 1.0f / m_mass;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support variable time steps.
		m_impulse *= data.step.dtRatio;

		// Warm starting.
		ci::Vec2f PA = -(m_impulse) * m_uA;
		ci::Vec2f PB = (-m_ratio * m_impulse) * m_uB;

		vA += m_invMassA * PA;
		wA += m_invIA * cb2Cross(m_rA, PA);
		vB += m_invMassB * PB;
		wB += m_invIB * cb2Cross(m_rB, PB);
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

void cb2PulleyJoint::SolveVelocityConstraints(const cb2SolverData& data)
{
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	ci::Vec2f vpA = vA + cb2Cross(wA, m_rA);
	ci::Vec2f vpB = vB + cb2Cross(wB, m_rB);

	float Cdot = -cb2Dot(m_uA, vpA) - m_ratio * cb2Dot(m_uB, vpB);
	float impulse = -m_mass * Cdot;
	m_impulse += impulse;

	ci::Vec2f PA = -impulse * m_uA;
	ci::Vec2f PB = -m_ratio * impulse * m_uB;
	vA += m_invMassA * PA;
	wA += m_invIA * cb2Cross(m_rA, PA);
	vB += m_invMassB * PB;
	wB += m_invIB * cb2Cross(m_rB, PB);

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool cb2PulleyJoint::SolvePositionConstraints(const cb2SolverData& data)
{
	ci::Vec2f cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	ci::Vec2f cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;

	cb2Rot qA(aA), qB(aB);

	ci::Vec2f rA = cb2Mul(qA, m_localAnchorA - m_localCenterA);
	ci::Vec2f rB = cb2Mul(qB, m_localAnchorB - m_localCenterB);

	// Get the pulley axes.
	ci::Vec2f uA = cA + rA - m_groundAnchorA;
	ci::Vec2f uB = cB + rB - m_groundAnchorB;

	float lengthA = uA.length();
	float lengthB = uB.length();

	if (lengthA > 10.0f * cb2_linearSlop)
	{
		uA *= 1.0f / lengthA;
	}
	else
	{
		cb2::setZero(uA);
	}

	if (lengthB > 10.0f * cb2_linearSlop)
	{
		uB *= 1.0f / lengthB;
	}
	else
	{
		cb2::setZero(uB);
	}

	// Compute effective mass.
	float ruA = cb2Cross(rA, uA);
	float ruB = cb2Cross(rB, uB);

	float mA = m_invMassA + m_invIA * ruA * ruA;
	float mB = m_invMassB + m_invIB * ruB * ruB;

	float mass = mA + m_ratio * m_ratio * mB;

	if (mass > 0.0f)
	{
		mass = 1.0f / mass;
	}

	float C = m_constant - lengthA - m_ratio * lengthB;
	float linearError = cb2Abs(C);

	float impulse = -mass * C;

	ci::Vec2f PA = -impulse * uA;
	ci::Vec2f PB = -m_ratio * impulse * uB;

	cA += m_invMassA * PA;
	aA += m_invIA * cb2Cross(rA, PA);
	cB += m_invMassB * PB;
	aB += m_invIB * cb2Cross(rB, PB);

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return linearError < cb2_linearSlop;
}

ci::Vec2f cb2PulleyJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

ci::Vec2f cb2PulleyJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

ci::Vec2f cb2PulleyJoint::GetReactionForce(float inv_dt) const
{
	ci::Vec2f P = m_impulse * m_uB;
	return inv_dt * P;
}

float cb2PulleyJoint::GetReactionTorque(float inv_dt) const
{
	CB2_NOT_USED(inv_dt);
	return 0.0f;
}

ci::Vec2f cb2PulleyJoint::GetGroundAnchorA() const
{
	return m_groundAnchorA;
}

ci::Vec2f cb2PulleyJoint::GetGroundAnchorB() const
{
	return m_groundAnchorB;
}

float cb2PulleyJoint::GetLengthA() const
{
	return m_lengthA;
}

float cb2PulleyJoint::GetLengthB() const
{
  return m_lengthB;
}

float cb2PulleyJoint::GetRatio() const
{
	return m_ratio;
}

float cb2PulleyJoint::GetCurrentLengthA() const
{
	ci::Vec2f p = m_bodyA->GetWorldPoint(m_localAnchorA);
	ci::Vec2f s = m_groundAnchorA;
	ci::Vec2f d = p - s;
	return d.length();
}

float cb2PulleyJoint::GetCurrentLengthB() const
{
	ci::Vec2f p = m_bodyB->GetWorldPoint(m_localAnchorB);
	ci::Vec2f s = m_groundAnchorB;
	ci::Vec2f d = p - s;
	return d.length();
}

void cb2PulleyJoint::Dump()
{
	int indexA = m_bodyA->m_islandIndex;
	int indexB = m_bodyB->m_islandIndex;

	cb2Log("  cb2PulleyJointDef jd;\n");
	cb2Log("  jd.bodyA = bodies[%d];\n", indexA);
	cb2Log("  jd.bodyB = bodies[%d];\n", indexB);
	cb2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	cb2Log("  jd.groundAnchorA.set(%.15lef, %.15lef);\n", m_groundAnchorA.x, m_groundAnchorA.y);
	cb2Log("  jd.groundAnchorB.set(%.15lef, %.15lef);\n", m_groundAnchorB.x, m_groundAnchorB.y);
	cb2Log("  jd.localAnchorA.set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	cb2Log("  jd.localAnchorB.set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	cb2Log("  jd.lengthA = %.15lef;\n", m_lengthA);
	cb2Log("  jd.lengthB = %.15lef;\n", m_lengthB);
	cb2Log("  jd.ratio = %.15lef;\n", m_ratio);
	cb2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}

void cb2PulleyJoint::ShiftOrigin(const ci::Vec2f& newOrigin)
{
	m_groundAnchorA -= newOrigin;
	m_groundAnchorB -= newOrigin;
}
