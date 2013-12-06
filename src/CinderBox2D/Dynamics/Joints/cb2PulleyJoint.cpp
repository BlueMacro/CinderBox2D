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

void b2PulleyJointDef::Initialize(b2Body* bA, b2Body* bB,
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
	b2Assert(ratio > b2_epsilon);
}

b2PulleyJoint::b2PulleyJoint(const b2PulleyJointDef* def)
: b2Joint(def)
{
	m_groundAnchorA = def->groundAnchorA;
	m_groundAnchorB = def->groundAnchorB;
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;

	m_lengthA = def->lengthA;
	m_lengthB = def->lengthB;

	b2Assert(def->ratio != 0.0f);
	m_ratio = def->ratio;

	m_constant = def->lengthA + m_ratio * def->lengthB;

	m_impulse = 0.0f;
}

void b2PulleyJoint::InitVelocityConstraints(const b2SolverData& data)
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

	b2Rot qA(aA), qB(aB);

	m_rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

	// Get the pulley axes.
	m_uA = cA + m_rA - m_groundAnchorA;
	m_uB = cB + m_rB - m_groundAnchorB;

	float lengthA = m_uA.length();
	float lengthB = m_uB.length();

	if (lengthA > 10.0f * b2_linearSlop)
	{
		m_uA *= 1.0f / lengthA;
	}
	else
	{
		cb2::setZero(m_uA);
	}

	if (lengthB > 10.0f * b2_linearSlop)
	{
		m_uB *= 1.0f / lengthB;
	}
	else
	{
		cb2::setZero(m_uB);
	}

	// Compute effective mass.
	float ruA = b2Cross(m_rA, m_uA);
	float ruB = b2Cross(m_rB, m_uB);

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
		wA += m_invIA * b2Cross(m_rA, PA);
		vB += m_invMassB * PB;
		wB += m_invIB * b2Cross(m_rB, PB);
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

void b2PulleyJoint::SolveVelocityConstraints(const b2SolverData& data)
{
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	ci::Vec2f vpA = vA + b2Cross(wA, m_rA);
	ci::Vec2f vpB = vB + b2Cross(wB, m_rB);

	float Cdot = -b2Dot(m_uA, vpA) - m_ratio * b2Dot(m_uB, vpB);
	float impulse = -m_mass * Cdot;
	m_impulse += impulse;

	ci::Vec2f PA = -impulse * m_uA;
	ci::Vec2f PB = -m_ratio * impulse * m_uB;
	vA += m_invMassA * PA;
	wA += m_invIA * b2Cross(m_rA, PA);
	vB += m_invMassB * PB;
	wB += m_invIB * b2Cross(m_rB, PB);

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool b2PulleyJoint::SolvePositionConstraints(const b2SolverData& data)
{
	ci::Vec2f cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	ci::Vec2f cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;

	b2Rot qA(aA), qB(aB);

	ci::Vec2f rA = b2Mul(qA, m_localAnchorA - m_localCenterA);
	ci::Vec2f rB = b2Mul(qB, m_localAnchorB - m_localCenterB);

	// Get the pulley axes.
	ci::Vec2f uA = cA + rA - m_groundAnchorA;
	ci::Vec2f uB = cB + rB - m_groundAnchorB;

	float lengthA = uA.length();
	float lengthB = uB.length();

	if (lengthA > 10.0f * b2_linearSlop)
	{
		uA *= 1.0f / lengthA;
	}
	else
	{
		cb2::setZero(uA);
	}

	if (lengthB > 10.0f * b2_linearSlop)
	{
		uB *= 1.0f / lengthB;
	}
	else
	{
		cb2::setZero(uB);
	}

	// Compute effective mass.
	float ruA = b2Cross(rA, uA);
	float ruB = b2Cross(rB, uB);

	float mA = m_invMassA + m_invIA * ruA * ruA;
	float mB = m_invMassB + m_invIB * ruB * ruB;

	float mass = mA + m_ratio * m_ratio * mB;

	if (mass > 0.0f)
	{
		mass = 1.0f / mass;
	}

	float C = m_constant - lengthA - m_ratio * lengthB;
	float linearError = b2Abs(C);

	float impulse = -mass * C;

	ci::Vec2f PA = -impulse * uA;
	ci::Vec2f PB = -m_ratio * impulse * uB;

	cA += m_invMassA * PA;
	aA += m_invIA * b2Cross(rA, PA);
	cB += m_invMassB * PB;
	aB += m_invIB * b2Cross(rB, PB);

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return linearError < b2_linearSlop;
}

ci::Vec2f b2PulleyJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

ci::Vec2f b2PulleyJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

ci::Vec2f b2PulleyJoint::GetReactionForce(float inv_dt) const
{
	ci::Vec2f P = m_impulse * m_uB;
	return inv_dt * P;
}

float b2PulleyJoint::GetReactionTorque(float inv_dt) const
{
	B2_NOT_USED(inv_dt);
	return 0.0f;
}

ci::Vec2f b2PulleyJoint::GetGroundAnchorA() const
{
	return m_groundAnchorA;
}

ci::Vec2f b2PulleyJoint::GetGroundAnchorB() const
{
	return m_groundAnchorB;
}

float b2PulleyJoint::GetLengthA() const
{
	ci::Vec2f p = m_bodyA->GetWorldPoint(m_localAnchorA);
	ci::Vec2f s = m_groundAnchorA;
	ci::Vec2f d = p - s;
	return d.length();
}

float b2PulleyJoint::GetLengthB() const
{
	ci::Vec2f p = m_bodyB->GetWorldPoint(m_localAnchorB);
	ci::Vec2f s = m_groundAnchorB;
	ci::Vec2f d = p - s;
	return d.length();
}

float b2PulleyJoint::GetRatio() const
{
	return m_ratio;
}

void b2PulleyJoint::Dump()
{
	int indexA = m_bodyA->m_islandIndex;
	int indexB = m_bodyB->m_islandIndex;

	b2Log("  b2PulleyJointDef jd;\n");
	b2Log("  jd.bodyA = bodies[%d];\n", indexA);
	b2Log("  jd.bodyB = bodies[%d];\n", indexB);
	b2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	b2Log("  jd.groundAnchorA.set(%.15lef, %.15lef);\n", m_groundAnchorA.x, m_groundAnchorA.y);
	b2Log("  jd.groundAnchorB.set(%.15lef, %.15lef);\n", m_groundAnchorB.x, m_groundAnchorB.y);
	b2Log("  jd.localAnchorA.set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	b2Log("  jd.localAnchorB.set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	b2Log("  jd.lengthA = %.15lef;\n", m_lengthA);
	b2Log("  jd.lengthB = %.15lef;\n", m_lengthB);
	b2Log("  jd.ratio = %.15lef;\n", m_ratio);
	b2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
