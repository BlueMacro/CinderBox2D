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

#include <CinderBox2D/Dynamics/Joints/cb2GearJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2RevoluteJoint.h>
#include <CinderBox2D/Dynamics/Joints/cb2PrismaticJoint.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>

// Gear Joint:
// C0 = (coordinate1 + ratio * coordinate2)_initial
// C = (coordinate1 + ratio * coordinate2) - C0 = 0
// J = [J1 ratio * J2]
// K = J * invM * JT
//   = J1 * invM1 * J1T + ratio * ratio * J2 * invM2 * J2T
//
// Revolute:
// coordinate = rotation
// Cdot = angularVelocity
// J = [0 0 1]
// K = J * invM * JT = invI
//
// Prismatic:
// coordinate = dot(p - pg, ug)
// Cdot = dot(v + cross(w, r), ug)
// J = [ug cross(r, ug)]
// K = J * invM * JT = invMass + invI * cross(r, ug)^2

cb2GearJoint::cb2GearJoint(const cb2GearJointDef* def)
: cb2Joint(def)
{
	m_joint1 = def->joint1;
	m_joint2 = def->joint2;

	m_typeA = m_joint1->GetType();
	m_typeB = m_joint2->GetType();

	cb2Assert(m_typeA == e_revoluteJoint || m_typeA == e_prismaticJoint);
	cb2Assert(m_typeB == e_revoluteJoint || m_typeB == e_prismaticJoint);

	float coordinateA, coordinateB;

	// TODO_ERIN there might be some problem with the joint edges in cb2Joint.

	m_bodyC = m_joint1->GetBodyA();
	m_bodyA = m_joint1->GetBodyB();

	// Get geometry of joint1
	cb2Transform xfA = m_bodyA->m_xf;
	float aA = m_bodyA->m_sweep.a;
	cb2Transform xfC = m_bodyC->m_xf;
	float aC = m_bodyC->m_sweep.a;

	if (m_typeA == e_revoluteJoint)
	{
		cb2RevoluteJoint* revolute = (cb2RevoluteJoint*)def->joint1;
		m_localAnchorC = revolute->m_localAnchorA;
		m_localAnchorA = revolute->m_localAnchorB;
		m_referenceAngleA = revolute->m_referenceAngle;

		coordinateA = aA - aC - m_referenceAngleA;
	}
	else
	{
		cb2PrismaticJoint* prismatic = (cb2PrismaticJoint*)def->joint1;
		m_localAnchorC = prismatic->m_localAnchorA;
		m_localAnchorA = prismatic->m_localAnchorB;
		m_referenceAngleA = prismatic->m_referenceAngle;
		m_localAxisC = prismatic->m_localXAxisA;

		ci::Vec2f pC = m_localAnchorC;
		ci::Vec2f pA = cb2MulT(xfC.q, cb2Mul(xfA.q, m_localAnchorA) + (xfA.p - xfC.p));
		coordinateA = cb2Dot(pA - pC, m_localAxisC);
	}

	m_bodyD = m_joint2->GetBodyA();
	m_bodyB = m_joint2->GetBodyB();

	// Get geometry of joint2
	cb2Transform xfB = m_bodyB->m_xf;
	float aB = m_bodyB->m_sweep.a;
	cb2Transform xfD = m_bodyD->m_xf;
	float aD = m_bodyD->m_sweep.a;

	if (m_typeB == e_revoluteJoint)
	{
		cb2RevoluteJoint* revolute = (cb2RevoluteJoint*)def->joint2;
		m_localAnchorD = revolute->m_localAnchorA;
		m_localAnchorB = revolute->m_localAnchorB;
		m_referenceAngleB = revolute->m_referenceAngle;

		coordinateB = aB - aD - m_referenceAngleB;
	}
	else
	{
		cb2PrismaticJoint* prismatic = (cb2PrismaticJoint*)def->joint2;
		m_localAnchorD = prismatic->m_localAnchorA;
		m_localAnchorB = prismatic->m_localAnchorB;
		m_referenceAngleB = prismatic->m_referenceAngle;
		m_localAxisD = prismatic->m_localXAxisA;

		ci::Vec2f pD = m_localAnchorD;
		ci::Vec2f pB = cb2MulT(xfD.q, cb2Mul(xfB.q, m_localAnchorB) + (xfB.p - xfD.p));
		coordinateB = cb2Dot(pB - pD, m_localAxisD);
	}

	m_ratio = def->ratio;

	m_constant = coordinateA + m_ratio * coordinateB;

	m_impulse = 0.0f;
}

void cb2GearJoint::InitVelocityConstraints(const cb2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_indexC = m_bodyC->m_islandIndex;
	m_indexD = m_bodyD->m_islandIndex;
	m_lcA = m_bodyA->m_sweep.localCenter;
	m_lcB = m_bodyB->m_sweep.localCenter;
	m_lcC = m_bodyC->m_sweep.localCenter;
	m_lcD = m_bodyD->m_sweep.localCenter;
	m_mA = m_bodyA->m_invMass;
	m_mB = m_bodyB->m_invMass;
	m_mC = m_bodyC->m_invMass;
	m_mD = m_bodyD->m_invMass;
	m_iA = m_bodyA->m_invI;
	m_iB = m_bodyB->m_invI;
	m_iC = m_bodyC->m_invI;
	m_iD = m_bodyD->m_invI;

	float aA = data.positions[m_indexA].a;
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;

	float aB = data.positions[m_indexB].a;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	float aC = data.positions[m_indexC].a;
	ci::Vec2f vC = data.velocities[m_indexC].v;
	float wC = data.velocities[m_indexC].w;

	float aD = data.positions[m_indexD].a;
	ci::Vec2f vD = data.velocities[m_indexD].v;
	float wD = data.velocities[m_indexD].w;

	cb2Rot qA(aA), qB(aB), qC(aC), qD(aD);

	m_mass = 0.0f;

	if (m_typeA == e_revoluteJoint)
	{
		cb2::setZero(m_JvAC);
		m_JwA = 1.0f;
		m_JwC = 1.0f;
		m_mass += m_iA + m_iC;
	}
	else
	{
		ci::Vec2f u = cb2Mul(qC, m_localAxisC);
		ci::Vec2f rC = cb2Mul(qC, m_localAnchorC - m_lcC);
		ci::Vec2f rA = cb2Mul(qA, m_localAnchorA - m_lcA);
		m_JvAC = u;
		m_JwC = cb2Cross(rC, u);
		m_JwA = cb2Cross(rA, u);
		m_mass += m_mC + m_mA + m_iC * m_JwC * m_JwC + m_iA * m_JwA * m_JwA;
	}

	if (m_typeB == e_revoluteJoint)
	{
		cb2::setZero(m_JvBD);
		m_JwB = m_ratio;
		m_JwD = m_ratio;
		m_mass += m_ratio * m_ratio * (m_iB + m_iD);
	}
	else
	{
		ci::Vec2f u = cb2Mul(qD, m_localAxisD);
		ci::Vec2f rD = cb2Mul(qD, m_localAnchorD - m_lcD);
		ci::Vec2f rB = cb2Mul(qB, m_localAnchorB - m_lcB);
		m_JvBD = m_ratio * u;
		m_JwD = m_ratio * cb2Cross(rD, u);
		m_JwB = m_ratio * cb2Cross(rB, u);
		m_mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * m_JwD * m_JwD + m_iB * m_JwB * m_JwB;
	}

	// Compute effective mass.
	m_mass = m_mass > 0.0f ? 1.0f / m_mass : 0.0f;

	if (data.step.warmStarting)
	{
		vA += (m_mA * m_impulse) * m_JvAC;
		wA += m_iA * m_impulse * m_JwA;
		vB += (m_mB * m_impulse) * m_JvBD;
		wB += m_iB * m_impulse * m_JwB;
		vC -= (m_mC * m_impulse) * m_JvAC;
		wC -= m_iC * m_impulse * m_JwC;
		vD -= (m_mD * m_impulse) * m_JvBD;
		wD -= m_iD * m_impulse * m_JwD;
	}
	else
	{
		m_impulse = 0.0f;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
	data.velocities[m_indexC].v = vC;
	data.velocities[m_indexC].w = wC;
	data.velocities[m_indexD].v = vD;
	data.velocities[m_indexD].w = wD;
}

void cb2GearJoint::SolveVelocityConstraints(const cb2SolverData& data)
{
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;
	ci::Vec2f vC = data.velocities[m_indexC].v;
	float wC = data.velocities[m_indexC].w;
	ci::Vec2f vD = data.velocities[m_indexD].v;
	float wD = data.velocities[m_indexD].w;

	float Cdot = cb2Dot(m_JvAC, vA - vC) + cb2Dot(m_JvBD, vB - vD);
	Cdot += (m_JwA * wA - m_JwC * wC) + (m_JwB * wB - m_JwD * wD);

	float impulse = -m_mass * Cdot;
	m_impulse += impulse;

	vA += (m_mA * impulse) * m_JvAC;
	wA += m_iA * impulse * m_JwA;
	vB += (m_mB * impulse) * m_JvBD;
	wB += m_iB * impulse * m_JwB;
	vC -= (m_mC * impulse) * m_JvAC;
	wC -= m_iC * impulse * m_JwC;
	vD -= (m_mD * impulse) * m_JvBD;
	wD -= m_iD * impulse * m_JwD;

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
	data.velocities[m_indexC].v = vC;
	data.velocities[m_indexC].w = wC;
	data.velocities[m_indexD].v = vD;
	data.velocities[m_indexD].w = wD;
}

bool cb2GearJoint::SolvePositionConstraints(const cb2SolverData& data)
{
	ci::Vec2f cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	ci::Vec2f cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;
	ci::Vec2f cC = data.positions[m_indexC].c;
	float aC = data.positions[m_indexC].a;
	ci::Vec2f cD = data.positions[m_indexD].c;
	float aD = data.positions[m_indexD].a;

	cb2Rot qA(aA), qB(aB), qC(aC), qD(aD);

	float linearError = 0.0f;

	float coordinateA, coordinateB;

	ci::Vec2f JvAC, JvBD;
	float JwA, JwB, JwC, JwD;
	float mass = 0.0f;

	if (m_typeA == e_revoluteJoint)
	{
		cb2::setZero(JvAC);
		JwA = 1.0f;
		JwC = 1.0f;
		mass += m_iA + m_iC;

		coordinateA = aA - aC - m_referenceAngleA;
	}
	else
	{
		ci::Vec2f u = cb2Mul(qC, m_localAxisC);
		ci::Vec2f rC = cb2Mul(qC, m_localAnchorC - m_lcC);
		ci::Vec2f rA = cb2Mul(qA, m_localAnchorA - m_lcA);
		JvAC = u;
		JwC = cb2Cross(rC, u);
		JwA = cb2Cross(rA, u);
		mass += m_mC + m_mA + m_iC * JwC * JwC + m_iA * JwA * JwA;

		ci::Vec2f pC = m_localAnchorC - m_lcC;
		ci::Vec2f pA = cb2MulT(qC, rA + (cA - cC));
		coordinateA = cb2Dot(pA - pC, m_localAxisC);
	}

	if (m_typeB == e_revoluteJoint)
	{
		cb2::setZero(JvBD);
		JwB = m_ratio;
		JwD = m_ratio;
		mass += m_ratio * m_ratio * (m_iB + m_iD);

		coordinateB = aB - aD - m_referenceAngleB;
	}
	else
	{
		ci::Vec2f u = cb2Mul(qD, m_localAxisD);
		ci::Vec2f rD = cb2Mul(qD, m_localAnchorD - m_lcD);
		ci::Vec2f rB = cb2Mul(qB, m_localAnchorB - m_lcB);
		JvBD = m_ratio * u;
		JwD = m_ratio * cb2Cross(rD, u);
		JwB = m_ratio * cb2Cross(rB, u);
		mass += m_ratio * m_ratio * (m_mD + m_mB) + m_iD * JwD * JwD + m_iB * JwB * JwB;

		ci::Vec2f pD = m_localAnchorD - m_lcD;
		ci::Vec2f pB = cb2MulT(qD, rB + (cB - cD));
		coordinateB = cb2Dot(pB - pD, m_localAxisD);
	}

	float C = (coordinateA + m_ratio * coordinateB) - m_constant;

	float impulse = 0.0f;
	if (mass > 0.0f)
	{
		impulse = -C / mass;
	}

	cA += m_mA * impulse * JvAC;
	aA += m_iA * impulse * JwA;
	cB += m_mB * impulse * JvBD;
	aB += m_iB * impulse * JwB;
	cC -= m_mC * impulse * JvAC;
	aC -= m_iC * impulse * JwC;
	cD -= m_mD * impulse * JvBD;
	aD -= m_iD * impulse * JwD;

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;
	data.positions[m_indexC].c = cC;
	data.positions[m_indexC].a = aC;
	data.positions[m_indexD].c = cD;
	data.positions[m_indexD].a = aD;

	// TODO_ERIN not implemented
	return linearError < cb2_linearSlop;
}

ci::Vec2f cb2GearJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

ci::Vec2f cb2GearJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

ci::Vec2f cb2GearJoint::GetReactionForce(float inv_dt) const
{
	ci::Vec2f P = m_impulse * m_JvAC;
	return inv_dt * P;
}

float cb2GearJoint::GetReactionTorque(float inv_dt) const
{
	float L = m_impulse * m_JwA;
	return inv_dt * L;
}

void cb2GearJoint::SetRatio(float ratio)
{
	cb2Assert(cb2::isValid(ratio));
	m_ratio = ratio;
}

float cb2GearJoint::GetRatio() const
{
	return m_ratio;
}

void cb2GearJoint::Dump()
{
	int indexA = m_bodyA->m_islandIndex;
	int indexB = m_bodyB->m_islandIndex;

	int index1 = m_joint1->m_index;
	int index2 = m_joint2->m_index;

	cb2Log("  cb2GearJointDef jd;\n");
	cb2Log("  jd.bodyA = bodies[%d];\n", indexA);
	cb2Log("  jd.bodyB = bodies[%d];\n", indexB);
	cb2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	cb2Log("  jd.joint1 = joints[%d];\n", index1);
	cb2Log("  jd.joint2 = joints[%d];\n", index2);
	cb2Log("  jd.ratio = %.15lef;\n", m_ratio);
	cb2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
