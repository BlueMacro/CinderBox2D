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

#include <CinderBox2D/Dynamics/Joints/cb2WeldJoint.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>

// Point-to-point constraint
// C = p2 - p1
// Cdot = v2 - v1
//      = v2 + cross(w2, r2) - v1 - cross(w1, r1)
// J = [-I -r1_skew I r2_skew ]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

// Angle constraint
// C = angle2 - angle1 - referenceAngle
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
// K = invI1 + invI2

void cb2WeldJointDef::Initialize(cb2Body* bA, cb2Body* bB, const ci::Vec2f& anchor)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

cb2WeldJoint::cb2WeldJoint(const cb2WeldJointDef* def)
: cb2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_referenceAngle = def->referenceAngle;
	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;

	cb2::setZero(m_impulse);
}

void cb2WeldJoint::InitVelocityConstraints(const cb2SolverData& data)
{
	m_indexA = m_bodyA->m_islandIndex;
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterA = m_bodyA->m_sweep.localCenter;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassA = m_bodyA->m_invMass;
	m_invMassB = m_bodyB->m_invMass;
	m_invIA = m_bodyA->m_invI;
	m_invIB = m_bodyB->m_invI;

	float aA = data.positions[m_indexA].a;
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;

	float aB = data.positions[m_indexB].a;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	cb2Rot qA(aA), qB(aB);

	m_rA = cb2Mul(qA, m_localAnchorA - m_localCenterA);
	m_rB = cb2Mul(qB, m_localAnchorB - m_localCenterB);

	// J = [-I -r1_skew I r2_skew]
	//     [ 0       -1 0       1]
	// r_skew = [-ry; rx]

	// Matlab
	// K = [ mA+r1y^2*iA+mB+r2y^2*iB,  -r1y*iA*r1x-r2y*iB*r2x,          -r1y*iA-r2y*iB]
	//     [  -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB,           r1x*iA+r2x*iB]
	//     [          -r1y*iA-r2y*iB,           r1x*iA+r2x*iB,                   iA+iB]

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	ci::Matrix33f K;
	K.m00 = mA + mB + m_rA.y * m_rA.y * iA + m_rB.y * m_rB.y * iB;
	K.m10 = -m_rA.y * m_rA.x * iA - m_rB.y * m_rB.x * iB;
	K.m20 = -m_rA.y * iA - m_rB.y * iB;
	K.m01 = K.m10;
	K.m11 = mA + mB + m_rA.x * m_rA.x * iA + m_rB.x * m_rB.x * iB;
	K.m21 = m_rA.x * iA + m_rB.x * iB;
	K.m02 = K.m20;
	K.m12 = K.m21;
	K.m22 = iA + iB;

	if (m_frequencyHz > 0.0f)
	{
		cb2::getInverse22(K, &m_mass);

		float invM = iA + iB;
		float m = invM > 0.0f ? 1.0f / invM : 0.0f;

		float C = aB - aA - m_referenceAngle;

		// Frequency
		float omega = 2.0f * cb2_pi * m_frequencyHz;

		// Damping coefficient
		float d = 2.0f * m * m_dampingRatio * omega;

		// Spring stiffness
		float k = m * omega * omega;

		// magic formulas
		float h = data.step.dt;
		m_gamma = h * (d + h * k);
		m_gamma = m_gamma != 0.0f ? 1.0f / m_gamma : 0.0f;
		m_bias = C * h * k * m_gamma;

		invM += m_gamma;
		m_mass.m22 = invM != 0.0f ? 1.0f / invM : 0.0f;
	}
	else
	{
		cb2::getSymInverse33(K, &m_mass);
		m_gamma = 0.0f;
		m_bias = 0.0f;
	}

	if (data.step.warmStarting)
	{
		// Scale impulses to support a variable time step.
		m_impulse *= data.step.dtRatio;

		ci::Vec2f P(m_impulse.x, m_impulse.y);

		vA -= mA * P;
		wA -= iA * (cb2Cross(m_rA, P) + m_impulse.z);

		vB += mB * P;
		wB += iB * (cb2Cross(m_rB, P) + m_impulse.z);
	}
	else
	{
		cb2::setZero(m_impulse);
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void cb2WeldJoint::SolveVelocityConstraints(const cb2SolverData& data)
{
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	if (m_frequencyHz > 0.0f)
	{
		float Cdot2 = wB - wA;

		float impulse2 = -m_mass.m22 * (Cdot2 + m_bias + m_gamma * m_impulse.z);
		m_impulse.z += impulse2;

		wA -= iA * impulse2;
		wB += iB * impulse2;

		ci::Vec2f Cdot1 = vB + cb2Cross(wB, m_rB) - vA - cb2Cross(wA, m_rA);

		ci::Vec2f impulse1 = -cb2Mul22(m_mass, Cdot1);
		m_impulse.x += impulse1.x;
		m_impulse.y += impulse1.y;

		ci::Vec2f P = impulse1;

		vA -= mA * P;
		wA -= iA * cb2Cross(m_rA, P);

		vB += mB * P;
		wB += iB * cb2Cross(m_rB, P);
	}
	else
	{
		ci::Vec2f Cdot1 = vB + cb2Cross(wB, m_rB) - vA - cb2Cross(wA, m_rA);
		float Cdot2 = wB - wA;
		ci::Vec3f Cdot(Cdot1.x, Cdot1.y, Cdot2);

		ci::Vec3f impulse = -cb2Mul(m_mass, Cdot);
		m_impulse += impulse;

		ci::Vec2f P(impulse.x, impulse.y);

		vA -= mA * P;
		wA -= iA * (cb2Cross(m_rA, P) + impulse.z);

		vB += mB * P;
		wB += iB * (cb2Cross(m_rB, P) + impulse.z);
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool cb2WeldJoint::SolvePositionConstraints(const cb2SolverData& data)
{
	ci::Vec2f cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	ci::Vec2f cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;

	cb2Rot qA(aA), qB(aB);

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	ci::Vec2f rA = cb2Mul(qA, m_localAnchorA - m_localCenterA);
	ci::Vec2f rB = cb2Mul(qB, m_localAnchorB - m_localCenterB);

	float positionError, angularError;

	ci::Matrix33f K;
	K.m00 = mA + mB + rA.y * rA.y * iA + rB.y * rB.y * iB;
	K.m10 = -rA.y * rA.x * iA - rB.y * rB.x * iB;
	K.m20 = -rA.y * iA - rB.y * iB;
	K.m01 = K.m10;
	K.m11 = mA + mB + rA.x * rA.x * iA + rB.x * rB.x * iB;
	K.m21 = rA.x * iA + rB.x * iB;
	K.m02 = K.m20;
	K.m12 = K.m21;
	K.m22 = iA + iB;

	if (m_frequencyHz > 0.0f)
	{
		ci::Vec2f C1 =  cB + rB - cA - rA;

		positionError = C1.length();
		angularError = 0.0f;

		ci::Vec2f P = -cb2::solve22(K, C1);

		cA -= mA * P;
		aA -= iA * cb2Cross(rA, P);

		cB += mB * P;
		aB += iB * cb2Cross(rB, P);
	}
	else
	{
		ci::Vec2f C1 =  cB + rB - cA - rA;
		float C2 = aB - aA - m_referenceAngle;

		positionError = C1.length();
		angularError = cb2Abs(C2);

		ci::Vec3f C(C1.x, C1.y, C2);
	
		ci::Vec3f impulse = -cb2::solve(K, C);
		ci::Vec2f P(impulse.x, impulse.y);

		cA -= mA * P;
		aA -= iA * (cb2Cross(rA, P) + impulse.z);

		cB += mB * P;
		aB += iB * (cb2Cross(rB, P) + impulse.z);
	}

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return positionError <= cb2_linearSlop && angularError <= cb2_angularSlop;
}

ci::Vec2f cb2WeldJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

ci::Vec2f cb2WeldJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

ci::Vec2f cb2WeldJoint::GetReactionForce(float inv_dt) const
{
	ci::Vec2f P(m_impulse.x, m_impulse.y);
	return inv_dt * P;
}

float cb2WeldJoint::GetReactionTorque(float inv_dt) const
{
	return inv_dt * m_impulse.z;
}

void cb2WeldJoint::Dump()
{
	int indexA = m_bodyA->m_islandIndex;
	int indexB = m_bodyB->m_islandIndex;

	cb2Log("  cb2WeldJointDef jd;\n");
	cb2Log("  jd.bodyA = bodies[%d];\n", indexA);
	cb2Log("  jd.bodyB = bodies[%d];\n", indexB);
	cb2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	cb2Log("  jd.localAnchorA.set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	cb2Log("  jd.localAnchorB.set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	cb2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
	cb2Log("  jd.frequencyHz = %.15lef;\n", m_frequencyHz);
	cb2Log("  jd.dampingRatio = %.15lef;\n", m_dampingRatio);
	cb2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
