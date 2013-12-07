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

#include <CinderBox2D/Dynamics/Joints/cb2PrismaticJoint.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>

// Linear constraint (point-to-line)
// d = p2 - p1 = x2 + r2 - x1 - r1
// C = dot(perp, d)
// Cdot = dot(d, cross(w1, perp)) + dot(perp, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//      = -dot(perp, v1) - dot(cross(d + r1, perp), w1) + dot(perp, v2) + dot(cross(r2, perp), v2)
// J = [-perp, -cross(d + r1, perp), perp, cross(r2,perp)]
//
// Angular constraint
// C = a2 - a1 + a_initial
// Cdot = w2 - w1
// J = [0 0 -1 0 0 1]
//
// K = J * invM * JT
//
// J = [-a -s1 a s2]
//     [0  -1  0  1]
// a = perp
// s1 = cross(d + r1, a) = cross(p2 - x1, a)
// s2 = cross(r2, a) = cross(p2 - x2, a)


// Motor/Limit linear constraint
// C = dot(ax1, d)
// Cdot = = -dot(ax1, v1) - dot(cross(d + r1, ax1), w1) + dot(ax1, v2) + dot(cross(r2, ax1), v2)
// J = [-ax1 -cross(d+r1,ax1) ax1 cross(r2,ax1)]

// Block Solver
// We develop a block solver that includes the joint limit. This makes the limit stiff (inelastic) even
// when the mass has poor distribution (leading to large torques about the joint anchor points).
//
// The Jacobian has 3 rows:
// J = [-uT -s1 uT s2] // linear
//     [0   -1   0  1] // angular
//     [-vT -a1 vT a2] // limit
//
// u = perp
// v = axis
// s1 = cross(d + r1, u), s2 = cross(r2, u)
// a1 = cross(d + r1, v), a2 = cross(r2, v)

// M * (v2 - v1) = JT * df
// J * v2 = bias
//
// v2 = v1 + invM * JT * df
// J * (v1 + invM * JT * df) = bias
// K * df = bias - J * v1 = -Cdot
// K = J * invM * JT
// Cdot = J * v1 - bias
//
// Now solve for f2.
// df = f2 - f1
// K * (f2 - f1) = -Cdot
// f2 = invK * (-Cdot) + f1
//
// Clamp accumulated limit impulse.
// lower: f2(3) = max(f2(3), 0)
// upper: f2(3) = min(f2(3), 0)
//
// Solve for correct f2(1:2)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:3) * f1
//                       = -Cdot(1:2) - K(1:2,3) * f2(3) + K(1:2,1:2) * f1(1:2) + K(1:2,3) * f1(3)
// K(1:2, 1:2) * f2(1:2) = -Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3)) + K(1:2,1:2) * f1(1:2)
// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
//
// Now compute impulse to be applied:
// df = f2 - f1

void cb2PrismaticJointDef::Initialize(cb2Body* bA, cb2Body* bB, const ci::Vec2f& anchor, const ci::Vec2f& axis)
{
	bodyA = bA;
	bodyB = bB;
	localAnchorA = bodyA->GetLocalPoint(anchor);
	localAnchorB = bodyB->GetLocalPoint(anchor);
	localAxisA = bodyA->GetLocalVector(axis);
	referenceAngle = bodyB->GetAngle() - bodyA->GetAngle();
}

cb2PrismaticJoint::cb2PrismaticJoint(const cb2PrismaticJointDef* def)
: cb2Joint(def)
{
	m_localAnchorA = def->localAnchorA;
	m_localAnchorB = def->localAnchorB;
	m_localXAxisA = def->localAxisA;
	m_localXAxisA.normalize();
	m_localYAxisA = cb2Cross(1.0f, m_localXAxisA);
	m_referenceAngle = def->referenceAngle;

	m_motorMass = 0.0;
	m_motorImpulse = 0.0f;

	m_lowerTranslation = def->lowerTranslation;
	m_upperTranslation = def->upperTranslation;
	m_maxMotorForce = def->maxMotorForce;
	m_motorSpeed = def->motorSpeed;
	m_enableLimit = def->enableLimit;
	m_enableMotor = def->enableMotor;
	m_limitState = e_inactiveLimit;
}

void cb2PrismaticJoint::InitVelocityConstraints(const cb2SolverData& data)
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

	// Compute the effective masses.
	ci::Vec2f rA = cb2Mul(qA, m_localAnchorA - m_localCenterA);
	ci::Vec2f rB = cb2Mul(qB, m_localAnchorB - m_localCenterB);
	ci::Vec2f d = (cB - cA) + rB - rA;

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	// Compute motor Jacobian and effective mass.
	{
		m_axis = cb2Mul(qA, m_localXAxisA);
		m_a1 = cb2Cross(d + rA, m_axis);
		m_a2 = cb2Cross(rB, m_axis);

		m_motorMass = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;
		if (m_motorMass > 0.0f)
		{
			m_motorMass = 1.0f / m_motorMass;
		}
	}

	// Prismatic constraint.
	{
		m_perp = cb2Mul(qA, m_localYAxisA);

		m_s1 = cb2Cross(d + rA, m_perp);
		m_s2 = cb2Cross(rB, m_perp);

		float k11 = mA + mB + iA * m_s1 * m_s1 + iB * m_s2 * m_s2;
		float k12 = iA * m_s1 + iB * m_s2;
		float k13 = iA * m_s1 * m_a1 + iB * m_s2 * m_a2;
		float k22 = iA + iB;
		if (k22 == 0.0f)
		{
			// For bodies with fixed rotation.
			k22 = 1.0f;
		}
		float k23 = iA * m_a1 + iB * m_a2;
		float k33 = mA + mB + iA * m_a1 * m_a1 + iB * m_a2 * m_a2;

    m_K.set(k11, k12, k13,
		        k12, k22, k23,
		        k13, k23, k33);
	}

	// Compute motor and limit terms.
	if (m_enableLimit)
	{
		float jointTranslation = cb2Dot(m_axis, d);
		if (cb2Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * cb2_linearSlop)
		{
			m_limitState = e_equalLimits;
		}
		else if (jointTranslation <= m_lowerTranslation)
		{
			if (m_limitState != e_atLowerLimit)
			{
				m_limitState = e_atLowerLimit;
				m_impulse.z = 0.0f;
			}
		}
		else if (jointTranslation >= m_upperTranslation)
		{
			if (m_limitState != e_atUpperLimit)
			{
				m_limitState = e_atUpperLimit;
				m_impulse.z = 0.0f;
			}
		}
		else
		{
			m_limitState = e_inactiveLimit;
			m_impulse.z = 0.0f;
		}
	}
	else
	{
		m_limitState = e_inactiveLimit;
		m_impulse.z = 0.0f;
	}

	if (m_enableMotor == false)
	{
		m_motorImpulse = 0.0f;
	}

	if (data.step.warmStarting)
	{
		// Account for variable time step.
		m_impulse *= data.step.dtRatio;
		m_motorImpulse *= data.step.dtRatio;

		ci::Vec2f P = m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis;
		float LA = m_impulse.x * m_s1 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a1;
		float LB = m_impulse.x * m_s2 + m_impulse.y + (m_motorImpulse + m_impulse.z) * m_a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}
	else
	{
		cb2::setZero(m_impulse);
		m_motorImpulse = 0.0f;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void cb2PrismaticJoint::SolveVelocityConstraints(const cb2SolverData& data)
{
	ci::Vec2f vA = data.velocities[m_indexA].v;
	float wA = data.velocities[m_indexA].w;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	// Solve linear motor constraint.
	if (m_enableMotor && m_limitState != e_equalLimits)
	{
		float Cdot = cb2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
		float impulse = m_motorMass * (m_motorSpeed - Cdot);
		float oldImpulse = m_motorImpulse;
		float maxImpulse = data.step.dt * m_maxMotorForce;
		m_motorImpulse = cb2Clamp(m_motorImpulse + impulse, -maxImpulse, maxImpulse);
		impulse = m_motorImpulse - oldImpulse;

		ci::Vec2f P = impulse * m_axis;
		float LA = impulse * m_a1;
		float LB = impulse * m_a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	ci::Vec2f Cdot1;
	Cdot1.x = cb2Dot(m_perp, vB - vA) + m_s2 * wB - m_s1 * wA;
	Cdot1.y = wB - wA;

	if (m_enableLimit && m_limitState != e_inactiveLimit)
	{
		// Solve prismatic and limit constraint in block form.
		float Cdot2;
		Cdot2 = cb2Dot(m_axis, vB - vA) + m_a2 * wB - m_a1 * wA;
		ci::Vec3f Cdot(Cdot1.x, Cdot1.y, Cdot2);

		ci::Vec3f f1 = m_impulse;
		ci::Vec3f df = cb2::solve(m_K, -Cdot);
		m_impulse += df;

		if (m_limitState == e_atLowerLimit)
		{
			m_impulse.z = cb2Max(m_impulse.z, 0.0f);
		}
		else if (m_limitState == e_atUpperLimit)
		{
			m_impulse.z = cb2Min(m_impulse.z, 0.0f);
		}

		// f2(1:2) = invK(1:2,1:2) * (-Cdot(1:2) - K(1:2,3) * (f2(3) - f1(3))) + f1(1:2)
		ci::Vec2f b = -Cdot1 - (m_impulse.z - f1.z) * ci::Vec2f(m_K.m20, m_K.m21);
		ci::Vec2f f2r = cb2::solve22(m_K, b) + ci::Vec2f(f1.x, f1.y);
		m_impulse.x = f2r.x;
		m_impulse.y = f2r.y;

		df = m_impulse - f1;

		ci::Vec2f P = df.x * m_perp + df.z * m_axis;
		float LA = df.x * m_s1 + df.y + df.z * m_a1;
		float LB = df.x * m_s2 + df.y + df.z * m_a2;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}
	else
	{
		// Limit is inactive, just solve the prismatic constraint in block form.
		ci::Vec2f df = cb2::solve22(m_K, -Cdot1);
		m_impulse.x += df.x;
		m_impulse.y += df.y;

		ci::Vec2f P = df.x * m_perp;
		float LA = df.x * m_s1 + df.y;
		float LB = df.x * m_s2 + df.y;

		vA -= mA * P;
		wA -= iA * LA;

		vB += mB * P;
		wB += iB * LB;
	}

	data.velocities[m_indexA].v = vA;
	data.velocities[m_indexA].w = wA;
	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool cb2PrismaticJoint::SolvePositionConstraints(const cb2SolverData& data)
{
	ci::Vec2f cA = data.positions[m_indexA].c;
	float aA = data.positions[m_indexA].a;
	ci::Vec2f cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;

	cb2Rot qA(aA), qB(aB);

	float mA = m_invMassA, mB = m_invMassB;
	float iA = m_invIA, iB = m_invIB;

	// Compute fresh Jacobians
	ci::Vec2f rA = cb2Mul(qA, m_localAnchorA - m_localCenterA);
	ci::Vec2f rB = cb2Mul(qB, m_localAnchorB - m_localCenterB);
	ci::Vec2f d = cB + rB - cA - rA;

	ci::Vec2f axis = cb2Mul(qA, m_localXAxisA);
	float a1 = cb2Cross(d + rA, axis);
	float a2 = cb2Cross(rB, axis);
	ci::Vec2f perp = cb2Mul(qA, m_localYAxisA);

	float s1 = cb2Cross(d + rA, perp);
	float s2 = cb2Cross(rB, perp);

	ci::Vec3f impulse;
	ci::Vec2f C1;
	C1.x = cb2Dot(perp, d);
	C1.y = aB - aA - m_referenceAngle;

	float linearError = cb2Abs(C1.x);
	float angularError = cb2Abs(C1.y);

	bool active = false;
	float C2 = 0.0f;
	if (m_enableLimit)
	{
		float translation = cb2Dot(axis, d);
		if (cb2Abs(m_upperTranslation - m_lowerTranslation) < 2.0f * cb2_linearSlop)
		{
			// Prevent large angular corrections
			C2 = cb2Clamp(translation, -cb2_maxLinearCorrection, cb2_maxLinearCorrection);
			linearError = cb2Max(linearError, cb2Abs(translation));
			active = true;
		}
		else if (translation <= m_lowerTranslation)
		{
			// Prevent large linear corrections and allow some slop.
			C2 = cb2Clamp(translation - m_lowerTranslation + cb2_linearSlop, -cb2_maxLinearCorrection, 0.0f);
			linearError = cb2Max(linearError, m_lowerTranslation - translation);
			active = true;
		}
		else if (translation >= m_upperTranslation)
		{
			// Prevent large linear corrections and allow some slop.
			C2 = cb2Clamp(translation - m_upperTranslation - cb2_linearSlop, 0.0f, cb2_maxLinearCorrection);
			linearError = cb2Max(linearError, translation - m_upperTranslation);
			active = true;
		}
	}

	if (active)
	{
		float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float k12 = iA * s1 + iB * s2;
		float k13 = iA * s1 * a1 + iB * s2 * a2;
		float k22 = iA + iB;
		if (k22 == 0.0f)
		{
			// For fixed rotation
			k22 = 1.0f;
		}
		float k23 = iA * a1 + iB * a2;
		float k33 = mA + mB + iA * a1 * a1 + iB * a2 * a2;

		ci::Matrix33f K(k11, k12, k13,
		                k12, k22, k23,
		                k13, k23, k33);

		ci::Vec3f C;
		C.x = C1.x;
		C.y = C1.y;
		C.z = C2;

		impulse = cb2::solve(K, -C);
	}
	else
	{
		float k11 = mA + mB + iA * s1 * s1 + iB * s2 * s2;
		float k12 = iA * s1 + iB * s2;
		float k22 = iA + iB;
		if (k22 == 0.0f)
		{
			k22 = 1.0f;
		}

		ci::Matrix22f K(k11, k12,
		                k12, k22);

		ci::Vec2f impulse1 = cb2::solve( K, -C1 );
		impulse.x = impulse1.x;
		impulse.y = impulse1.y;
		impulse.z = 0.0f;
	}

	ci::Vec2f P = impulse.x * perp + impulse.z * axis;
	float LA = impulse.x * s1 + impulse.y + impulse.z * a1;
	float LB = impulse.x * s2 + impulse.y + impulse.z * a2;

	cA -= mA * P;
	aA -= iA * LA;
	cB += mB * P;
	aB += iB * LB;

	data.positions[m_indexA].c = cA;
	data.positions[m_indexA].a = aA;
	data.positions[m_indexB].c = cB;
	data.positions[m_indexB].a = aB;

	return linearError <= cb2_linearSlop && angularError <= cb2_angularSlop;
}

ci::Vec2f cb2PrismaticJoint::GetAnchorA() const
{
	return m_bodyA->GetWorldPoint(m_localAnchorA);
}

ci::Vec2f cb2PrismaticJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

ci::Vec2f cb2PrismaticJoint::GetReactionForce(float inv_dt) const
{
	return inv_dt * (m_impulse.x * m_perp + (m_motorImpulse + m_impulse.z) * m_axis);
}

float cb2PrismaticJoint::GetReactionTorque(float inv_dt) const
{
	return inv_dt * m_impulse.y;
}

float cb2PrismaticJoint::GetJointTranslation() const
{
	ci::Vec2f pA = m_bodyA->GetWorldPoint(m_localAnchorA);
	ci::Vec2f pB = m_bodyB->GetWorldPoint(m_localAnchorB);
	ci::Vec2f d = pB - pA;
	ci::Vec2f axis = m_bodyA->GetWorldVector(m_localXAxisA);

	float translation = cb2Dot(d, axis);
	return translation;
}

float cb2PrismaticJoint::GetJointSpeed() const
{
	cb2Body* bA = m_bodyA;
	cb2Body* bB = m_bodyB;

	ci::Vec2f rA = cb2Mul(bA->m_xf.q, m_localAnchorA - bA->m_sweep.localCenter);
	ci::Vec2f rB = cb2Mul(bB->m_xf.q, m_localAnchorB - bB->m_sweep.localCenter);
	ci::Vec2f p1 = bA->m_sweep.c + rA;
	ci::Vec2f p2 = bB->m_sweep.c + rB;
	ci::Vec2f d = p2 - p1;
	ci::Vec2f axis = cb2Mul(bA->m_xf.q, m_localXAxisA);

	ci::Vec2f vA = bA->m_linearVelocity;
	ci::Vec2f vB = bB->m_linearVelocity;
	float wA = bA->m_angularVelocity;
	float wB = bB->m_angularVelocity;

	float speed = cb2Dot(d, cb2Cross(wA, axis)) + cb2Dot(axis, vB + cb2Cross(wB, rB) - vA - cb2Cross(wA, rA));
	return speed;
}

bool cb2PrismaticJoint::IsLimitEnabled() const
{
	return m_enableLimit;
}

void cb2PrismaticJoint::EnableLimit(bool flag)
{
	if (flag != m_enableLimit)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_enableLimit = flag;
		m_impulse.z = 0.0f;
	}
}

float cb2PrismaticJoint::GetLowerLimit() const
{
	return m_lowerTranslation;
}

float cb2PrismaticJoint::GetUpperLimit() const
{
	return m_upperTranslation;
}

void cb2PrismaticJoint::SetLimits(float lower, float upper)
{
	cb2Assert(lower <= upper);
	if (lower != m_lowerTranslation || upper != m_upperTranslation)
	{
		m_bodyA->SetAwake(true);
		m_bodyB->SetAwake(true);
		m_lowerTranslation = lower;
		m_upperTranslation = upper;
		m_impulse.z = 0.0f;
	}
}

bool cb2PrismaticJoint::IsMotorEnabled() const
{
	return m_enableMotor;
}

void cb2PrismaticJoint::EnableMotor(bool flag)
{
	m_bodyA->SetAwake(true);
	m_bodyB->SetAwake(true);
	m_enableMotor = flag;
}

void cb2PrismaticJoint::SetMotorSpeed(float speed)
{
	m_bodyA->SetAwake(true);
	m_bodyB->SetAwake(true);
	m_motorSpeed = speed;
}

void cb2PrismaticJoint::SetMaxMotorForce(float force)
{
	m_bodyA->SetAwake(true);
	m_bodyB->SetAwake(true);
	m_maxMotorForce = force;
}

float cb2PrismaticJoint::GetMotorForce(float inv_dt) const
{
	return inv_dt * m_motorImpulse;
}

void cb2PrismaticJoint::Dump()
{
	int indexA = m_bodyA->m_islandIndex;
	int indexB = m_bodyB->m_islandIndex;

	cb2Log("  cb2PrismaticJointDef jd;\n");
	cb2Log("  jd.bodyA = bodies[%d];\n", indexA);
	cb2Log("  jd.bodyB = bodies[%d];\n", indexB);
	cb2Log("  jd.collideConnected = bool(%d);\n", m_collideConnected);
	cb2Log("  jd.localAnchorA.set(%.15lef, %.15lef);\n", m_localAnchorA.x, m_localAnchorA.y);
	cb2Log("  jd.localAnchorB.set(%.15lef, %.15lef);\n", m_localAnchorB.x, m_localAnchorB.y);
	cb2Log("  jd.localAxisA.set(%.15lef, %.15lef);\n", m_localXAxisA.x, m_localXAxisA.y);
	cb2Log("  jd.referenceAngle = %.15lef;\n", m_referenceAngle);
	cb2Log("  jd.enableLimit = bool(%d);\n", m_enableLimit);
	cb2Log("  jd.lowerTranslation = %.15lef;\n", m_lowerTranslation);
	cb2Log("  jd.upperTranslation = %.15lef;\n", m_upperTranslation);
	cb2Log("  jd.enableMotor = bool(%d);\n", m_enableMotor);
	cb2Log("  jd.motorSpeed = %.15lef;\n", m_motorSpeed);
	cb2Log("  jd.maxMotorForce = %.15lef;\n", m_maxMotorForce);
	cb2Log("  joints[%d] = m_world->CreateJoint(&jd);\n", m_index);
}
