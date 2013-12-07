/*
* Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Dynamics/Joints/cb2MouseJoint.h>
#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Dynamics/cb2TimeStep.h>

// p = attached point, m = mouse point
// C = p - m
// Cdot = v
//      = v + cross(w, r)
// J = [I r_skew]
// Identity used:
// w k % (rx i + ry j) = w * (-ry i + rx j)

cb2MouseJoint::cb2MouseJoint(const cb2MouseJointDef* def)
: cb2Joint(def)
{
	cb2Assert(cb2::isValid(def->target));
	cb2Assert(cb2::isValid(def->maxForce) && def->maxForce >= 0.0f);
	cb2Assert(cb2::isValid(def->frequencyHz) && def->frequencyHz >= 0.0f);
	cb2Assert(cb2::isValid(def->dampingRatio) && def->dampingRatio >= 0.0f);

	m_targetA = def->target;
	m_localAnchorB = cb2MulT(m_bodyB->GetTransform(), m_targetA);

	m_maxForce = def->maxForce;
	cb2::setZero(m_impulse);

	m_frequencyHz = def->frequencyHz;
	m_dampingRatio = def->dampingRatio;

	m_beta = 0.0f;
	m_gamma = 0.0f;
}

void cb2MouseJoint::SetTarget(const ci::Vec2f& target)
{
	if (m_bodyB->IsAwake() == false)
	{
		m_bodyB->SetAwake(true);
	}
	m_targetA = target;
}

const ci::Vec2f& cb2MouseJoint::GetTarget() const
{
	return m_targetA;
}

void cb2MouseJoint::SetMaxForce(float force)
{
	m_maxForce = force;
}

float cb2MouseJoint::GetMaxForce() const
{
	return m_maxForce;
}

void cb2MouseJoint::SetFrequency(float hz)
{
	m_frequencyHz = hz;
}

float cb2MouseJoint::GetFrequency() const
{
	return m_frequencyHz;
}

void cb2MouseJoint::SetDampingRatio(float ratio)
{
	m_dampingRatio = ratio;
}

float cb2MouseJoint::GetDampingRatio() const
{
	return m_dampingRatio;
}

void cb2MouseJoint::InitVelocityConstraints(const cb2SolverData& data)
{
	m_indexB = m_bodyB->m_islandIndex;
	m_localCenterB = m_bodyB->m_sweep.localCenter;
	m_invMassB = m_bodyB->m_invMass;
	m_invIB = m_bodyB->m_invI;

	ci::Vec2f cB = data.positions[m_indexB].c;
	float aB = data.positions[m_indexB].a;
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	cb2Rot qB(aB);

	float mass = m_bodyB->GetMass();

	// Frequency
	float omega = 2.0f * cb2_pi * m_frequencyHz;

	// Damping coefficient
	float d = 2.0f * mass * m_dampingRatio * omega;

	// Spring stiffness
	float k = mass * (omega * omega);

	// magic formulas
	// gamma has units of inverse mass.
	// beta has units of inverse time.
	float h = data.step.dt;
	cb2Assert(d + h * k > cb2_epsilon);
	m_gamma = h * (d + h * k);
	if (m_gamma != 0.0f)
	{
		m_gamma = 1.0f / m_gamma;
	}
	m_beta = h * k * m_gamma;

	// Compute the effective mass matrix.
	m_rB = cb2Mul(qB, m_localAnchorB - m_localCenterB);

	// K    = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
	//      = [1/m1+1/m2     0    ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
	//        [    0     1/m1+1/m2]           [-r1.x*r1.y r1.x*r1.x]           [-r1.x*r1.y r1.x*r1.x]
	ci::Matrix22f K;
	K.m00 = m_invMassB + m_invIB * m_rB.y * m_rB.y + m_gamma;
	K.m01 = -m_invIB * m_rB.x * m_rB.y;
	K.m10 = K.m01;
	K.m11 = m_invMassB + m_invIB * m_rB.x * m_rB.x + m_gamma;

	m_mass = K.inverted();

	m_C = cB + m_rB - m_targetA;
	m_C *= m_beta;

	// Cheat with some damping
	wB *= 0.98f;

	if (data.step.warmStarting)
	{
		m_impulse *= data.step.dtRatio;
		vB += m_invMassB * m_impulse;
		wB += m_invIB * cb2Cross(m_rB, m_impulse);
	}
	else
	{
		cb2::setZero(m_impulse);
	}

	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

void cb2MouseJoint::SolveVelocityConstraints(const cb2SolverData& data)
{
	ci::Vec2f vB = data.velocities[m_indexB].v;
	float wB = data.velocities[m_indexB].w;

	// Cdot = v + cross(w, r)
	ci::Vec2f Cdot = vB + cb2Cross(wB, m_rB);
	ci::Vec2f impulse = cb2Mul(m_mass, -(Cdot + m_C + m_gamma * m_impulse));

	ci::Vec2f oldImpulse = m_impulse;
	m_impulse += impulse;
	float maxImpulse = data.step.dt * m_maxForce;
	if (m_impulse.lengthSquared() > maxImpulse * maxImpulse)
	{
		m_impulse *= maxImpulse / m_impulse.length();
	}
	impulse = m_impulse - oldImpulse;

	vB += m_invMassB * impulse;
	wB += m_invIB * cb2Cross(m_rB, impulse);

	data.velocities[m_indexB].v = vB;
	data.velocities[m_indexB].w = wB;
}

bool cb2MouseJoint::SolvePositionConstraints(const cb2SolverData& data)
{
	CB2_NOT_USED(data);
	return true;
}

ci::Vec2f cb2MouseJoint::GetAnchorA() const
{
	return m_targetA;
}

ci::Vec2f cb2MouseJoint::GetAnchorB() const
{
	return m_bodyB->GetWorldPoint(m_localAnchorB);
}

ci::Vec2f cb2MouseJoint::GetReactionForce(float inv_dt) const
{
	return inv_dt * m_impulse;
}

float cb2MouseJoint::GetReactionTorque(float inv_dt) const
{
	return inv_dt * 0.0f;
}

void cb2MouseJoint::ShiftOrigin(const ci::Vec2f& newOrigin)
{
	m_targetA -= newOrigin;
}
