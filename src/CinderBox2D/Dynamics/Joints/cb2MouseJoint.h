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

#ifndef B2_MOUSE_JOINT_H
#define B2_MOUSE_JOINT_H

#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>

/// Mouse joint definition. This requires a world target point,
/// tuning parameters, and the time step.
struct b2MouseJointDef : public b2JointDef
{
	b2MouseJointDef()
	{
		type = e_mouseJoint;
		target.set(0.0f, 0.0f);
		maxForce = 0.0f;
		frequencyHz = 5.0f;
		dampingRatio = 0.7f;
	}

	/// The initial world target point. This is assumed
	/// to coincide with the body anchor initially.
	ci::Vec2f target;

	/// The maximum constraint force that can be exerted
	/// to move the candidate body. Usually you will express
	/// as some multiple of the weight (multiplier * mass * gravity).
	float maxForce;

	/// The response speed.
	float frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float dampingRatio;
};

/// A mouse joint is used to make a point on a body track a
/// specified world point. This a soft constraint with a maximum
/// force. This allows the constraint to stretch and without
/// applying huge forces.
/// NOTE: this joint is not documented in the manual because it was
/// developed to be used in the testbed. If you want to learn how to
/// use the mouse joint, look at the testbed.
class b2MouseJoint : public b2Joint
{
public:

	/// Implements b2Joint.
	ci::Vec2f GetAnchorA() const;

	/// Implements b2Joint.
	ci::Vec2f GetAnchorB() const;

	/// Implements b2Joint.
	ci::Vec2f GetReactionForce(float inv_dt) const;

	/// Implements b2Joint.
	float GetReactionTorque(float inv_dt) const;

	/// Use this to update the target point.
	void SetTarget(const ci::Vec2f& target);
	const ci::Vec2f& GetTarget() const;

	/// set/get the maximum force in Newtons.
	void SetMaxForce(float force);
	float GetMaxForce() const;

	/// set/get the frequency in Hertz.
	void SetFrequency(float hz);
	float GetFrequency() const;

	/// set/get the damping ratio (dimensionless).
	void SetDampingRatio(float ratio);
	float GetDampingRatio() const;

	/// The mouse joint does not support dumping.
	void Dump() { b2Log("Mouse joint dumping is not supported.\n"); }

protected:
	friend class b2Joint;

	b2MouseJoint(const b2MouseJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);

	ci::Vec2f m_localAnchorB;
	ci::Vec2f m_targetA;
	float m_frequencyHz;
	float m_dampingRatio;
	float m_beta;
	
	// Solver shared
	ci::Vec2f m_impulse;
	float m_maxForce;
	float m_gamma;

	// Solver temp
	int m_indexA;
	int m_indexB;
	ci::Vec2f m_rB;
	ci::Vec2f m_localCenterB;
	float m_invMassB;
	float m_invIB;
	ci::Matrix22f m_mass;
	ci::Vec2f m_C;
};

#endif
