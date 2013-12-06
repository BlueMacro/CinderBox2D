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

#ifndef B2_WHEEL_JOINT_H
#define B2_WHEEL_JOINT_H

#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>

/// Wheel joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct b2WheelJointDef : public b2JointDef
{
	b2WheelJointDef()
	{
		type = e_wheelJoint;
		cb2::setZero(localAnchorA);
		cb2::setZero(localAnchorB);
		localAxisA.set(1.0f, 0.0f);
		enableMotor = false;
		maxMotorTorque = 0.0f;
		motorSpeed = 0.0f;
		frequencyHz = 2.0f;
		dampingRatio = 0.7f;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and world axis.
	void Initialize(b2Body* bodyA, b2Body* bodyB, const ci::Vec2f& anchor, const ci::Vec2f& axis);

	/// The local anchor point relative to bodyA's origin.
	ci::Vec2f localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	ci::Vec2f localAnchorB;

	/// The local translation axis in bodyA.
	ci::Vec2f localAxisA;

	/// Enable/disable the joint motor.
	bool enableMotor;

	/// The maximum motor torque, usually in N-m.
	float maxMotorTorque;

	/// The desired motor speed in radians per second.
	float motorSpeed;

	/// Suspension frequency, zero indicates no suspension
	float frequencyHz;

	/// Suspension damping ratio, one indicates critical damping
	float dampingRatio;
};

/// A wheel joint. This joint provides two degrees of freedom: translation
/// along an axis fixed in bodyA and rotation in the plane. You can use a
/// joint limit to restrict the range of motion and a joint motor to drive
/// the rotation or to model rotational friction.
/// This joint is designed for vehicle suspensions.
class b2WheelJoint : public b2Joint
{
public:
	ci::Vec2f GetAnchorA() const;
	ci::Vec2f GetAnchorB() const;

	ci::Vec2f GetReactionForce(float inv_dt) const;
	float GetReactionTorque(float inv_dt) const;

	/// The local anchor point relative to bodyA's origin.
	const ci::Vec2f& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const ci::Vec2f& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// The local joint axis relative to bodyA.
	const ci::Vec2f& GetLocalAxisA() const { return m_localXAxisA; }

	/// Get the current joint translation, usually in meters.
	float GetJointTranslation() const;

	/// Get the current joint translation speed, usually in meters per second.
	float GetJointSpeed() const;

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// set the motor speed, usually in radians per second.
	void SetMotorSpeed(float speed);

	/// Get the motor speed, usually in radians per second.
	float GetMotorSpeed() const;

	/// set/Get the maximum motor force, usually in N-m.
	void SetMaxMotorTorque(float torque);
	float GetMaxMotorTorque() const;

	/// Get the current motor torque given the inverse time step, usually in N-m.
	float GetMotorTorque(float inv_dt) const;

	/// set/Get the spring frequency in hertz. Setting the frequency to zero disables the spring.
	void SetSpringFrequencyHz(float hz);
	float GetSpringFrequencyHz() const;

	/// set/Get the spring damping ratio
	void SetSpringDampingRatio(float ratio);
	float GetSpringDampingRatio() const;

	/// Dump to b2Log
	void Dump();

protected:

	friend class b2Joint;
	b2WheelJoint(const b2WheelJointDef* def);

	void InitVelocityConstraints(const b2SolverData& data);
	void SolveVelocityConstraints(const b2SolverData& data);
	bool SolvePositionConstraints(const b2SolverData& data);

	float m_frequencyHz;
	float m_dampingRatio;

	// Solver shared
	ci::Vec2f m_localAnchorA;
	ci::Vec2f m_localAnchorB;
	ci::Vec2f m_localXAxisA;
	ci::Vec2f m_localYAxisA;

	float m_impulse;
	float m_motorImpulse;
	float m_springImpulse;

	float m_maxMotorTorque;
	float m_motorSpeed;
	bool m_enableMotor;

	// Solver temp
	int m_indexA;
	int m_indexB;
	ci::Vec2f m_localCenterA;
	ci::Vec2f m_localCenterB;
	float m_invMassA;
	float m_invMassB;
	float m_invIA;
	float m_invIB;

	ci::Vec2f m_ax, m_ay;
	float m_sAx, m_sBx;
	float m_sAy, m_sBy;

	float m_mass;
	float m_motorMass;
	float m_springMass;

	float m_bias;
	float m_gamma;
};

inline float b2WheelJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

inline float b2WheelJoint::GetMaxMotorTorque() const
{
	return m_maxMotorTorque;
}

inline void b2WheelJoint::SetSpringFrequencyHz(float hz)
{
	m_frequencyHz = hz;
}

inline float b2WheelJoint::GetSpringFrequencyHz() const
{
	return m_frequencyHz;
}

inline void b2WheelJoint::SetSpringDampingRatio(float ratio)
{
	m_dampingRatio = ratio;
}

inline float b2WheelJoint::GetSpringDampingRatio() const
{
	return m_dampingRatio;
}

#endif
