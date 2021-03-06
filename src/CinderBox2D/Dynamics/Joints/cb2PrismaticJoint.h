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

#ifndef CB2_PRISMATIC_JOINT_H
#define CB2_PRISMATIC_JOINT_H

#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>

/// Prismatic joint definition. This requires defining a line of
/// motion using an axis and an anchor point. The definition uses local
/// anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero
/// when the local anchor points coincide in world space. Using local
/// anchors and a local axis helps when saving and loading a game.
struct cb2PrismaticJointDef : public cb2JointDef
{
	cb2PrismaticJointDef()
	{
		type = e_prismaticJoint;
		cb2::setZero(localAnchorA);
		cb2::setZero(localAnchorB);
		localAxisA.set(1.0f, 0.0f);
		referenceAngle = 0.0f;
		enableLimit = false;
		lowerTranslation = 0.0f;
		upperTranslation = 0.0f;
		enableMotor = false;
		maxMotorForce = 0.0f;
		motorSpeed = 0.0f;
	}

	/// Initialize the bodies, anchors, axis, and reference angle using the world
	/// anchor and unit world axis.
	void Initialize(cb2Body* bodyA, cb2Body* bodyB, const ci::Vec2f& anchor, const ci::Vec2f& axis);

	/// The local anchor point relative to bodyA's origin.
	ci::Vec2f localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	ci::Vec2f localAnchorB;

	/// The local translation unit axis in bodyA.
	ci::Vec2f localAxisA;

	/// The constrained angle between the bodies: bodyB_angle - bodyA_angle.
	float referenceAngle;

	/// Enable/disable the joint limit.
	bool enableLimit;

	/// The lower translation limit, usually in meters.
	float lowerTranslation;

	/// The upper translation limit, usually in meters.
	float upperTranslation;

	/// Enable/disable the joint motor.
	bool enableMotor;

	/// The maximum motor torque, usually in N-m.
	float maxMotorForce;

	/// The desired motor speed in radians per second.
	float motorSpeed;
};

/// A prismatic joint. This joint provides one degree of freedom: translation
/// along an axis fixed in bodyA. Relative rotation is prevented. You can
/// use a joint limit to restrict the range of motion and a joint motor to
/// drive the motion or to model joint friction.
class cb2PrismaticJoint : public cb2Joint
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

	/// Get the reference angle.
	float GetReferenceAngle() const { return m_referenceAngle; }

	/// Get the current joint translation, usually in meters.
	float GetJointTranslation() const;

	/// Get the current joint translation speed, usually in meters per second.
	float GetJointSpeed() const;

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	/// Enable/disable the joint limit.
	void EnableLimit(bool flag);

	/// Get the lower joint limit, usually in meters.
	float GetLowerLimit() const;

	/// Get the upper joint limit, usually in meters.
	float GetUpperLimit() const;

	/// set the joint limits, usually in meters.
	void SetLimits(float lower, float upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// set the motor speed, usually in meters per second.
	void SetMotorSpeed(float speed);

	/// Get the motor speed, usually in meters per second.
	float GetMotorSpeed() const;

	/// set the maximum motor force, usually in N.
	void SetMaxMotorForce(float force);
	float GetMaxMotorForce() const { return m_maxMotorForce; }

	/// Get the current motor force given the inverse time step, usually in N.
	float GetMotorForce(float inv_dt) const;

	/// Dump to cb2Log
	void Dump();

protected:
	friend class cb2Joint;
	friend class cb2GearJoint;
	cb2PrismaticJoint(const cb2PrismaticJointDef* def);

	void InitVelocityConstraints(const cb2SolverData& data);
	void SolveVelocityConstraints(const cb2SolverData& data);
	bool SolvePositionConstraints(const cb2SolverData& data);

	// Solver shared
	ci::Vec2f m_localAnchorA;
	ci::Vec2f m_localAnchorB;
	ci::Vec2f m_localXAxisA;
	ci::Vec2f m_localYAxisA;
	float m_referenceAngle;
	ci::Vec3f m_impulse;
	float m_motorImpulse;
	float m_lowerTranslation;
	float m_upperTranslation;
	float m_maxMotorForce;
	float m_motorSpeed;
	bool m_enableLimit;
	bool m_enableMotor;
	cb2LimitState m_limitState;

	// Solver temp
	int m_indexA;
	int m_indexB;
	ci::Vec2f m_localCenterA;
	ci::Vec2f m_localCenterB;
	float m_invMassA;
	float m_invMassB;
	float m_invIA;
	float m_invIB;
	ci::Vec2f m_axis, m_perp;
	float m_s1, m_s2;
	float m_a1, m_a2;
	ci::Matrix33f m_K;
	float m_motorMass;
};

inline float cb2PrismaticJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

#endif
