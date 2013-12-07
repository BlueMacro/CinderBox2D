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

#ifndef CB2_REVOLUTE_JOINT_H
#define CB2_REVOLUTE_JOINT_H

#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>

/// Revolute joint definition. This requires defining an
/// anchor point where the bodies are joined. The definition
/// uses local anchor points so that the initial configuration
/// can violate the constraint slightly. You also need to
/// specify the initial relative angle for joint limits. This
/// helps when saving and loading a game.
/// The local anchor points are measured from the body's origin
/// rather than the center of mass because:
/// 1. you might not know where the center of mass will be.
/// 2. if you add/remove shapes from a body and recompute the mass,
///    the joints will be broken.
struct cb2RevoluteJointDef : public cb2JointDef
{
	cb2RevoluteJointDef()
	{
		type = e_revoluteJoint;
		localAnchorA.set(0.0f, 0.0f);
		localAnchorB.set(0.0f, 0.0f);
		referenceAngle = 0.0f;
		lowerAngle = 0.0f;
		upperAngle = 0.0f;
		maxMotorTorque = 0.0f;
		motorSpeed = 0.0f;
		enableLimit = false;
		enableMotor = false;
	}

	/// Initialize the bodies, anchors, and reference angle using a world
	/// anchor point.
	void Initialize(cb2Body* bodyA, cb2Body* bodyB, const ci::Vec2f& anchor);

	/// The local anchor point relative to bodyA's origin.
	ci::Vec2f localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	ci::Vec2f localAnchorB;

	/// The bodyB angle minus bodyA angle in the reference state (radians).
	float referenceAngle;

	/// A flag to enable joint limits.
	bool enableLimit;

	/// The lower angle for the joint limit (radians).
	float lowerAngle;

	/// The upper angle for the joint limit (radians).
	float upperAngle;

	/// A flag to enable the joint motor.
	bool enableMotor;

	/// The desired motor speed. Usually in radians per second.
	float motorSpeed;

	/// The maximum motor torque used to achieve the desired motor speed.
	/// Usually in N-m.
	float maxMotorTorque;
};

/// A revolute joint constrains two bodies to share a common point while they
/// are free to rotate about the point. The relative rotation about the shared
/// point is the joint angle. You can limit the relative rotation with
/// a joint limit that specifies a lower and upper angle. You can use a motor
/// to drive the relative rotation about the shared point. A maximum motor torque
/// is provided so that infinite forces are not generated.
class cb2RevoluteJoint : public cb2Joint
{
public:
	ci::Vec2f GetAnchorA() const;
	ci::Vec2f GetAnchorB() const;

	/// The local anchor point relative to bodyA's origin.
	const ci::Vec2f& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const ci::Vec2f& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// Get the reference angle.
	float GetReferenceAngle() const { return m_referenceAngle; }

	/// Get the current joint angle in radians.
	float GetJointAngle() const;

	/// Get the current joint angle speed in radians per second.
	float GetJointSpeed() const;

	/// Is the joint limit enabled?
	bool IsLimitEnabled() const;

	/// Enable/disable the joint limit.
	void EnableLimit(bool flag);

	/// Get the lower joint limit in radians.
	float GetLowerLimit() const;

	/// Get the upper joint limit in radians.
	float GetUpperLimit() const;

	/// set the joint limits in radians.
	void SetLimits(float lower, float upper);

	/// Is the joint motor enabled?
	bool IsMotorEnabled() const;

	/// Enable/disable the joint motor.
	void EnableMotor(bool flag);

	/// set the motor speed in radians per second.
	void SetMotorSpeed(float speed);

	/// Get the motor speed in radians per second.
	float GetMotorSpeed() const;

	/// set the maximum motor torque, usually in N-m.
	void SetMaxMotorTorque(float torque);
	float GetMaxMotorTorque() const { return m_maxMotorTorque; }

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	ci::Vec2f GetReactionForce(float inv_dt) const;

	/// Get the reaction torque due to the joint limit given the inverse time step.
	/// Unit is N*m.
	float GetReactionTorque(float inv_dt) const;

	/// Get the current motor torque given the inverse time step.
	/// Unit is N*m.
	float GetMotorTorque(float inv_dt) const;

	/// Dump to cb2Log.
	void Dump();

protected:
	
	friend class cb2Joint;
	friend class cb2GearJoint;

	cb2RevoluteJoint(const cb2RevoluteJointDef* def);

	void InitVelocityConstraints(const cb2SolverData& data);
	void SolveVelocityConstraints(const cb2SolverData& data);
	bool SolvePositionConstraints(const cb2SolverData& data);

	// Solver shared
	ci::Vec2f m_localAnchorA;
	ci::Vec2f m_localAnchorB;
	ci::Vec3f m_impulse;
	float m_motorImpulse;

	bool m_enableMotor;
	float m_maxMotorTorque;
	float m_motorSpeed;

	bool m_enableLimit;
	float m_referenceAngle;
	float m_lowerAngle;
	float m_upperAngle;

	// Solver temp
	int m_indexA;
	int m_indexB;
	ci::Vec2f m_rA;
	ci::Vec2f m_rB;
	ci::Vec2f m_localCenterA;
	ci::Vec2f m_localCenterB;
	float m_invMassA;
	float m_invMassB;
	float m_invIA;
	float m_invIB;
	ci::Matrix33f m_mass;			// effective mass for point-to-point constraint.
	float m_motorMass;	// effective mass for motor/limit angular constraint.
	cb2LimitState m_limitState;
};

inline float cb2RevoluteJoint::GetMotorSpeed() const
{
	return m_motorSpeed;
}

#endif
