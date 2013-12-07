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

#ifndef CB2_DISTANCE_JOINT_H
#define CB2_DISTANCE_JOINT_H

#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>

/// Distance joint definition. This requires defining an
/// anchor point on both bodies and the non-zero length of the
/// distance joint. The definition uses local anchor points
/// so that the initial configuration can violate the constraint
/// slightly. This helps when saving and loading a game.
/// @warning Do not use a zero or short length.
struct cb2DistanceJointDef : public cb2JointDef
{
	cb2DistanceJointDef()
	{
		type = e_distanceJoint;
		localAnchorA.set(0.0f, 0.0f);
		localAnchorB.set(0.0f, 0.0f);
		length = 1.0f;
		frequencyHz = 0.0f;
		dampingRatio = 0.0f;
	}

	/// Initialize the bodies, anchors, and length using the world
	/// anchors.
	void Initialize(cb2Body* bodyA, cb2Body* bodyB,
					const ci::Vec2f& anchorA, const ci::Vec2f& anchorB);

	/// The local anchor point relative to bodyA's origin.
	ci::Vec2f localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	ci::Vec2f localAnchorB;

	/// The natural length between the anchor points.
	float length;

	/// The mass-spring-damper frequency in Hertz. A value of 0
	/// disables softness.
	float frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float dampingRatio;
};

/// A distance joint constrains two points on two bodies
/// to remain at a fixed distance from each other. You can view
/// this as a massless, rigid rod.
class cb2DistanceJoint : public cb2Joint
{
public:

	ci::Vec2f GetAnchorA() const;
	ci::Vec2f GetAnchorB() const;

	/// Get the reaction force given the inverse time step.
	/// Unit is N.
	ci::Vec2f GetReactionForce(float inv_dt) const;

	/// Get the reaction torque given the inverse time step.
	/// Unit is N*m. This is always zero for a distance joint.
	float GetReactionTorque(float inv_dt) const;

	/// The local anchor point relative to bodyA's origin.
	const ci::Vec2f& GetLocalAnchorA() const { return m_localAnchorA; }

	/// The local anchor point relative to bodyB's origin.
	const ci::Vec2f& GetLocalAnchorB() const  { return m_localAnchorB; }

	/// set/get the natural length.
	/// Manipulating the length can lead to non-physical behavior when the frequency is zero.
	void SetLength(float length);
	float GetLength() const;

	/// set/get frequency in Hz.
	void SetFrequency(float hz);
	float GetFrequency() const;

	/// set/get damping ratio.
	void SetDampingRatio(float ratio);
	float GetDampingRatio() const;

	/// Dump joint to dmLog
	void Dump();

protected:

	friend class cb2Joint;
	cb2DistanceJoint(const cb2DistanceJointDef* data);

	void InitVelocityConstraints(const cb2SolverData& data);
	void SolveVelocityConstraints(const cb2SolverData& data);
	bool SolvePositionConstraints(const cb2SolverData& data);

	float m_frequencyHz;
	float m_dampingRatio;
	float m_bias;

	// Solver shared
	ci::Vec2f m_localAnchorA;
	ci::Vec2f m_localAnchorB;
	float m_gamma;
	float m_impulse;
	float m_length;

	// Solver temp
	int m_indexA;
	int m_indexB;
	ci::Vec2f m_u;
	ci::Vec2f m_rA;
	ci::Vec2f m_rB;
	ci::Vec2f m_localCenterA;
	ci::Vec2f m_localCenterB;
	float m_invMassA;
	float m_invMassB;
	float m_invIA;
	float m_invIB;
	float m_mass;
};

inline void cb2DistanceJoint::SetLength(float length)
{
	m_length = length;
}

inline float cb2DistanceJoint::GetLength() const
{
	return m_length;
}

inline void cb2DistanceJoint::SetFrequency(float hz)
{
	m_frequencyHz = hz;
}

inline float cb2DistanceJoint::GetFrequency() const
{
	return m_frequencyHz;
}

inline void cb2DistanceJoint::SetDampingRatio(float ratio)
{
	m_dampingRatio = ratio;
}

inline float cb2DistanceJoint::GetDampingRatio() const
{
	return m_dampingRatio;
}

#endif
