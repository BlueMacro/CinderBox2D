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

#ifndef CB2_WELD_JOINT_H
#define CB2_WELD_JOINT_H

#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>

/// Weld joint definition. You need to specify local anchor points
/// where they are attached and the relative body angle. The position
/// of the anchor points is important for computing the reaction torque.
struct cb2WeldJointDef : public cb2JointDef
{
	cb2WeldJointDef()
	{
		type = e_weldJoint;
		localAnchorA.set(0.0f, 0.0f);
		localAnchorB.set(0.0f, 0.0f);
		referenceAngle = 0.0f;
		frequencyHz = 0.0f;
		dampingRatio = 0.0f;
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
	
	/// The mass-spring-damper frequency in Hertz. Rotation only.
	/// Disable softness with a value of 0.
	float frequencyHz;

	/// The damping ratio. 0 = no damping, 1 = critical damping.
	float dampingRatio;
};

/// A weld joint essentially glues two bodies together. A weld joint may
/// distort somewhat because the island constraint solver is approximate.
class cb2WeldJoint : public cb2Joint
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

	/// Get the reference angle.
	float GetReferenceAngle() const { return m_referenceAngle; }

	/// set/get frequency in Hz.
	void SetFrequency(float hz) { m_frequencyHz = hz; }
	float GetFrequency() const { return m_frequencyHz; }

	/// set/get damping ratio.
	void SetDampingRatio(float ratio) { m_dampingRatio = ratio; }
	float GetDampingRatio() const { return m_dampingRatio; }

	/// Dump to cb2Log
	void Dump();

protected:

	friend class cb2Joint;

	cb2WeldJoint(const cb2WeldJointDef* def);

	void InitVelocityConstraints(const cb2SolverData& data);
	void SolveVelocityConstraints(const cb2SolverData& data);
	bool SolvePositionConstraints(const cb2SolverData& data);

	float m_frequencyHz;
	float m_dampingRatio;
	float m_bias;

	// Solver shared
	ci::Vec2f m_localAnchorA;
	ci::Vec2f m_localAnchorB;
	float m_referenceAngle;
	float m_gamma;
	ci::Vec3f m_impulse;

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
	ci::Matrix33f m_mass;
};

#endif
