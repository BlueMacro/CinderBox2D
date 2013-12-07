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

#ifndef CB2_ROPE_JOINT_H
#define CB2_ROPE_JOINT_H

#include <CinderBox2D/Dynamics/Joints/cb2Joint.h>

/// Rope joint definition. This requires two body anchor points and
/// a maximum lengths.
/// Note: by default the connected objects will not collide.
/// see collideConnected in cb2JointDef.
struct cb2RopeJointDef : public cb2JointDef
{
	cb2RopeJointDef()
	{
		type = e_ropeJoint;
		localAnchorA.set(-1.0f, 0.0f);
		localAnchorB.set(1.0f, 0.0f);
		maxLength = 0.0f;
	}

	/// The local anchor point relative to bodyA's origin.
	ci::Vec2f localAnchorA;

	/// The local anchor point relative to bodyB's origin.
	ci::Vec2f localAnchorB;

	/// The maximum length of the rope.
	/// Warning: this must be larger than cb2_linearSlop or
	/// the joint will have no effect.
	float maxLength;
};

/// A rope joint enforces a maximum distance between two points
/// on two bodies. It has no other effect.
/// Warning: if you attempt to change the maximum length during
/// the simulation you will get some non-physical behavior.
/// A model that would allow you to dynamically modify the length
/// would have some sponginess, so I chose not to implement it
/// that way. See cb2DistanceJoint if you want to dynamically
/// control length.
class cb2RopeJoint : public cb2Joint
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

	/// set/Get the maximum length of the rope.
	void SetMaxLength(float length) { m_maxLength = length; }
	float GetMaxLength() const;

	cb2LimitState GetLimitState() const;

	/// Dump joint to dmLog
	void Dump();

protected:

	friend class cb2Joint;
	cb2RopeJoint(const cb2RopeJointDef* data);

	void InitVelocityConstraints(const cb2SolverData& data);
	void SolveVelocityConstraints(const cb2SolverData& data);
	bool SolvePositionConstraints(const cb2SolverData& data);

	// Solver shared
	ci::Vec2f m_localAnchorA;
	ci::Vec2f m_localAnchorB;
	float m_maxLength;
	float m_length;
	float m_impulse;

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
	cb2LimitState m_state;
};

#endif
