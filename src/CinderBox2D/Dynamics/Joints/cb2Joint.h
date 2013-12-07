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

#ifndef CB2_JOINT_H
#define CB2_JOINT_H

#include <CinderBox2D/Common/cb2Math.h>

class cb2Body;
class cb2Joint;
struct cb2SolverData;
class cb2BlockAllocator;

enum cb2JointType
{
	e_unknownJoint,
	e_revoluteJoint,
	e_prismaticJoint,
	e_distanceJoint,
	e_pulleyJoint,
	e_mouseJoint,
	e_gearJoint,
	e_wheelJoint,
    e_weldJoint,
	e_frictionJoint,
	e_ropeJoint,
	e_motorJoint
};

enum cb2LimitState
{
	e_inactiveLimit,
	e_atLowerLimit,
	e_atUpperLimit,
	e_equalLimits
};

struct cb2Jacobian
{
	ci::Vec2f linear;
	float angularA;
	float angularB;
};

/// A joint edge is used to connect bodies and joints together
/// in a joint graph where each body is a node and each joint
/// is an edge. A joint edge belongs to a doubly linked list
/// maintained in each attached body. Each joint has two joint
/// nodes, one for each attached body.
struct cb2JointEdge
{
	cb2Body* other;			///< provides quick access to the other body attached.
	cb2Joint* joint;			///< the joint
	cb2JointEdge* prev;		///< the previous joint edge in the body's joint list
	cb2JointEdge* next;		///< the next joint edge in the body's joint list
};

/// Joint definitions are used to construct joints.
struct cb2JointDef
{
	cb2JointDef()
	{
		type = e_unknownJoint;
		userData = NULL;
		bodyA = NULL;
		bodyB = NULL;
		collideConnected = false;
	}

	/// The joint type is set automatically for concrete joint types.
	cb2JointType type;

	/// Use this to attach application specific data to your joints.
	void* userData;

	/// The first attached body.
	cb2Body* bodyA;

	/// The second attached body.
	cb2Body* bodyB;

	/// set this flag to true if the attached bodies should collide.
	bool collideConnected;
};

/// The base joint class. Joints are used to constraint two bodies together in
/// various fashions. Some joints also feature limits and motors.
class cb2Joint
{
public:

	/// Get the type of the concrete joint.
	cb2JointType GetType() const;

	/// Get the first body attached to this joint.
	cb2Body* GetBodyA();

	/// Get the second body attached to this joint.
	cb2Body* GetBodyB();

	/// Get the anchor point on bodyA in world coordinates.
	virtual ci::Vec2f GetAnchorA() const = 0;

	/// Get the anchor point on bodyB in world coordinates.
	virtual ci::Vec2f GetAnchorB() const = 0;

	/// Get the reaction force on bodyB at the joint anchor in Newtons.
	virtual ci::Vec2f GetReactionForce(float inv_dt) const = 0;

	/// Get the reaction torque on bodyB in N*m.
	virtual float GetReactionTorque(float inv_dt) const = 0;

	/// Get the next joint the world joint list.
	cb2Joint* GetNext();
	const cb2Joint* GetNext() const;

	/// Get the user data pointer.
	void* GetUserData() const;

	/// set the user data pointer.
	void SetUserData(void* data);

	/// Short-cut function to determine if either body is inactive.
	bool IsActive() const;

	/// Get collide connected.
	/// Note: modifying the collide connect flag won't work correctly because
	/// the flag is only checked when fixture AABBs begin to overlap.
	bool GetCollideConnected() const;

	/// Dump this joint to the log file.
	virtual void Dump() { cb2Log("// Dump is not supported for this joint type.\n"); }

	/// Shift the origin for any points stored in world coordinates.
	virtual void ShiftOrigin(const ci::Vec2f& newOrigin) { CB2_NOT_USED(newOrigin);  }

protected:
	friend class cb2World;
	friend class cb2Body;
	friend class cb2Island;
	friend class cb2GearJoint;

	static cb2Joint* Create(const cb2JointDef* def, cb2BlockAllocator* allocator);
	static void Destroy(cb2Joint* joint, cb2BlockAllocator* allocator);

	cb2Joint(const cb2JointDef* def);
	virtual ~cb2Joint() {}

	virtual void InitVelocityConstraints(const cb2SolverData& data) = 0;
	virtual void SolveVelocityConstraints(const cb2SolverData& data) = 0;

	// This returns true if the position errors are within tolerance.
	virtual bool SolvePositionConstraints(const cb2SolverData& data) = 0;

	cb2JointType m_type;
	cb2Joint* m_prev;
	cb2Joint* m_next;
	cb2JointEdge m_edgeA;
	cb2JointEdge m_edgeB;
	cb2Body* m_bodyA;
	cb2Body* m_bodyB;

	int m_index;

	bool m_islandFlag;
	bool m_collideConnected;

	void* m_userData;
};

inline cb2JointType cb2Joint::GetType() const
{
	return m_type;
}

inline cb2Body* cb2Joint::GetBodyA()
{
	return m_bodyA;
}

inline cb2Body* cb2Joint::GetBodyB()
{
	return m_bodyB;
}

inline cb2Joint* cb2Joint::GetNext()
{
	return m_next;
}

inline const cb2Joint* cb2Joint::GetNext() const
{
	return m_next;
}

inline void* cb2Joint::GetUserData() const
{
	return m_userData;
}

inline void cb2Joint::SetUserData(void* data)
{
	m_userData = data;
}

inline bool cb2Joint::GetCollideConnected() const
{
	return m_collideConnected;
}

#endif
