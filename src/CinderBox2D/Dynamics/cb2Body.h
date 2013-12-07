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

#ifndef CB2_BODY_H
#define CB2_BODY_H

#include <CinderBox2D/Common/cb2Math.h>
#include <CinderBox2D/Collision/Shapes/cb2Shape.h>
#include <memory>

class cb2Fixture;
class cb2Joint;
class cb2Contact;
class cb2Controller;
class cb2World;
struct cb2FixtureDef;
struct cb2JointEdge;
struct cb2ContactEdge;

/// The body type.
/// static: zero mass, zero velocity, may be manually moved
/// kinematic: zero mass, non-zero velocity set by user, moved by solver
/// dynamic: positive mass, non-zero velocity determined by forces, moved by solver
enum cb2BodyType
{
	cb2_staticBody = 0,
	cb2_kinematicBody,
	cb2_dynamicBody

	// TODO_ERIN
	//cb2_bulletBody,
};

/// A body definition holds all the data needed to construct a rigid body.
/// You can safely re-use body definitions. Shapes are added to a body after construction.
struct cb2BodyDef
{
	/// This constructor sets the body definition default values.
	cb2BodyDef()
	{
		userData = NULL;
		position.set(0.0f, 0.0f);
		angle = 0.0f;
		linearVelocity.set(0.0f, 0.0f);
		angularVelocity = 0.0f;
		linearDamping = 0.0f;
		angularDamping = 0.0f;
		allowSleep = true;
		awake = true;
		fixedRotation = false;
		bullet = false;
		type = cb2_staticBody;
		active = true;
		gravityScale = 1.0f;
	}

	/// The body type: static, kinematic, or dynamic.
	/// Note: if a dynamic body would have zero mass, the mass is set to one.
	cb2BodyType type;

	/// The world position of the body. Avoid creating bodies at the origin
	/// since this can lead to many overlapping shapes.
	ci::Vec2f position;

	/// The world angle of the body in radians.
	float angle;

	/// The linear velocity of the body's origin in world co-ordinates.
	ci::Vec2f linearVelocity;

	/// The angular velocity of the body.
	float angularVelocity;

	/// Linear damping is use to reduce the linear velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float linearDamping;

	/// Angular damping is use to reduce the angular velocity. The damping parameter
	/// can be larger than 1.0f but the damping effect becomes sensitive to the
	/// time step when the damping parameter is large.
	float angularDamping;

	/// set this flag to false if this body should never fall asleep. Note that
	/// this increases CPU usage.
	bool allowSleep;

	/// Is this body initially awake or sleeping?
	bool awake;

	/// Should this body be prevented from rotating? Useful for characters.
	bool fixedRotation;

	/// Is this a fast moving body that should be prevented from tunneling through
	/// other moving bodies? Note that all bodies are prevented from tunneling through
	/// kinematic and static bodies. This setting is only considered on dynamic bodies.
	/// @warning You should use this flag sparingly since it increases processing time.
	bool bullet;

	/// Does this body start out active?
	bool active;

	/// Use this to store application specific body data.
	void* userData;

	/// Scale the gravity applied to this body.
	float gravityScale;
};

/// A rigid body. These are created via cb2World::CreateBody.
class cb2Body
{
public:
	/// Creates a fixture and attach it to this body. Use this function if you need
	/// to set some fixture parameters, like friction. Otherwise you can create the
	/// fixture directly from a shape.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// Contacts are not created until the next time step.
	/// @param def the fixture definition.
	/// @warning This function is locked during callbacks.
	cb2Fixture* CreateFixture(const cb2FixtureDef* def);

	/// Creates a fixture from a shape and attach it to this body.
	/// This is a convenience function. Use cb2FixtureDef if you need to set parameters
	/// like friction, restitution, user data, or filtering.
	/// If the density is non-zero, this function automatically updates the mass of the body.
	/// @param shape the shape to be cloned.
	/// @param density the shape density (set to zero for static bodies).
	/// @warning This function is locked during callbacks.
	cb2Fixture* CreateFixture(const cb2Shape* shape, float density);

	/// Destroy a fixture. This removes the fixture from the broad-phase and
	/// destroys all contacts associated with this fixture. This will
	/// automatically adjust the mass of the body if the body is dynamic and the
	/// fixture has positive density.
	/// All fixtures attached to a body are implicitly destroyed when the body is destroyed.
	/// @param fixture the fixture to be removed.
	/// @warning This function is locked during callbacks.
	void DestroyFixture(cb2Fixture* fixture);

	/// set the position of the body's origin and rotation.
	/// This breaks any contacts and wakes the other bodies.
	/// Manipulating a body's transform may cause non-physical behavior.
	/// Note: contacts are updated on the next call to cb2World::Step.
	/// @param position the world position of the body's local origin.
	/// @param angle the world rotation in radians.
	void SetTransform(const ci::Vec2f& position, float angle);

	/// Get the body transform for the body's origin.
	/// @return the world transform of the body's origin.
	const cb2Transform& GetTransform() const;
  
	/// Set the world body origin position.
	/// @param position the world position of the body's local origin.
	void SetPosition(ci::Vec2f& position);

	/// Get the world body origin position.
	/// @return the world position of the body's origin.
	const ci::Vec2f& GetPosition() const;

	/// Get the angle in radians.
	/// @return the current world rotation angle in radians.
	float GetAngle() const;

	/// Get the world position of the center of mass.
	const ci::Vec2f& GetWorldCenter() const;

	/// Get the local position of the center of mass.
	const ci::Vec2f& GetLocalCenter() const;

	/// set the linear velocity of the center of mass.
	/// @param v the new linear velocity of the center of mass.
	void SetLinearVelocity(const ci::Vec2f& v);

	/// Get the linear velocity of the center of mass.
	/// @return the linear velocity of the center of mass.
	ci::Vec2f GetLinearVelocity() const;

	/// set the angular velocity.
	/// @param omega the new angular velocity in radians/second.
	void SetAngularVelocity(float omega);

	/// Get the angular velocity.
	/// @return the angular velocity in radians/second.
	float GetAngularVelocity() const;

	/// Apply a force at a world point. If the force is not
	/// applied at the center of mass, it will generate a torque and
	/// affect the angular velocity. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyForce(const ci::Vec2f& force, const ci::Vec2f& point, bool wake);

	/// Apply a force to the center of mass. This wakes up the body.
	/// @param force the world force vector, usually in Newtons (N).
	/// @param wake also wake up the body
	void ApplyForceToCenter(const ci::Vec2f& force, bool wake);

	/// Apply a torque. This affects the angular velocity
	/// without affecting the linear velocity of the center of mass.
	/// This wakes up the body.
	/// @param torque about the z-axis (out of the screen), usually in N-m.
	/// @param wake also wake up the body
	void ApplyTorque(float torque, bool wake);

	/// Apply an impulse at a point. This immediately modifies the velocity.
	/// It also modifies the angular velocity if the point of application
	/// is not at the center of mass. This wakes up the body.
	/// @param impulse the world impulse vector, usually in N-seconds or kg-m/s.
	/// @param point the world position of the point of application.
	/// @param wake also wake up the body
	void ApplyLinearImpulse(const ci::Vec2f& impulse, const ci::Vec2f& point, bool wake);

	/// Apply an angular impulse.
	/// @param impulse the angular impulse in units of kg*m*m/s
	/// @param wake also wake up the body
	void ApplyAngularImpulse(float impulse, bool wake);

	/// Get the total mass of the body.
	/// @return the mass, usually in kilograms (kg).
	float GetMass() const;

	/// Get the rotational inertia of the body about the local origin.
	/// @return the rotational inertia, usually in kg-m^2.
	float GetInertia() const;

	/// Get the mass data of the body.
	/// @return a struct containing the mass, inertia and center of the body.
	void GetMassData(cb2MassData* data) const;

	/// set the mass properties to override the mass properties of the fixtures.
	/// Note that this changes the center of mass position.
	/// Note that creating or destroying fixtures can also alter the mass.
	/// This function has no effect if the body isn't dynamic.
	/// @param massData the mass properties.
	void SetMassData(const cb2MassData* data);

	/// This resets the mass properties to the sum of the mass properties of the fixtures.
	/// This normally does not need to be called unless you called SetMassData to override
	/// the mass and you later want to reset the mass.
	void ResetMassData();

	/// Get the world coordinates of a point given the local coordinates.
	/// @param localPoint a point on the body measured relative the the body's origin.
	/// @return the same point expressed in world coordinates.
	ci::Vec2f GetWorldPoint(const ci::Vec2f& localPoint) const;

	/// Get the world coordinates of a vector given the local coordinates.
	/// @param localVector a vector fixed in the body.
	/// @return the same vector expressed in world coordinates.
	ci::Vec2f GetWorldVector(const ci::Vec2f& localVector) const;

	/// Gets a local point relative to the body's origin given a world point.
	/// @param a point in world coordinates.
	/// @return the corresponding local point relative to the body's origin.
	ci::Vec2f GetLocalPoint(const ci::Vec2f& worldPoint) const;

	/// Gets a local vector given a world vector.
	/// @param a vector in world coordinates.
	/// @return the corresponding local vector.
	ci::Vec2f GetLocalVector(const ci::Vec2f& worldVector) const;

	/// Get the world linear velocity of a world point attached to this body.
	/// @param a point in world coordinates.
	/// @return the world velocity of a point.
	ci::Vec2f GetLinearVelocityFromWorldPoint(const ci::Vec2f& worldPoint) const;

	/// Get the world velocity of a local point.
	/// @param a point in local coordinates.
	/// @return the world velocity of a point.
	ci::Vec2f GetLinearVelocityFromLocalPoint(const ci::Vec2f& localPoint) const;

	/// Get the linear damping of the body.
	float GetLinearDamping() const;

	/// set the linear damping of the body.
	void SetLinearDamping(float linearDamping);

	/// Get the angular damping of the body.
	float GetAngularDamping() const;

	/// set the angular damping of the body.
	void SetAngularDamping(float angularDamping);

	/// Get the gravity scale of the body.
	float GetGravityScale() const;

	/// set the gravity scale of the body.
	void SetGravityScale(float scale);

	/// set the type of this body. This may alter the mass and velocity.
	void SetType(cb2BodyType type);

	/// Get the type of this body.
	cb2BodyType GetType() const;

	/// Should this body be treated like a bullet for continuous collision detection?
	void SetBullet(bool flag);

	/// Is this body treated like a bullet for continuous collision detection?
	bool IsBullet() const;

	/// You can disable sleeping on this body. If you disable sleeping, the
	/// body will be woken.
	void SetSleepingAllowed(bool flag);

	/// Is this body allowed to sleep
	bool IsSleepingAllowed() const;

	/// set the sleep state of the body. A sleeping body has very
	/// low CPU cost.
	/// @param flag set to true to wake the body, false to put it to sleep.
	void SetAwake(bool flag);

	/// Get the sleeping state of this body.
	/// @return true if the body is awake.
	bool IsAwake() const;

	/// Set the active state of the body. An inactive body is not
	/// simulated and cannot be collided with or woken up.
	/// If you pass a flag of true, all fixtures will be added to the
	/// broad-phase.
	/// If you pass a flag of false, all fixtures will be removed from
	/// the broad-phase and all contacts will be destroyed.
	/// Fixtures and joints are otherwise unaffected. You may continue
	/// to create/destroy fixtures and joints on inactive bodies.
	/// Fixtures on an inactive body are implicitly inactive and will
	/// not participate in collisions, ray-casts, or queries.
	/// Joints connected to an inactive body are implicitly inactive.
	/// An inactive body is still owned by a cb2World object and remains
	/// in the body list.
	void SetActive(bool flag);

	/// Get the active state of the body.
	bool IsActive() const;

	/// Set this body to have fixed rotation. This causes the mass
	/// to be reset.
	void SetFixedRotation(bool flag);

	/// Does this body have fixed rotation?
	bool IsFixedRotation() const;

	/// Get the list of all fixtures attached to this body.
	cb2Fixture* GetFixtureList();
	const cb2Fixture* GetFixtureList() const;

	/// Get the list of all joints attached to this body.
	cb2JointEdge* GetJointList();
	const cb2JointEdge* GetJointList() const;

	/// Get the list of all contacts attached to this body.
	/// @warning this list changes during the time step and you may
	/// miss some collisions if you don't use cb2ContactListener.
	cb2ContactEdge* GetContactList();
	const cb2ContactEdge* GetContactList() const;

	/// Get the next body in the world's body list.
	cb2Body* GetNext();
	const cb2Body* GetNext() const;

	/// Get the user data pointer that was provided in the body definition.
	void* GetUserData() const;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data);

	/// Get the parent world of this body.
	cb2World* GetWorld();
	const cb2World* GetWorld() const;

	/// Dump this body to a log file
	void Dump();

private:

	friend class cb2World;
	friend class cb2Island;
	friend class cb2ContactManager;
	friend class cb2ContactSolver;
	friend class cb2Contact;
	
	friend class cb2DistanceJoint;
	friend class cb2FrictionJoint;
	friend class cb2GearJoint;
	friend class cb2MotorJoint;
	friend class cb2MouseJoint;
	friend class cb2PrismaticJoint;
	friend class cb2PulleyJoint;
	friend class cb2RevoluteJoint;
	friend class cb2RopeJoint;
	friend class cb2WeldJoint;
	friend class cb2WheelJoint;

	// m_flags
	enum
	{
		e_islandFlag		= 0x0001,
		e_awakeFlag			= 0x0002,
		e_autoSleepFlag		= 0x0004,
		e_bulletFlag		= 0x0008,
		e_fixedRotationFlag	= 0x0010,
		e_activeFlag		= 0x0020,
		e_toiFlag			= 0x0040
	};

	cb2Body(const cb2BodyDef* bd, cb2World* world);
	~cb2Body();

	void SynchronizeFixtures();
	void SynchronizeTransform();

	// This is used to prevent connected bodies from colliding.
	// It may lie, depending on the collideConnected flag.
	bool ShouldCollide(const cb2Body* other) const;

	void Advance(float t);

	cb2BodyType m_type;

	unsigned short m_flags;

	int m_islandIndex;

	cb2Transform m_xf;		// the body origin transform
	cb2Sweep m_sweep;		// the swept motion for CCD

	ci::Vec2f m_linearVelocity;
	float m_angularVelocity;

	ci::Vec2f m_force;
	float m_torque;

	cb2World* m_world;
	cb2Body* m_prev;
	cb2Body* m_next;

	cb2Fixture* m_fixtureList;
	int m_fixtureCount;

	cb2JointEdge* m_jointList;
	cb2ContactEdge* m_contactList;

	float m_mass, m_invMass;

	// Rotational inertia about the center of mass.
	float m_I, m_invI;

	float m_linearDamping;
	float m_angularDamping;
	float m_gravityScale;

	float m_sleepTime;

	void* m_userData;
};

inline cb2BodyType cb2Body::GetType() const
{
	return m_type;
}

inline const cb2Transform& cb2Body::GetTransform() const
{
	return m_xf;
}

inline const ci::Vec2f& cb2Body::GetPosition() const
{
	return m_xf.p;
}

inline float cb2Body::GetAngle() const
{
	return m_sweep.a;
}

inline const ci::Vec2f& cb2Body::GetWorldCenter() const
{
	return m_sweep.c;
}

inline const ci::Vec2f& cb2Body::GetLocalCenter() const
{
	return m_sweep.localCenter;
}

inline void cb2Body::SetLinearVelocity(const ci::Vec2f& v)
{
	if (m_type == cb2_staticBody)
	{
		return;
	}

	if (cb2Dot(v,v) > 0.0f)
	{
		SetAwake(true);
	}

	m_linearVelocity = v;
}

inline ci::Vec2f cb2Body::GetLinearVelocity() const
{
	return m_linearVelocity;
}

inline void cb2Body::SetAngularVelocity(float w)
{
	if (m_type == cb2_staticBody)
	{
		return;
	}

	if (w * w > 0.0f)
	{
		SetAwake(true);
	}

	m_angularVelocity = w;
}

inline float cb2Body::GetAngularVelocity() const
{
	return m_angularVelocity;
}

inline float cb2Body::GetMass() const
{
	return m_mass;
}

inline float cb2Body::GetInertia() const
{
	return m_I + m_mass * cb2Dot(m_sweep.localCenter, m_sweep.localCenter);
}

inline void cb2Body::GetMassData(cb2MassData* data) const
{
	data->mass = m_mass;
	data->I = m_I + m_mass * cb2Dot(m_sweep.localCenter, m_sweep.localCenter);
	data->center = m_sweep.localCenter;
}

inline ci::Vec2f cb2Body::GetWorldPoint(const ci::Vec2f& localPoint) const
{
	return cb2Mul(m_xf, localPoint);
}

inline ci::Vec2f cb2Body::GetWorldVector(const ci::Vec2f& localVector) const
{
	return cb2Mul(m_xf.q, localVector);
}

inline ci::Vec2f cb2Body::GetLocalPoint(const ci::Vec2f& worldPoint) const
{
	return cb2MulT(m_xf, worldPoint);
}

inline ci::Vec2f cb2Body::GetLocalVector(const ci::Vec2f& worldVector) const
{
	return cb2MulT(m_xf.q, worldVector);
}

inline ci::Vec2f cb2Body::GetLinearVelocityFromWorldPoint(const ci::Vec2f& worldPoint) const
{
	return m_linearVelocity + cb2Cross(m_angularVelocity, worldPoint - m_sweep.c);
}

inline ci::Vec2f cb2Body::GetLinearVelocityFromLocalPoint(const ci::Vec2f& localPoint) const
{
	return GetLinearVelocityFromWorldPoint(GetWorldPoint(localPoint));
}

inline float cb2Body::GetLinearDamping() const
{
	return m_linearDamping;
}

inline void cb2Body::SetLinearDamping(float linearDamping)
{
	m_linearDamping = linearDamping;
}

inline float cb2Body::GetAngularDamping() const
{
	return m_angularDamping;
}

inline void cb2Body::SetAngularDamping(float angularDamping)
{
	m_angularDamping = angularDamping;
}

inline float cb2Body::GetGravityScale() const
{
	return m_gravityScale;
}

inline void cb2Body::SetGravityScale(float scale)
{
	m_gravityScale = scale;
}

inline void cb2Body::SetBullet(bool flag)
{
	if (flag)
	{
		m_flags |= e_bulletFlag;
	}
	else
	{
		m_flags &= ~e_bulletFlag;
	}
}

inline bool cb2Body::IsBullet() const
{
	return (m_flags & e_bulletFlag) == e_bulletFlag;
}

inline void cb2Body::SetAwake(bool flag)
{
	if (flag)
	{
		if ((m_flags & e_awakeFlag) == 0)
		{
			m_flags |= e_awakeFlag;
			m_sleepTime = 0.0f;
		}
	}
	else
	{
		m_flags &= ~e_awakeFlag;
		m_sleepTime = 0.0f;
		cb2::setZero(m_linearVelocity);
		m_angularVelocity = 0.0f;
		cb2::setZero(m_force);
		m_torque = 0.0f;
	}
}

inline bool cb2Body::IsAwake() const
{
	return (m_flags & e_awakeFlag) == e_awakeFlag;
}

inline bool cb2Body::IsActive() const
{
	return (m_flags & e_activeFlag) == e_activeFlag;
}

inline bool cb2Body::IsFixedRotation() const
{
	return (m_flags & e_fixedRotationFlag) == e_fixedRotationFlag;
}

inline void cb2Body::SetSleepingAllowed(bool flag)
{
	if (flag)
	{
		m_flags |= e_autoSleepFlag;
	}
	else
	{
		m_flags &= ~e_autoSleepFlag;
		SetAwake(true);
	}
}

inline bool cb2Body::IsSleepingAllowed() const
{
	return (m_flags & e_autoSleepFlag) == e_autoSleepFlag;
}

inline cb2Fixture* cb2Body::GetFixtureList()
{
	return m_fixtureList;
}

inline const cb2Fixture* cb2Body::GetFixtureList() const
{
	return m_fixtureList;
}

inline cb2JointEdge* cb2Body::GetJointList()
{
	return m_jointList;
}

inline const cb2JointEdge* cb2Body::GetJointList() const
{
	return m_jointList;
}

inline cb2ContactEdge* cb2Body::GetContactList()
{
	return m_contactList;
}

inline const cb2ContactEdge* cb2Body::GetContactList() const
{
	return m_contactList;
}

inline cb2Body* cb2Body::GetNext()
{
	return m_next;
}

inline const cb2Body* cb2Body::GetNext() const
{
	return m_next;
}

inline void cb2Body::SetUserData(void* data)
{
	m_userData = data;
}

inline void* cb2Body::GetUserData() const
{
	return m_userData;
}

inline void cb2Body::ApplyForce(const ci::Vec2f& force, const ci::Vec2f& point, bool wake)
{
	if (m_type != cb2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping.
	if (m_flags & e_awakeFlag)
	{
	m_force += force;
	m_torque += cb2Cross(point - m_sweep.c, force);
  }
}

inline void cb2Body::ApplyForceToCenter(const ci::Vec2f& force, bool wake)
{
	if (m_type != cb2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
    m_force += force;
  }
}

inline void cb2Body::ApplyTorque(float torque, bool wake)
{
	if (m_type != cb2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate a force if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
    m_torque += torque;
  }
}

inline void cb2Body::ApplyLinearImpulse(const ci::Vec2f& impulse, const ci::Vec2f& point, bool wake)
{
	if (m_type != cb2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
    m_linearVelocity += m_invMass * impulse;
    m_angularVelocity += m_invI * cb2Cross(point - m_sweep.c, impulse);
  }
}

inline void cb2Body::ApplyAngularImpulse(float impulse, bool wake)
{
	if (m_type != cb2_dynamicBody)
	{
		return;
	}

	if (wake && (m_flags & e_awakeFlag) == 0)
	{
		SetAwake(true);
	}

	// Don't accumulate velocity if the body is sleeping
	if (m_flags & e_awakeFlag)
	{
    m_angularVelocity += m_invI * impulse;
  }
}

inline void cb2Body::SynchronizeTransform()
{
	m_xf.q.set(m_sweep.a);
	m_xf.p = m_sweep.c - cb2Mul(m_xf.q, m_sweep.localCenter);
}

inline void cb2Body::Advance(float alpha)
{
	// Advance to the new safe time. This doesn't sync the broad-phase.
	m_sweep.Advance(alpha);
	m_sweep.c = m_sweep.c0;
	m_sweep.a = m_sweep.a0;
	m_xf.q.set(m_sweep.a);
	m_xf.p = m_sweep.c - cb2Mul(m_xf.q, m_sweep.localCenter);
}

inline cb2World* cb2Body::GetWorld()
{
	return m_world;
}

inline const cb2World* cb2Body::GetWorld() const
{
	return m_world;
}

#endif
