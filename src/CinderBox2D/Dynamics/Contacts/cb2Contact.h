/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
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

#ifndef CB2_CONTACT_H
#define CB2_CONTACT_H

#include <CinderBox2D/Common/cb2Math.h>
#include <CinderBox2D/Collision/cb2Collision.h>
#include <CinderBox2D/Collision/Shapes/cb2Shape.h>
#include <CinderBox2D/Dynamics/cb2Fixture.h>

class cb2Body;
class cb2Contact;
class cb2Fixture;
class cb2World;
class cb2BlockAllocator;
class cb2StackAllocator;
class cb2ContactListener;

/// Friction mixing law. The idea is to allow either fixture to drive the restitution to zero.
/// For example, anything slides on ice.
inline float cb2MixFriction(float friction1, float friction2)
{
	return cb2Sqrt(friction1 * friction2);
}

/// Restitution mixing law. The idea is allow for anything to bounce off an inelastic surface.
/// For example, a superball bounces on anything.
inline float cb2MixRestitution(float restitution1, float restitution2)
{
	return restitution1 > restitution2 ? restitution1 : restitution2;
}

typedef cb2Contact* cb2ContactCreateFcn(	cb2Fixture* fixtureA, int indexA,
										cb2Fixture* fixtureB, int indexB,
										cb2BlockAllocator* allocator);
typedef void cb2ContactDestroyFcn(cb2Contact* contact, cb2BlockAllocator* allocator);

struct cb2ContactRegister
{
	cb2ContactCreateFcn* createFcn;
	cb2ContactDestroyFcn* destroyFcn;
	bool primary;
};

/// A contact edge is used to connect bodies and contacts together
/// in a contact graph where each body is a node and each contact
/// is an edge. A contact edge belongs to a doubly linked list
/// maintained in each attached body. Each contact has two contact
/// nodes, one for each attached body.
struct cb2ContactEdge
{
	cb2Body* other;			///< provides quick access to the other body attached.
	cb2Contact* contact;		///< the contact
	cb2ContactEdge* prev;	///< the previous contact edge in the body's contact list
	cb2ContactEdge* next;	///< the next contact edge in the body's contact list
};

/// The class manages contact between two shapes. A contact exists for each overlapping
/// AABB in the broad-phase (except if filtered). Therefore a contact object may exist
/// that has no contact points.
class cb2Contact
{
public:

	/// Get the contact manifold. Do not modify the manifold unless you understand the
	/// internals of Box2D.
	cb2Manifold* GetManifold();
	const cb2Manifold* GetManifold() const;

	/// Get the world manifold.
	void GetWorldManifold(cb2WorldManifold* worldManifold) const;

	/// Is this contact touching?
	bool IsTouching() const;

	/// Enable/disable this contact. This can be used inside the pre-solve
	/// contact listener. The contact is only disabled for the current
	/// time step (or sub-step in continuous collisions).
	void SetEnabled(bool flag);

	/// Has this contact been disabled?
	bool IsEnabled() const;

	/// Get the next contact in the world's contact list.
	cb2Contact* GetNext();
	const cb2Contact* GetNext() const;

	/// Get fixture A in this contact.
	cb2Fixture* GetFixtureA();
	const cb2Fixture* GetFixtureA() const;

	/// Get the child primitive index for fixture A.
	int GetChildIndexA() const;

	/// Get fixture B in this contact.
	cb2Fixture* GetFixtureB();
	const cb2Fixture* GetFixtureB() const;

	/// Get the child primitive index for fixture B.
	int GetChildIndexB() const;

	/// Override the default friction mixture. You can call this in cb2ContactListener::PreSolve.
	/// This value persists until set or reset.
	void SetFriction(float friction);

	/// Get the friction.
	float GetFriction() const;

	/// Reset the friction mixture to the default value.
	void ResetFriction();

	/// Override the default restitution mixture. You can call this in cb2ContactListener::PreSolve.
	/// The value persists until you set or reset.
	void SetRestitution(float restitution);

	/// Get the restitution.
	float GetRestitution() const;

	/// Reset the restitution to the default value.
	void ResetRestitution();

	/// Set the desired tangent speed for a conveyor belt behavior. In meters per second.
	void SetTangentSpeed(float speed);

	/// Get the desired tangent speed. In meters per second.
	float GetTangentSpeed() const;

	/// Evaluate this contact with your own manifold and transforms.
	virtual void Evaluate(cb2Manifold* manifold, const cb2Transform& xfA, const cb2Transform& xfB) = 0;

protected:
	friend class cb2ContactManager;
	friend class cb2World;
	friend class cb2ContactSolver;
	friend class cb2Body;
	friend class cb2Fixture;

	// Flags stored in m_flags
	enum
	{
		// Used when crawling contact graph when forming islands.
		e_islandFlag		= 0x0001,

        // set when the shapes are touching.
		e_touchingFlag		= 0x0002,

		// This contact can be disabled (by user)
		e_enabledFlag		= 0x0004,

		// This contact needs filtering because a fixture filter was changed.
		e_filterFlag		= 0x0008,

		// This bullet contact had a TOI event
		e_bulletHitFlag		= 0x0010,

		// This contact has a valid TOI in m_toi
		e_toiFlag			= 0x0020
	};

	/// Flag this contact for filtering. Filtering will occur the next time step.
	void FlagForFiltering();

	static void AddType(cb2ContactCreateFcn* createFcn, cb2ContactDestroyFcn* destroyFcn,
						cb2Shape::Type typeA, cb2Shape::Type typeB);
	static void InitializeRegisters();
	static cb2Contact* Create(cb2Fixture* fixtureA, int indexA, cb2Fixture* fixtureB, int indexB, cb2BlockAllocator* allocator);
	static void Destroy(cb2Contact* contact, cb2Shape::Type typeA, cb2Shape::Type typeB, cb2BlockAllocator* allocator);
	static void Destroy(cb2Contact* contact, cb2BlockAllocator* allocator);

	cb2Contact() : m_fixtureA(NULL), m_fixtureB(NULL) {}
	cb2Contact(cb2Fixture* fixtureA, int indexA, cb2Fixture* fixtureB, int indexB);
	virtual ~cb2Contact() {}

	void Update(cb2ContactListener* listener);

	static cb2ContactRegister s_registers[cb2Shape::e_typeCount][cb2Shape::e_typeCount];
	static bool s_initialized;

	unsigned int m_flags;

	// World pool and list pointers.
	cb2Contact* m_prev;
	cb2Contact* m_next;

	// Nodes for connecting bodies.
	cb2ContactEdge m_nodeA;
	cb2ContactEdge m_nodeB;

	cb2Fixture* m_fixtureA;
	cb2Fixture* m_fixtureB;

	int m_indexA;
	int m_indexB;

	cb2Manifold m_manifold;

	int m_toiCount;
	float m_toi;

	float m_friction;
	float m_restitution;

	float m_tangentSpeed;
};

inline cb2Manifold* cb2Contact::GetManifold()
{
	return &m_manifold;
}

inline const cb2Manifold* cb2Contact::GetManifold() const
{
	return &m_manifold;
}

inline void cb2Contact::GetWorldManifold(cb2WorldManifold* worldManifold) const
{
	const cb2Body* bodyA = m_fixtureA->GetBody();
	const cb2Body* bodyB = m_fixtureB->GetBody();
	const cb2Shape* shapeA = m_fixtureA->GetShape();
	const cb2Shape* shapeB = m_fixtureB->GetShape();

	worldManifold->Initialize(&m_manifold, bodyA->GetTransform(), shapeA->m_radius, bodyB->GetTransform(), shapeB->m_radius);
}

inline void cb2Contact::SetEnabled(bool flag)
{
	if (flag)
	{
		m_flags |= e_enabledFlag;
	}
	else
	{
		m_flags &= ~e_enabledFlag;
	}
}

inline bool cb2Contact::IsEnabled() const
{
	return (m_flags & e_enabledFlag) == e_enabledFlag;
}

inline bool cb2Contact::IsTouching() const
{
	return (m_flags & e_touchingFlag) == e_touchingFlag;
}

inline cb2Contact* cb2Contact::GetNext()
{
	return m_next;
}

inline const cb2Contact* cb2Contact::GetNext() const
{
	return m_next;
}

inline cb2Fixture* cb2Contact::GetFixtureA()
{
	return m_fixtureA;
}

inline const cb2Fixture* cb2Contact::GetFixtureA() const
{
	return m_fixtureA;
}

inline cb2Fixture* cb2Contact::GetFixtureB()
{
	return m_fixtureB;
}

inline int cb2Contact::GetChildIndexA() const
{
	return m_indexA;
}

inline const cb2Fixture* cb2Contact::GetFixtureB() const
{
	return m_fixtureB;
}

inline int cb2Contact::GetChildIndexB() const
{
	return m_indexB;
}

inline void cb2Contact::FlagForFiltering()
{
	m_flags |= e_filterFlag;
}

inline void cb2Contact::SetFriction(float friction)
{
	m_friction = friction;
}

inline float cb2Contact::GetFriction() const
{
	return m_friction;
}

inline void cb2Contact::ResetFriction()
{
	m_friction = cb2MixFriction(m_fixtureA->m_friction, m_fixtureB->m_friction);
}

inline void cb2Contact::SetRestitution(float restitution)
{
	m_restitution = restitution;
}

inline float cb2Contact::GetRestitution() const
{
	return m_restitution;
}

inline void cb2Contact::ResetRestitution()
{
	m_restitution = cb2MixRestitution(m_fixtureA->m_restitution, m_fixtureB->m_restitution);
}

inline void cb2Contact::SetTangentSpeed(float speed)
{
	m_tangentSpeed = speed;
}

inline float cb2Contact::GetTangentSpeed() const
{
	return m_tangentSpeed;
}

#endif
