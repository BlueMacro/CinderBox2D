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

#ifndef CB2_FIXTURE_H
#define CB2_FIXTURE_H

#include <CinderBox2D/Dynamics/cb2Body.h>
#include <CinderBox2D/Collision/cb2Collision.h>
#include <CinderBox2D/Collision/Shapes/cb2Shape.h>

class cb2BlockAllocator;
class cb2Body;
class cb2BroadPhase;
class cb2Fixture;

/// This holds contact filtering data.
struct cb2Filter
{
	cb2Filter()
	{
		categoryBits = 0x0001;
		maskBits = 0xFFFF;
		groupIndex = 0;
	}

	/// The collision category bits. Normally you would just set one bit.
	unsigned short categoryBits;

	/// The collision mask bits. This states the categories that this
	/// shape would accept for collision.
	unsigned short maskBits;

	/// Collision groups allow a certain group of objects to never collide (negative)
	/// or always collide (positive). Zero means no collision group. Non-zero group
	/// filtering always wins against the mask bits.
	short groupIndex;
};

/// A fixture definition is used to create a fixture. This class defines an
/// abstract fixture definition. You can reuse fixture definitions safely.
struct cb2FixtureDef
{
	/// The constructor sets the default fixture definition values.
	cb2FixtureDef()
	{
		shape = NULL;
		userData = NULL;
		friction = 0.2f;
		restitution = 0.0f;
		density = 0.0f;
		isSensor = false;
	}

	/// The shape, this must be set. The shape will be cloned, so you
	/// can create the shape on the stack.
	const cb2Shape* shape;

	/// Use this to store application specific fixture data.
	void* userData;

	/// The friction coefficient, usually in the range [0,1].
	float friction;

	/// The restitution (elasticity) usually in the range [0,1].
	float restitution;

	/// The density, usually in kg/m^2.
	float density;

	/// A sensor shape collects contact information but never generates a collision
	/// response.
	bool isSensor;

	/// Contact filtering data.
	cb2Filter filter;
};

/// This proxy is used internally to connect fixtures to the broad-phase.
struct cb2FixtureProxy
{
	cb2AABB aabb;
	cb2Fixture* fixture;
	int childIndex;
	int proxyId;
};

/// A fixture is used to attach a shape to a body for collision detection. A fixture
/// inherits its transform from its parent. Fixtures hold additional non-geometric data
/// such as friction, collision filters, etc.
/// Fixtures are created via cb2Body::CreateFixture.
/// @warning you cannot reuse fixtures.
class cb2Fixture
{
public:
	/// Get the type of the child shape. You can use this to down cast to the concrete shape.
	/// @return the shape type.
	cb2Shape::Type GetType() const;

	/// Get the child shape. You can modify the child shape, however you should not change the
	/// number of vertices because this will crash some collision caching mechanisms.
	/// Manipulating the shape may lead to non-physical behavior.
	cb2Shape* GetShape();
	const cb2Shape* GetShape() const;

	/// Set if this fixture is a sensor.
	void SetSensor(bool sensor);

	/// Is this fixture a sensor (non-solid)?
	/// @return the true if the shape is a sensor.
	bool IsSensor() const;

	/// Set the contact filtering data. This will not update contacts until the next time
	/// step when either parent body is active and awake.
	/// This automatically calls Refilter.
	void SetFilterData(const cb2Filter& filter);

	/// Get the contact filtering data.
	const cb2Filter& GetFilterData() const;

	/// Call this if you want to establish collision that was previously disabled by cb2ContactFilter::ShouldCollide.
	void Refilter();

	/// Get the parent body of this fixture. This is NULL if the fixture is not attached.
	/// @return the parent body.
	cb2Body* GetBody();
	const cb2Body* GetBody() const;

	/// Get the next fixture in the parent body's fixture list.
	/// @return the next shape.
	cb2Fixture* GetNext();
	const cb2Fixture* GetNext() const;

	/// Get the user data that was assigned in the fixture definition. Use this to
	/// store your application specific data.
	void* GetUserData() const;

	/// Set the user data. Use this to store your application specific data.
	void SetUserData(void* data);

	/// Test a point for containment in this fixture.
	/// @param p a point in world coordinates.
	bool TestPoint(const ci::Vec2f& p) const;

	/// Cast a ray against this shape.
	/// @param output the ray-cast results.
	/// @param input the ray-cast input parameters.
	bool RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input, int childIndex) const;

	/// Get the mass data for this fixture. The mass data is based on the density and
	/// the shape. The rotational inertia is about the shape's origin. This operation
	/// may be expensive.
	void GetMassData(cb2MassData* massData) const;

	/// Set the density of this fixture. This will _not_ automatically adjust the mass
	/// of the body. You must call cb2Body::ResetMassData to update the body's mass.
	void SetDensity(float density);

	/// Get the density of this fixture.
	float GetDensity() const;

	/// Get the coefficient of friction.
	float GetFriction() const;

	/// Set the coefficient of friction. This will _not_ change the friction of
	/// existing contacts.
	void SetFriction(float friction);

	/// Get the coefficient of restitution.
	float GetRestitution() const;

	/// Set the coefficient of restitution. This will _not_ change the restitution of
	/// existing contacts.
	void SetRestitution(float restitution);

	/// Get the fixture's AABB. This AABB may be enlarge and/or stale.
	/// If you need a more accurate AABB, compute it using the shape and
	/// the body transform.
	const cb2AABB& GetAABB(int childIndex) const;

	/// Dump this fixture to the log file.
	void Dump(int bodyIndex);

protected:

	friend class cb2Body;
	friend class cb2World;
	friend class cb2Contact;
	friend class cb2ContactManager;

	cb2Fixture();

	// We need separation create/destroy functions from the constructor/destructor because
	// the destructor cannot access the allocator (no destructor arguments allowed by C++).
	void Create(cb2BlockAllocator* allocator, cb2Body* body, const cb2FixtureDef* def);
	void Destroy(cb2BlockAllocator* allocator);

	// These support body activation/deactivation.
	void CreateProxies(cb2BroadPhase* broadPhase, const cb2Transform& xf);
	void DestroyProxies(cb2BroadPhase* broadPhase);

	void Synchronize(cb2BroadPhase* broadPhase, const cb2Transform& xf1, const cb2Transform& xf2);

	float m_density;

	cb2Fixture* m_next;
	cb2Body* m_body;

	cb2Shape* m_shape;

	float m_friction;
	float m_restitution;

	cb2FixtureProxy* m_proxies;
	int m_proxyCount;

	cb2Filter m_filter;

	bool m_isSensor;

	void* m_userData;
};

inline cb2Shape::Type cb2Fixture::GetType() const
{
	return m_shape->GetType();
}

inline cb2Shape* cb2Fixture::GetShape()
{
	return m_shape;
}

inline const cb2Shape* cb2Fixture::GetShape() const
{
	return m_shape;
}

inline bool cb2Fixture::IsSensor() const
{
	return m_isSensor;
}

inline const cb2Filter& cb2Fixture::GetFilterData() const
{
	return m_filter;
}

inline void* cb2Fixture::GetUserData() const
{
	return m_userData;
}

inline void cb2Fixture::SetUserData(void* data)
{
	m_userData = data;
}

inline cb2Body* cb2Fixture::GetBody()
{
	return m_body;
}

inline const cb2Body* cb2Fixture::GetBody() const
{
	return m_body;
}

inline cb2Fixture* cb2Fixture::GetNext()
{
	return m_next;
}

inline const cb2Fixture* cb2Fixture::GetNext() const
{
	return m_next;
}

inline void cb2Fixture::SetDensity(float density)
{
	cb2Assert(cb2::isValid(density) && density >= 0.0f);
	m_density = density;
}

inline float cb2Fixture::GetDensity() const
{
	return m_density;
}

inline float cb2Fixture::GetFriction() const
{
	return m_friction;
}

inline void cb2Fixture::SetFriction(float friction)
{
	m_friction = friction;
}

inline float cb2Fixture::GetRestitution() const
{
	return m_restitution;
}

inline void cb2Fixture::SetRestitution(float restitution)
{
	m_restitution = restitution;
}

inline bool cb2Fixture::TestPoint(const ci::Vec2f& p) const
{
	return m_shape->TestPoint(m_body->GetTransform(), p);
}

inline bool cb2Fixture::RayCast(cb2RayCastOutput* output, const cb2RayCastInput& input, int childIndex) const
{
	return m_shape->RayCast(output, input, m_body->GetTransform(), childIndex);
}

inline void cb2Fixture::GetMassData(cb2MassData* massData) const
{
	m_shape->ComputeMass(massData, m_density);
}

inline const cb2AABB& cb2Fixture::GetAABB(int childIndex) const
{
	cb2Assert(0 <= childIndex && childIndex < m_proxyCount);
	return m_proxies[childIndex].aabb;
}

#endif
