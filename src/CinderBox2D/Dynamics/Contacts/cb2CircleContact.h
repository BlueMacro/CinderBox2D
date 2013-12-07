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

#ifndef CB2_CIRCLE_CONTACT_H
#define CB2_CIRCLE_CONTACT_H

#include <CinderBox2D/Dynamics/Contacts/cb2Contact.h>

class cb2BlockAllocator;

class cb2CircleContact : public cb2Contact
{
public:
	static cb2Contact* Create(	cb2Fixture* fixtureA, int indexA,
								cb2Fixture* fixtureB, int indexB, cb2BlockAllocator* allocator);
	static void Destroy(cb2Contact* contact, cb2BlockAllocator* allocator);

	cb2CircleContact(cb2Fixture* fixtureA, cb2Fixture* fixtureB);
	~cb2CircleContact() {}

	void Evaluate(cb2Manifold* manifold, const cb2Transform& xfA, const cb2Transform& xfB);
};

#endif
