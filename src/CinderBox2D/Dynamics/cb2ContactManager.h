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

#ifndef CB2_CONTACT_MANAGER_H
#define CB2_CONTACT_MANAGER_H

#include <CinderBox2D/Collision/cb2BroadPhase.h>

class cb2Contact;
class cb2ContactFilter;
class cb2ContactListener;
class cb2BlockAllocator;

// Delegate of cb2World.
class cb2ContactManager
{
public:
	cb2ContactManager();

	// Broad-phase callback.
	void AddPair(void* proxyUserDataA, void* proxyUserDataB);

	void FindNewContacts();

	void Destroy(cb2Contact* c);

	void Collide();
            
	cb2BroadPhase m_broadPhase;
	cb2Contact* m_contactList;
	int m_contactCount;
	cb2ContactFilter* m_contactFilter;
	cb2ContactListener* m_contactListener;
	cb2BlockAllocator* m_allocator;
};

#endif
