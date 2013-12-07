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

#include <CinderBox2D/Collision/cb2BroadPhase.h>

cb2BroadPhase::cb2BroadPhase()
{
	m_proxyCount = 0;

	m_pairCapacity = 16;
	m_pairCount = 0;
	m_pairBuffer = (cb2Pair*)cb2Alloc(m_pairCapacity * sizeof(cb2Pair));

	m_moveCapacity = 16;
	m_moveCount = 0;
	m_moveBuffer = (int*)cb2Alloc(m_moveCapacity * sizeof(int));
}

cb2BroadPhase::~cb2BroadPhase()
{
	cb2Free(m_moveBuffer);
	cb2Free(m_pairBuffer);
}

int cb2BroadPhase::CreateProxy(const cb2AABB& aabb, void* userData)
{
	int proxyId = m_tree.CreateProxy(aabb, userData);
	++m_proxyCount;
	BufferMove(proxyId);
	return proxyId;
}

void cb2BroadPhase::DestroyProxy(int proxyId)
{
	UnBufferMove(proxyId);
	--m_proxyCount;
	m_tree.DestroyProxy(proxyId);
}

void cb2BroadPhase::MoveProxy(int proxyId, const cb2AABB& aabb, const ci::Vec2f& displacement)
{
	bool buffer = m_tree.MoveProxy(proxyId, aabb, displacement);
	if (buffer)
	{
		BufferMove(proxyId);
	}
}

void cb2BroadPhase::TouchProxy(int proxyId)
{
	BufferMove(proxyId);
}

void cb2BroadPhase::BufferMove(int proxyId)
{
	if (m_moveCount == m_moveCapacity)
	{
		int* oldBuffer = m_moveBuffer;
		m_moveCapacity *= 2;
		m_moveBuffer = (int*)cb2Alloc(m_moveCapacity * sizeof(int));
		memcpy(m_moveBuffer, oldBuffer, m_moveCount * sizeof(int));
		cb2Free(oldBuffer);
	}

	m_moveBuffer[m_moveCount] = proxyId;
	++m_moveCount;
}

void cb2BroadPhase::UnBufferMove(int proxyId)
{
	for (int i = 0; i < m_moveCount; ++i)
	{
		if (m_moveBuffer[i] == proxyId)
		{
			m_moveBuffer[i] = e_nullProxy;
		}
	}
}

// This is called from cb2DynamicTree::Query when we are gathering pairs.
bool cb2BroadPhase::QueryCallback(int proxyId)
{
	// A proxy cannot form a pair with itself.
	if (proxyId == m_queryProxyId)
	{
		return true;
	}

	// Grow the pair buffer as needed.
	if (m_pairCount == m_pairCapacity)
	{
		cb2Pair* oldBuffer = m_pairBuffer;
		m_pairCapacity *= 2;
		m_pairBuffer = (cb2Pair*)cb2Alloc(m_pairCapacity * sizeof(cb2Pair));
		memcpy(m_pairBuffer, oldBuffer, m_pairCount * sizeof(cb2Pair));
		cb2Free(oldBuffer);
	}

	m_pairBuffer[m_pairCount].proxyIdA = cb2Min(proxyId, m_queryProxyId);
	m_pairBuffer[m_pairCount].proxyIdB = cb2Max(proxyId, m_queryProxyId);
	++m_pairCount;

	return true;
}
