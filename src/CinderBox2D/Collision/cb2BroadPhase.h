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

#ifndef CB2_BROAD_PHASE_H
#define CB2_BROAD_PHASE_H

#include <CinderBox2D/Common/cb2Settings.h>
#include <CinderBox2D/Collision/cb2Collision.h>
#include <CinderBox2D/Collision/cb2DynamicTree.h>
#include <algorithm>

struct cb2Pair
{
	int proxyIdA;
	int proxyIdB;
	int next;
};

/// The broad-phase is used for computing pairs and performing volume queries and ray casts.
/// This broad-phase does not persist pairs. Instead, this reports potentially new pairs.
/// It is up to the client to consume the new pairs and to track subsequent overlap.
class cb2BroadPhase
{
public:

	enum
	{
		e_nullProxy = -1
	};

	cb2BroadPhase();
	~cb2BroadPhase();

	/// Create a proxy with an initial AABB. Pairs are not reported until
	/// UpdatePairs is called.
	int CreateProxy(const cb2AABB& aabb, void* userData);

	/// Destroy a proxy. It is up to the client to remove any pairs.
	void DestroyProxy(int proxyId);

	/// Call MoveProxy as many times as you like, then when you are done
	/// call UpdatePairs to finalized the proxy pairs (for your time step).
	void MoveProxy(int proxyId, const cb2AABB& aabb, const ci::Vec2f& displacement);

	/// Call to trigger a re-processing of it's pairs on the next call to UpdatePairs.
	void TouchProxy(int proxyId);

	/// Get the fat AABB for a proxy.
	const cb2AABB& GetFatAABB(int proxyId) const;

	/// Get user data from a proxy. Returns NULL if the id is invalid.
	void* GetUserData(int proxyId) const;

	/// Test overlap of fat AABBs.
	bool TestOverlap(int proxyIdA, int proxyIdB) const;

	/// Get the number of proxies.
	int GetProxyCount() const;

	/// Update the pairs. This results in pair callbacks. This can only add pairs.
	template <typename T>
	void UpdatePairs(T* callback);

	/// Query an AABB for overlapping proxies. The callback class
	/// is called for each proxy that overlaps the supplied AABB.
	template <typename T>
	void Query(T* callback, const cb2AABB& aabb) const;

	/// Ray-cast against the proxies in the tree. This relies on the callback
	/// to perform a exact ray-cast in the case were the proxy contains a shape.
	/// The callback also performs the any collision filtering. This has performance
	/// roughly equal to k * log(n), where k is the number of collisions and n is the
	/// number of proxies in the tree.
	/// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
	/// @param callback a callback class that is called for each proxy that is hit by the ray.
	template <typename T>
	void RayCast(T* callback, const cb2RayCastInput& input) const;

	/// Get the height of the embedded tree.
	int GetTreeHeight() const;

	/// Get the balance of the embedded tree.
	int GetTreeBalance() const;

	/// Get the quality metric of the embedded tree.
	float GetTreeQuality() const;

	/// Shift the world origin. Useful for large worlds.
	/// The shift formula is: position -= newOrigin
	/// @param newOrigin the new origin with respect to the old origin
	void ShiftOrigin(const ci::Vec2f& newOrigin);

private:

	friend class cb2DynamicTree;

	void BufferMove(int proxyId);
	void UnBufferMove(int proxyId);

	bool QueryCallback(int proxyId);

	cb2DynamicTree m_tree;

	int m_proxyCount;

	int* m_moveBuffer;
	int m_moveCapacity;
	int m_moveCount;

	cb2Pair* m_pairBuffer;
	int m_pairCapacity;
	int m_pairCount;

	int m_queryProxyId;
};

/// This is used to sort pairs.
inline bool cb2PairLessThan(const cb2Pair& pair1, const cb2Pair& pair2)
{
	if (pair1.proxyIdA < pair2.proxyIdA)
	{
		return true;
	}

	if (pair1.proxyIdA == pair2.proxyIdA)
	{
		return pair1.proxyIdB < pair2.proxyIdB;
	}

	return false;
}

inline void* cb2BroadPhase::GetUserData(int proxyId) const
{
	return m_tree.GetUserData(proxyId);
}

inline bool cb2BroadPhase::TestOverlap(int proxyIdA, int proxyIdB) const
{
	const cb2AABB& aabbA = m_tree.GetFatAABB(proxyIdA);
	const cb2AABB& aabbB = m_tree.GetFatAABB(proxyIdB);
	return cb2TestOverlap(aabbA, aabbB);
}

inline const cb2AABB& cb2BroadPhase::GetFatAABB(int proxyId) const
{
	return m_tree.GetFatAABB(proxyId);
}

inline int cb2BroadPhase::GetProxyCount() const
{
	return m_proxyCount;
}

inline int cb2BroadPhase::GetTreeHeight() const
{
	return m_tree.GetHeight();
}

inline int cb2BroadPhase::GetTreeBalance() const
{
	return m_tree.GetMaxBalance();
}

inline float cb2BroadPhase::GetTreeQuality() const
{
	return m_tree.GetAreaRatio();
}

template <typename T>
void cb2BroadPhase::UpdatePairs(T* callback)
{
	// Reset pair buffer
	m_pairCount = 0;

	// Perform tree queries for all moving proxies.
	for (int i = 0; i < m_moveCount; ++i)
	{
		m_queryProxyId = m_moveBuffer[i];
		if (m_queryProxyId == e_nullProxy)
		{
			continue;
		}

		// We have to query the tree with the fat AABB so that
		// we don't fail to create a pair that may touch later.
		const cb2AABB& fatAABB = m_tree.GetFatAABB(m_queryProxyId);

		// Query tree, create pairs and add them pair buffer.
		m_tree.Query(this, fatAABB);
	}

	// Reset move buffer
	m_moveCount = 0;

	// Sort the pair buffer to expose duplicates.
	std::sort(m_pairBuffer, m_pairBuffer + m_pairCount, cb2PairLessThan);

	// Send the pairs back to the client.
	int i = 0;
	while (i < m_pairCount)
	{
		cb2Pair* primaryPair = m_pairBuffer + i;
		void* userDataA = m_tree.GetUserData(primaryPair->proxyIdA);
		void* userDataB = m_tree.GetUserData(primaryPair->proxyIdB);

		callback->AddPair(userDataA, userDataB);
		++i;

		// Skip any duplicate pairs.
		while (i < m_pairCount)
		{
			cb2Pair* pair = m_pairBuffer + i;
			if (pair->proxyIdA != primaryPair->proxyIdA || pair->proxyIdB != primaryPair->proxyIdB)
			{
				break;
			}
			++i;
		}
	}

	// Try to keep the tree balanced.
	//m_tree.Rebalance(4);
}

template <typename T>
inline void cb2BroadPhase::Query(T* callback, const cb2AABB& aabb) const
{
	m_tree.Query(callback, aabb);
}

template <typename T>
inline void cb2BroadPhase::RayCast(T* callback, const cb2RayCastInput& input) const
{
	m_tree.RayCast(callback, input);
}

inline void cb2BroadPhase::ShiftOrigin(const ci::Vec2f& newOrigin)
{
	m_tree.ShiftOrigin(newOrigin);
}

#endif
