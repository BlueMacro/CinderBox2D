/*
* Copyright (c) 2009 Erin Catto http://www.box2d.org
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

#include <CinderBox2D/Collision/cb2DynamicTree.h>
#include <memory.h>

cb2DynamicTree::cb2DynamicTree()
{
	m_root = cb2_nullNode;

	m_nodeCapacity = 16;
	m_nodeCount = 0;
	m_nodes = (cb2TreeNode*)cb2Alloc(m_nodeCapacity * sizeof(cb2TreeNode));
	memset(m_nodes, 0, m_nodeCapacity * sizeof(cb2TreeNode));

	// Build a linked list for the free list.
	for (int i = 0; i < m_nodeCapacity - 1; ++i)
	{
		m_nodes[i].next = i + 1;
		m_nodes[i].height = -1;
	}
	m_nodes[m_nodeCapacity-1].next = cb2_nullNode;
	m_nodes[m_nodeCapacity-1].height = -1;
	m_freeList = 0;

	m_path = 0;

	m_insertionCount = 0;
}

cb2DynamicTree::~cb2DynamicTree()
{
	// This frees the entire tree in one shot.
	cb2Free(m_nodes);
}

// Allocate a node from the pool. Grow the pool if necessary.
int cb2DynamicTree::AllocateNode()
{
	// Expand the node pool as needed.
	if (m_freeList == cb2_nullNode)
	{
		cb2Assert(m_nodeCount == m_nodeCapacity);

		// The free list is empty. Rebuild a bigger pool.
		cb2TreeNode* oldNodes = m_nodes;
		m_nodeCapacity *= 2;
		m_nodes = (cb2TreeNode*)cb2Alloc(m_nodeCapacity * sizeof(cb2TreeNode));
		memcpy(m_nodes, oldNodes, m_nodeCount * sizeof(cb2TreeNode));
		cb2Free(oldNodes);

		// Build a linked list for the free list. The parent
		// pointer becomes the "next" pointer.
		for (int i = m_nodeCount; i < m_nodeCapacity - 1; ++i)
		{
			m_nodes[i].next = i + 1;
			m_nodes[i].height = -1;
		}
		m_nodes[m_nodeCapacity-1].next = cb2_nullNode;
		m_nodes[m_nodeCapacity-1].height = -1;
		m_freeList = m_nodeCount;
	}

	// Peel a node off the free list.
	int nodeId = m_freeList;
	m_freeList = m_nodes[nodeId].next;
	m_nodes[nodeId].parent = cb2_nullNode;
	m_nodes[nodeId].child1 = cb2_nullNode;
	m_nodes[nodeId].child2 = cb2_nullNode;
	m_nodes[nodeId].height = 0;
	m_nodes[nodeId].userData = NULL;
	++m_nodeCount;
	return nodeId;
}

// Return a node to the pool.
void cb2DynamicTree::FreeNode(int nodeId)
{
	cb2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
	cb2Assert(0 < m_nodeCount);
	m_nodes[nodeId].next = m_freeList;
	m_nodes[nodeId].height = -1;
	m_freeList = nodeId;
	--m_nodeCount;
}

// Create a proxy in the tree as a leaf node. We return the index
// of the node instead of a pointer so that we can grow
// the node pool.
int cb2DynamicTree::CreateProxy(const cb2AABB& aabb, void* userData)
{
	int proxyId = AllocateNode();

	// Fatten the aabb.
	ci::Vec2f r(cb2_aabbExtension, cb2_aabbExtension);
	m_nodes[proxyId].aabb.lowerBound = aabb.lowerBound - r;
	m_nodes[proxyId].aabb.upperBound = aabb.upperBound + r;
	m_nodes[proxyId].userData = userData;
	m_nodes[proxyId].height = 0;

	InsertLeaf(proxyId);

	return proxyId;
}

void cb2DynamicTree::DestroyProxy(int proxyId)
{
	cb2Assert(0 <= proxyId && proxyId < m_nodeCapacity);
	cb2Assert(m_nodes[proxyId].IsLeaf());

	RemoveLeaf(proxyId);
	FreeNode(proxyId);
}

bool cb2DynamicTree::MoveProxy(int proxyId, const cb2AABB& aabb, const ci::Vec2f& displacement)
{
	cb2Assert(0 <= proxyId && proxyId < m_nodeCapacity);

	cb2Assert(m_nodes[proxyId].IsLeaf());

	if (m_nodes[proxyId].aabb.Contains(aabb))
	{
		return false;
	}

	RemoveLeaf(proxyId);

	// Extend AABB.
	cb2AABB b = aabb;
	ci::Vec2f r(cb2_aabbExtension, cb2_aabbExtension);
	b.lowerBound = b.lowerBound - r;
	b.upperBound = b.upperBound + r;

	// Predict AABB displacement.
	ci::Vec2f d = cb2_aabbMultiplier * displacement;

	if (d.x < 0.0f)
	{
		b.lowerBound.x += d.x;
	}
	else
	{
		b.upperBound.x += d.x;
	}

	if (d.y < 0.0f)
	{
		b.lowerBound.y += d.y;
	}
	else
	{
		b.upperBound.y += d.y;
	}

	m_nodes[proxyId].aabb = b;

	InsertLeaf(proxyId);
	return true;
}

void cb2DynamicTree::InsertLeaf(int leaf)
{
	++m_insertionCount;

	if (m_root == cb2_nullNode)
	{
		m_root = leaf;
		m_nodes[m_root].parent = cb2_nullNode;
		return;
	}

	// Find the best sibling for this node
	cb2AABB leafAABB = m_nodes[leaf].aabb;
	int index = m_root;
	while (m_nodes[index].IsLeaf() == false)
	{
		int child1 = m_nodes[index].child1;
		int child2 = m_nodes[index].child2;

		float area = m_nodes[index].aabb.GetPerimeter();

		cb2AABB combinedAABB;
		combinedAABB.Combine(m_nodes[index].aabb, leafAABB);
		float combinedArea = combinedAABB.GetPerimeter();

		// Cost of creating a new parent for this node and the new leaf
		float cost = 2.0f * combinedArea;

		// Minimum cost of pushing the leaf further down the tree
		float inheritanceCost = 2.0f * (combinedArea - area);

		// Cost of descending into child1
		float cost1;
		if (m_nodes[child1].IsLeaf())
		{
			cb2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child1].aabb);
			cost1 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			cb2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child1].aabb);
			float oldArea = m_nodes[child1].aabb.GetPerimeter();
			float newArea = aabb.GetPerimeter();
			cost1 = (newArea - oldArea) + inheritanceCost;
		}

		// Cost of descending into child2
		float cost2;
		if (m_nodes[child2].IsLeaf())
		{
			cb2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child2].aabb);
			cost2 = aabb.GetPerimeter() + inheritanceCost;
		}
		else
		{
			cb2AABB aabb;
			aabb.Combine(leafAABB, m_nodes[child2].aabb);
			float oldArea = m_nodes[child2].aabb.GetPerimeter();
			float newArea = aabb.GetPerimeter();
			cost2 = newArea - oldArea + inheritanceCost;
		}

		// Descend according to the minimum cost.
		if (cost < cost1 && cost < cost2)
		{
			break;
		}

		// Descend
		if (cost1 < cost2)
		{
			index = child1;
		}
		else
		{
			index = child2;
		}
	}

	int sibling = index;

	// Create a new parent.
	int oldParent = m_nodes[sibling].parent;
	int newParent = AllocateNode();
	m_nodes[newParent].parent = oldParent;
	m_nodes[newParent].userData = NULL;
	m_nodes[newParent].aabb.Combine(leafAABB, m_nodes[sibling].aabb);
	m_nodes[newParent].height = m_nodes[sibling].height + 1;

	if (oldParent != cb2_nullNode)
	{
		// The sibling was not the root.
		if (m_nodes[oldParent].child1 == sibling)
		{
			m_nodes[oldParent].child1 = newParent;
		}
		else
		{
			m_nodes[oldParent].child2 = newParent;
		}

		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
	}
	else
	{
		// The sibling was the root.
		m_nodes[newParent].child1 = sibling;
		m_nodes[newParent].child2 = leaf;
		m_nodes[sibling].parent = newParent;
		m_nodes[leaf].parent = newParent;
		m_root = newParent;
	}

	// Walk back up the tree fixing heights and AABBs
	index = m_nodes[leaf].parent;
	while (index != cb2_nullNode)
	{
		index = Balance(index);

		int child1 = m_nodes[index].child1;
		int child2 = m_nodes[index].child2;

		cb2Assert(child1 != cb2_nullNode);
		cb2Assert(child2 != cb2_nullNode);

		m_nodes[index].height = 1 + cb2Max(m_nodes[child1].height, m_nodes[child2].height);
		m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

		index = m_nodes[index].parent;
	}

	//Validate();
}

void cb2DynamicTree::RemoveLeaf(int leaf)
{
	if (leaf == m_root)
	{
		m_root = cb2_nullNode;
		return;
	}

	int parent = m_nodes[leaf].parent;
	int grandParent = m_nodes[parent].parent;
	int sibling;
	if (m_nodes[parent].child1 == leaf)
	{
		sibling = m_nodes[parent].child2;
	}
	else
	{
		sibling = m_nodes[parent].child1;
	}

	if (grandParent != cb2_nullNode)
	{
		// Destroy parent and connect sibling to grandParent.
		if (m_nodes[grandParent].child1 == parent)
		{
			m_nodes[grandParent].child1 = sibling;
		}
		else
		{
			m_nodes[grandParent].child2 = sibling;
		}
		m_nodes[sibling].parent = grandParent;
		FreeNode(parent);

		// Adjust ancestor bounds.
		int index = grandParent;
		while (index != cb2_nullNode)
		{
			index = Balance(index);

			int child1 = m_nodes[index].child1;
			int child2 = m_nodes[index].child2;

			m_nodes[index].aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);
			m_nodes[index].height = 1 + cb2Max(m_nodes[child1].height, m_nodes[child2].height);

			index = m_nodes[index].parent;
		}
	}
	else
	{
		m_root = sibling;
		m_nodes[sibling].parent = cb2_nullNode;
		FreeNode(parent);
	}

	//Validate();
}

// Perform a left or right rotation if node A is imbalanced.
// Returns the new root index.
int cb2DynamicTree::Balance(int iA)
{
	cb2Assert(iA != cb2_nullNode);

	cb2TreeNode* A = m_nodes + iA;
	if (A->IsLeaf() || A->height < 2)
	{
		return iA;
	}

	int iB = A->child1;
	int iC = A->child2;
	cb2Assert(0 <= iB && iB < m_nodeCapacity);
	cb2Assert(0 <= iC && iC < m_nodeCapacity);

	cb2TreeNode* B = m_nodes + iB;
	cb2TreeNode* C = m_nodes + iC;

	int balance = C->height - B->height;

	// Rotate C up
	if (balance > 1)
	{
		int iF = C->child1;
		int iG = C->child2;
		cb2TreeNode* F = m_nodes + iF;
		cb2TreeNode* G = m_nodes + iG;
		cb2Assert(0 <= iF && iF < m_nodeCapacity);
		cb2Assert(0 <= iG && iG < m_nodeCapacity);

		// Swap A and C
		C->child1 = iA;
		C->parent = A->parent;
		A->parent = iC;

		// A's old parent should point to C
		if (C->parent != cb2_nullNode)
		{
			if (m_nodes[C->parent].child1 == iA)
			{
				m_nodes[C->parent].child1 = iC;
			}
			else
			{
				cb2Assert(m_nodes[C->parent].child2 == iA);
				m_nodes[C->parent].child2 = iC;
			}
		}
		else
		{
			m_root = iC;
		}

		// Rotate
		if (F->height > G->height)
		{
			C->child2 = iF;
			A->child2 = iG;
			G->parent = iA;
			A->aabb.Combine(B->aabb, G->aabb);
			C->aabb.Combine(A->aabb, F->aabb);

			A->height = 1 + cb2Max(B->height, G->height);
			C->height = 1 + cb2Max(A->height, F->height);
		}
		else
		{
			C->child2 = iG;
			A->child2 = iF;
			F->parent = iA;
			A->aabb.Combine(B->aabb, F->aabb);
			C->aabb.Combine(A->aabb, G->aabb);

			A->height = 1 + cb2Max(B->height, F->height);
			C->height = 1 + cb2Max(A->height, G->height);
		}

		return iC;
	}
	
	// Rotate B up
	if (balance < -1)
	{
		int iD = B->child1;
		int iE = B->child2;
		cb2TreeNode* D = m_nodes + iD;
		cb2TreeNode* E = m_nodes + iE;
		cb2Assert(0 <= iD && iD < m_nodeCapacity);
		cb2Assert(0 <= iE && iE < m_nodeCapacity);

		// Swap A and B
		B->child1 = iA;
		B->parent = A->parent;
		A->parent = iB;

		// A's old parent should point to B
		if (B->parent != cb2_nullNode)
		{
			if (m_nodes[B->parent].child1 == iA)
			{
				m_nodes[B->parent].child1 = iB;
			}
			else
			{
				cb2Assert(m_nodes[B->parent].child2 == iA);
				m_nodes[B->parent].child2 = iB;
			}
		}
		else
		{
			m_root = iB;
		}

		// Rotate
		if (D->height > E->height)
		{
			B->child2 = iD;
			A->child1 = iE;
			E->parent = iA;
			A->aabb.Combine(C->aabb, E->aabb);
			B->aabb.Combine(A->aabb, D->aabb);

			A->height = 1 + cb2Max(C->height, E->height);
			B->height = 1 + cb2Max(A->height, D->height);
		}
		else
		{
			B->child2 = iE;
			A->child1 = iD;
			D->parent = iA;
			A->aabb.Combine(C->aabb, D->aabb);
			B->aabb.Combine(A->aabb, E->aabb);

			A->height = 1 + cb2Max(C->height, D->height);
			B->height = 1 + cb2Max(A->height, E->height);
		}

		return iB;
	}

	return iA;
}

int cb2DynamicTree::GetHeight() const
{
	if (m_root == cb2_nullNode)
	{
		return 0;
	}

	return m_nodes[m_root].height;
}

//
float cb2DynamicTree::GetAreaRatio() const
{
	if (m_root == cb2_nullNode)
	{
		return 0.0f;
	}

	const cb2TreeNode* root = m_nodes + m_root;
	float rootArea = root->aabb.GetPerimeter();

	float totalArea = 0.0f;
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		const cb2TreeNode* node = m_nodes + i;
		if (node->height < 0)
		{
			// Free node in pool
			continue;
		}

		totalArea += node->aabb.GetPerimeter();
	}

	return totalArea / rootArea;
}

// Compute the height of a sub-tree.
int cb2DynamicTree::ComputeHeight(int nodeId) const
{
	cb2Assert(0 <= nodeId && nodeId < m_nodeCapacity);
	cb2TreeNode* node = m_nodes + nodeId;

	if (node->IsLeaf())
	{
		return 0;
	}

	int height1 = ComputeHeight(node->child1);
	int height2 = ComputeHeight(node->child2);
	return 1 + cb2Max(height1, height2);
}

int cb2DynamicTree::ComputeHeight() const
{
	int height = ComputeHeight(m_root);
	return height;
}

void cb2DynamicTree::ValidateStructure(int index) const
{
	if (index == cb2_nullNode)
	{
		return;
	}

	if (index == m_root)
	{
		cb2Assert(m_nodes[index].parent == cb2_nullNode);
	}

	const cb2TreeNode* node = m_nodes + index;

	int child1 = node->child1;
	int child2 = node->child2;

	if (node->IsLeaf())
	{
		cb2Assert(child1 == cb2_nullNode);
		cb2Assert(child2 == cb2_nullNode);
		cb2Assert(node->height == 0);
		return;
	}

	cb2Assert(0 <= child1 && child1 < m_nodeCapacity);
	cb2Assert(0 <= child2 && child2 < m_nodeCapacity);

	cb2Assert(m_nodes[child1].parent == index);
	cb2Assert(m_nodes[child2].parent == index);

	ValidateStructure(child1);
	ValidateStructure(child2);
}

void cb2DynamicTree::ValidateMetrics(int index) const
{
	if (index == cb2_nullNode)
	{
		return;
	}

	const cb2TreeNode* node = m_nodes + index;

	int child1 = node->child1;
	int child2 = node->child2;

	if (node->IsLeaf())
	{
		cb2Assert(child1 == cb2_nullNode);
		cb2Assert(child2 == cb2_nullNode);
		cb2Assert(node->height == 0);
		return;
	}

	cb2Assert(0 <= child1 && child1 < m_nodeCapacity);
	cb2Assert(0 <= child2 && child2 < m_nodeCapacity);

	int height1 = m_nodes[child1].height;
	int height2 = m_nodes[child2].height;
	int height;
	height = 1 + cb2Max(height1, height2);
	cb2Assert(node->height == height);

	cb2AABB aabb;
	aabb.Combine(m_nodes[child1].aabb, m_nodes[child2].aabb);

	cb2Assert(aabb.lowerBound == node->aabb.lowerBound);
	cb2Assert(aabb.upperBound == node->aabb.upperBound);

	ValidateMetrics(child1);
	ValidateMetrics(child2);
}

void cb2DynamicTree::Validate() const
{
	ValidateStructure(m_root);
	ValidateMetrics(m_root);

	int freeCount = 0;
	int freeIndex = m_freeList;
	while (freeIndex != cb2_nullNode)
	{
		cb2Assert(0 <= freeIndex && freeIndex < m_nodeCapacity);
		freeIndex = m_nodes[freeIndex].next;
		++freeCount;
	}

	cb2Assert(GetHeight() == ComputeHeight());

	cb2Assert(m_nodeCount + freeCount == m_nodeCapacity);
}

int cb2DynamicTree::GetMaxBalance() const
{
	int maxBalance = 0;
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		const cb2TreeNode* node = m_nodes + i;
		if (node->height <= 1)
		{
			continue;
		}

		cb2Assert(node->IsLeaf() == false);

		int child1 = node->child1;
		int child2 = node->child2;
		int balance = cb2Abs(m_nodes[child2].height - m_nodes[child1].height);
		maxBalance = cb2Max(maxBalance, balance);
	}

	return maxBalance;
}

void cb2DynamicTree::RebuildBottomUp()
{
	int* nodes = (int*)cb2Alloc(m_nodeCount * sizeof(int));
	int count = 0;

	// Build array of leaves. Free the rest.
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		if (m_nodes[i].height < 0)
		{
			// free node in pool
			continue;
		}

		if (m_nodes[i].IsLeaf())
		{
			m_nodes[i].parent = cb2_nullNode;
			nodes[count] = i;
			++count;
		}
		else
		{
			FreeNode(i);
		}
	}

	while (count > 1)
	{
		float minCost = cb2_maxFloat;
		int iMin = -1, jMin = -1;
		for (int i = 0; i < count; ++i)
		{
			cb2AABB aabbi = m_nodes[nodes[i]].aabb;

			for (int j = i + 1; j < count; ++j)
			{
				cb2AABB aabbj = m_nodes[nodes[j]].aabb;
				cb2AABB b;
				b.Combine(aabbi, aabbj);
				float cost = b.GetPerimeter();
				if (cost < minCost)
				{
					iMin = i;
					jMin = j;
					minCost = cost;
				}
			}
		}

		int index1 = nodes[iMin];
		int index2 = nodes[jMin];
		cb2TreeNode* child1 = m_nodes + index1;
		cb2TreeNode* child2 = m_nodes + index2;

		int parentIndex = AllocateNode();
		cb2TreeNode* parent = m_nodes + parentIndex;
		parent->child1 = index1;
		parent->child2 = index2;
		parent->height = 1 + cb2Max(child1->height, child2->height);
		parent->aabb.Combine(child1->aabb, child2->aabb);
		parent->parent = cb2_nullNode;

		child1->parent = parentIndex;
		child2->parent = parentIndex;

		nodes[jMin] = nodes[count-1];
		nodes[iMin] = parentIndex;
		--count;
	}

	m_root = nodes[0];
	cb2Free(nodes);

	Validate();
}

void cb2DynamicTree::ShiftOrigin(const ci::Vec2f& newOrigin)
{
	// Build array of leaves. Free the rest.
	for (int i = 0; i < m_nodeCapacity; ++i)
	{
		m_nodes[i].aabb.lowerBound -= newOrigin;
		m_nodes[i].aabb.upperBound -= newOrigin;
	}
}
