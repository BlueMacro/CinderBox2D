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

#include <CinderBox2D/Common/cb2BlockAllocator.h>
#include <limits.h>
#include <memory.h>
#include <stddef.h>

int cb2BlockAllocator::s_blockSizes[cb2_blockSizes] = 
{
	16,		// 0
	32,		// 1
	64,		// 2
	96,		// 3
	128,	// 4
	160,	// 5
	192,	// 6
	224,	// 7
	256,	// 8
	320,	// 9
	384,	// 10
	448,	// 11
	512,	// 12
	640,	// 13
};
unsigned char cb2BlockAllocator::s_blockSizeLookup[cb2_maxBlockSize + 1];
bool cb2BlockAllocator::s_blockSizeLookupInitialized;

struct cb2Chunk
{
	int blockSize;
	cb2Block* blocks;
};

struct cb2Block
{
	cb2Block* next;
};

cb2BlockAllocator::cb2BlockAllocator()
{
	cb2Assert(cb2_blockSizes < UCHAR_MAX);

	m_chunkSpace = cb2_chunkArrayIncrement;
	m_chunkCount = 0;
	m_chunks = (cb2Chunk*)cb2Alloc(m_chunkSpace * sizeof(cb2Chunk));
	
	memset(m_chunks, 0, m_chunkSpace * sizeof(cb2Chunk));
	memset(m_freeLists, 0, sizeof(m_freeLists));

	if (s_blockSizeLookupInitialized == false)
	{
		int j = 0;
		for (int i = 1; i <= cb2_maxBlockSize; ++i)
		{
			cb2Assert(j < cb2_blockSizes);
			if (i <= s_blockSizes[j])
			{
				s_blockSizeLookup[i] = (unsigned char)j;
			}
			else
			{
				++j;
				s_blockSizeLookup[i] = (unsigned char)j;
			}
		}

		s_blockSizeLookupInitialized = true;
	}
}

cb2BlockAllocator::~cb2BlockAllocator()
{
	for (int i = 0; i < m_chunkCount; ++i)
	{
		cb2Free(m_chunks[i].blocks);
	}

	cb2Free(m_chunks);
}

void* cb2BlockAllocator::Allocate(int size)
{
	if (size == 0)
		return NULL;

	cb2Assert(0 < size);

	if (size > cb2_maxBlockSize)
	{
		return cb2Alloc(size);
	}

	int index = s_blockSizeLookup[size];
	cb2Assert(0 <= index && index < cb2_blockSizes);

	if (m_freeLists[index])
	{
		cb2Block* block = m_freeLists[index];
		m_freeLists[index] = block->next;
		return block;
	}
	else
	{
		if (m_chunkCount == m_chunkSpace)
		{
			cb2Chunk* oldChunks = m_chunks;
			m_chunkSpace += cb2_chunkArrayIncrement;
			m_chunks = (cb2Chunk*)cb2Alloc(m_chunkSpace * sizeof(cb2Chunk));
			memcpy(m_chunks, oldChunks, m_chunkCount * sizeof(cb2Chunk));
			memset(m_chunks + m_chunkCount, 0, cb2_chunkArrayIncrement * sizeof(cb2Chunk));
			cb2Free(oldChunks);
		}

		cb2Chunk* chunk = m_chunks + m_chunkCount;
		chunk->blocks = (cb2Block*)cb2Alloc(cb2_chunkSize);
#if defined(_DEBUG)
		memset(chunk->blocks, 0xcd, cb2_chunkSize);
#endif
		int blockSize = s_blockSizes[index];
		chunk->blockSize = blockSize;
		int blockCount = cb2_chunkSize / blockSize;
		cb2Assert(blockCount * blockSize <= cb2_chunkSize);
		for (int i = 0; i < blockCount - 1; ++i)
		{
			cb2Block* block = (cb2Block*)((char*)chunk->blocks + blockSize * i);
			cb2Block* next = (cb2Block*)((char*)chunk->blocks + blockSize * (i + 1));
			block->next = next;
		}
		cb2Block* last = (cb2Block*)((char*)chunk->blocks + blockSize * (blockCount - 1));
		last->next = NULL;

		m_freeLists[index] = chunk->blocks->next;
		++m_chunkCount;

		return chunk->blocks;
	}
}

void cb2BlockAllocator::Free(void* p, int size)
{
	if (size == 0)
	{
		return;
	}

	cb2Assert(0 < size);

	if (size > cb2_maxBlockSize)
	{
		cb2Free(p);
		return;
	}

	int index = s_blockSizeLookup[size];
	cb2Assert(0 <= index && index < cb2_blockSizes);

#ifdef _DEBUG
	// Verify the memory address and size is valid.
	int blockSize = s_blockSizes[index];
	bool found = false;
	for (int i = 0; i < m_chunkCount; ++i)
	{
		cb2Chunk* chunk = m_chunks + i;
		if (chunk->blockSize != blockSize)
		{
			cb2Assert(	(char*)p + blockSize <= (char*)chunk->blocks ||
						(char*)chunk->blocks + cb2_chunkSize <= (char*)p);
		}
		else
		{
			if ((char*)chunk->blocks <= (char*)p && (char*)p + blockSize <= (char*)chunk->blocks + cb2_chunkSize)
			{
				found = true;
			}
		}
	}

	cb2Assert(found);

	memset(p, 0xfd, blockSize);
#endif

	cb2Block* block = (cb2Block*)p;
	block->next = m_freeLists[index];
	m_freeLists[index] = block;
}

void cb2BlockAllocator::Clear()
{
	for (int i = 0; i < m_chunkCount; ++i)
	{
		cb2Free(m_chunks[i].blocks);
	}

	m_chunkCount = 0;
	memset(m_chunks, 0, m_chunkSpace * sizeof(cb2Chunk));

	memset(m_freeLists, 0, sizeof(m_freeLists));
}
