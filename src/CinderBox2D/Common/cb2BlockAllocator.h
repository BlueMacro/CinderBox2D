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

#ifndef CB2_BLOCK_ALLOCATOR_H
#define CB2_BLOCK_ALLOCATOR_H

#include <CinderBox2D/Common/cb2Settings.h>

const int cb2_chunkSize = 16 * 1024;
const int cb2_maxBlockSize = 640;
const int cb2_blockSizes = 14;
const int cb2_chunkArrayIncrement = 128;

struct cb2Block;
struct cb2Chunk;

/// This is a small object allocator used for allocating small
/// objects that persist for more than one time step.
/// See: http://www.codeproject.com/useritems/Small_Block_Allocator.asp
class cb2BlockAllocator
{
public:
	cb2BlockAllocator();
	~cb2BlockAllocator();

	/// Allocate memory. This will use cb2Alloc if the size is larger than cb2_maxBlockSize.
	void* Allocate(int size);

	/// Free memory. This will use cb2Free if the size is larger than cb2_maxBlockSize.
	void Free(void* p, int size);

	void Clear();

private:

	cb2Chunk* m_chunks;
	int m_chunkCount;
	int m_chunkSpace;

	cb2Block* m_freeLists[cb2_blockSizes];

	static int s_blockSizes[cb2_blockSizes];
	static unsigned char s_blockSizeLookup[cb2_maxBlockSize + 1];
	static bool s_blockSizeLookupInitialized;
};

#endif
