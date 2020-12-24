/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_ALIGNED_ALLOCATOR
#define BT_ALIGNED_ALLOCATOR

// we probably replace this with our own aligned memory allocator
// so we replace _aligned_malloc and _aligned_free with our own
// that is better portable and more predictable

#include "btDefines.h"

#if defined(BT_DEBUG) && defined(_MSC_VER)
#define BT_DEBUG_MEMORY_ALLOCATIONS
#endif

#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
// Debug versions of the aligned allocator, for CRT debugging, etc.
void *btDbgAlignedAllocInternal(size_t size, int alignment, int blockType, const char *fileName, int line);
void btDbgAlignedFreeInternal(void *ptr, int blockType);

// Non-aligned allocations
void *btDbgAllocInternal(size_t size, int blockType, const char *fileName, int line);
void btDbgFreeInternal(void *ptr, int blockType);

#define _NORMAL_BLOCK 1

#define btAlignedAlloc(size, alignment) btDbgAlignedAllocInternal(size, alignment, _NORMAL_BLOCK, __FILE__, __LINE__)
#define btAlignedFree(ptr) btDbgAlignedFreeInternal(ptr, _NORMAL_BLOCK)

#define btAlloc(size) btDbgAllocInternal(size, _NORMAL_BLOCK, __FILE__, __LINE__)
#define btFree(ptr) btDbgFreeInternal(ptr, _NORMAL_BLOCK)

// Don't use these macros unless you have a good reason to!
#define btDbgAlignedAlloc(size, alignment, blocktype, filename, line) btDbgAlignedAllocInternal(size, alignment, blocktype, filename, line)
#define btDbgAlignedFree(ptr, blocktype) btDbgAlignedFreeInternal(ptr, blocktype)

typedef int size_type;

typedef void *(btDbgAlignedAllocFunc)(size_t size, int alignment, int blockType, const char *fileName, int line);
typedef void(btDbgAlignedFreeFunc)(void *memblock, int blockType);
typedef void *(btDbgAllocFunc)(size_t size, int blockType, const char *fileName, int line);
typedef void(btDbgFreeFunc)(void *memblock, int blockType);

void btDbgAllocSetCustom(btDbgAllocFunc *allocFunc, btDbgFreeFunc *freeFunc);
void btDbgAlignedAllocSetCustom(btDbgAlignedAllocFunc *allocFunc, btDbgAlignedFreeFunc *freeFunc);
#else
void *btAlignedAllocInternal(size_t size, int alignment);
void btAlignedFreeInternal(void *ptr);

// Non-aligned allocations
void *btAllocInternal(size_t size);
void btFreeInternal(void *ptr);

#define btAlignedAlloc(size, alignment) btAlignedAllocInternal(size, alignment)
#define btAlignedFree(ptr) btAlignedFreeInternal(ptr)

#define btAlloc(size) btAllocInternal(size)
#define btFree(ptr) btFreeInternal(ptr)

// Don't use these macros unless you have a good reason to!
#define btDbgAlignedAlloc(size, alignment, blocktype, filename, line) btAlignedAlloc(size, alignment)
#define btDbgAlignedFree(ptr, blocktype) btAlignedFree(ptr)

typedef int size_type;

typedef void *(btAlignedAllocFunc)(size_t size, int alignment);
typedef void(btAlignedFreeFunc)(void *memblock);
typedef void *(btAllocFunc)(size_t size);
typedef void(btFreeFunc)(void *memblock);

// The developer can let all Bullet memory allocations go through a custom memory allocator, using btAlignedAllocSetCustom
// NULL to revert to the defaults.
void btAlignedAllocSetCustom(btAllocFunc *allocFunc, btFreeFunc *freeFunc);
// If the developer has already an custom aligned allocator, then btAlignedAllocSetCustomAligned can be used. The default aligned allocator pre-allocates extra memory using the non-aligned allocator, and instruments it.
void btAlignedAllocSetCustomAligned(btAlignedAllocFunc *allocFunc, btAlignedFreeFunc *freeFunc);
#endif

// The btAlignedAllocator is a portable class for aligned memory allocations.
// Default implementations for unaligned and aligned allocations can be overridden by a custom allocator using btAlignedAllocSetCustom and btAlignedAllocSetCustomAligned.
template <typename T, unsigned Alignment>
class btAlignedAllocator
{
	typedef btAlignedAllocator<T, Alignment> self_type;

public:
	btAlignedAllocator() {}

	template <typename Other>
	btAlignedAllocator(const btAlignedAllocator<Other, Alignment> &)
	{
	}

	typedef const T *const_pointer;
	typedef const T &const_reference;
	typedef T *pointer;
	typedef T &reference;
	typedef T value_type;

	pointer address(reference ref) const { return &ref; }
	const_pointer address(const_reference ref) const { return &ref; }

	pointer allocate(size_type n, const_pointer *hint = 0)
	{
		(void)hint;
		return reinterpret_cast<pointer>(btAlignedAlloc(sizeof(value_type) * n, Alignment));
	}

	void construct(pointer ptr, const value_type &value)
	{
		new (ptr) value_type(value);
	}

	void deallocate(pointer ptr)
	{
		btAlignedFree(reinterpret_cast<void *>(ptr));
	}

	void destroy(pointer ptr)
	{
		ptr->~value_type();
	}

	template <typename O>
	struct rebind
	{
		typedef btAlignedAllocator<O, Alignment> other;
	};

	template <typename O>
	self_type &operator=(const btAlignedAllocator<O, Alignment> &)
	{
		return *this;
	}

	friend bool operator==(const self_type &, const self_type &) { return true; }
};

#endif  //BT_ALIGNED_ALLOCATOR