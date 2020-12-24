/*
Copyright (c) 2003-2009 Erwin Coumans  http://bullet.googlecode.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_DEFINES_H
#define BT_DEFINES_H

#ifdef BT_MANAGED_CODE
// Aligned data types not supported in managed code
#pragma unmanaged
#endif

#include <math.h>
#include <stdlib.h>  //size_t for MSVC 6.0
#include <float.h>

// For some debug print messages
#include "btDebug.h"

/* SVN $Revision$ on $Date$ from http://bullet.googlecode.com*/
#define BT_BULLET_VERSION 289

inline int btGetVersion()
{
	return BT_BULLET_VERSION;
}

inline int btIsDoublePrecision()
{
#ifdef BT_USE_DOUBLE_PRECISION
	return true;
#else
	return false;
#endif
}

// The following macro "BT_NOT_EMPTY_FILE" can be put into a file
// in order suppress the MS Visual C++ Linker warning 4221
//
// warning LNK4221: no public symbols found; archive member will be inaccessible
//
// This warning occurs on PC and XBOX when a file compiles out completely
// has no externally visible symbols which may be dependant on configuration
// #defines and options.
//
// see more https://stackoverflow.com/questions/1822887/what-is-the-best-way-to-eliminate-ms-visual-c-linker-warning-warning-lnk422

#if defined(_MSC_VER)
#define BT_NOT_EMPTY_FILE_CAT_II(p, res) res
#define BT_NOT_EMPTY_FILE_CAT_I(a, b) BT_NOT_EMPTY_FILE_CAT_II(~, a##b)
#define BT_NOT_EMPTY_FILE_CAT(a, b) BT_NOT_EMPTY_FILE_CAT_I(a, b)
#define BT_NOT_EMPTY_FILE                                      \
	namespace                                                  \
	{                                                          \
	char BT_NOT_EMPTY_FILE_CAT(NoEmptyFileDummy, __COUNTER__); \
	}
#else
#define BT_NOT_EMPTY_FILE
#endif

#if defined(DEBUG) || defined(_DEBUG)
#define BT_DEBUG
#endif

#if defined(RELEASE) || defined(_RELEASE) || defined(NDEBUG)
#define BT_RELEASE
#endif

#ifdef _WIN32

#if defined(__GNUC__)  // it should handle both MINGW and CYGWIN
#define SIMD_FORCE_INLINE __inline__ __attribute__((always_inline))
#define ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
#define ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
#define ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))
#elif (defined(_MSC_VER) && _MSC_VER < 1300)
#define SIMD_FORCE_INLINE inline
#define ATTRIBUTE_ALIGNED16(a) a
#define ATTRIBUTE_ALIGNED64(a) a
#define ATTRIBUTE_ALIGNED128(a) a
#elif defined(_M_ARM)
#define SIMD_FORCE_INLINE __forceinline
#define ATTRIBUTE_ALIGNED16(a) __declspec() a
#define ATTRIBUTE_ALIGNED64(a) __declspec() a
#define ATTRIBUTE_ALIGNED128(a) __declspec() a
#else
#define BT_HAS_ALIGNED_ALLOCATOR
#pragma warning(disable : 4324)  // disable padding warning
//		#pragma warning(disable:4530) // Disable the exception disable but used in MSCV Stl warning.
#pragma warning(disable : 4996)  //Turn off warnings about deprecated C routines
//		#pragma warning(disable:4786) // Disable the "debug name too long" warning

#define SIMD_FORCE_INLINE __forceinline
#define ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
#define ATTRIBUTE_ALIGNED64(a) __declspec(align(64)) a
#define ATTRIBUTE_ALIGNED128(a) __declspec(align(128)) a
#ifdef _XBOX
#define BT_USE_VMX128

#include <ppcintrinsics.h>
#define BT_HAVE_NATIVE_FSEL
#define btFsel(a, b, c) __fsel((a), (b), (c))
#else

#if defined(_M_ARM)
//Do not turn SSE on for ARM (may want to turn on BT_USE_NEON however)
#elif (defined(_WIN32) && (_MSC_VER) && _MSC_VER >= 1400) && (!defined(BT_USE_DOUBLE_PRECISION))
#ifdef __clang__
#define __BT_DISABLE_SSE__
#endif
#ifndef __BT_DISABLE_SSE__
#if _MSC_VER > 1400
#define BT_USE_SIMD_VECTOR3
#endif

#define BT_USE_SSE
#endif  //__BT_DISABLE_SSE__
#ifdef BT_USE_SSE

#if (_MSC_FULL_VER >= 170050727)  //Visual Studio 2012 can compile SSE4/FMA3 (but SSE4/FMA3 is not enabled by default)
#define BT_ALLOW_SSE4
#endif  //(_MSC_FULL_VER >= 160040219)

// BT_USE_SSE_IN_API is disabled under Windows by default, because
// it makes it harder to integrate Bullet into your application under Windows
// (structures embedding Bullet structs/classes need to be 16-byte aligned)
// with relatively little performance gain
// If you are not embedding Bullet data in your classes, or you make sure that you align those classes on 16-byte boundaries
// you can manually enable this line or set it in the build system for a bit of performance gain (a few percent, dependent on usage)
// #define BT_USE_SSE_IN_API
#endif  //BT_USE_SSE
#include <emmintrin.h>
#endif

#endif  //_XBOX

#endif  //__MINGW32__

#ifdef BT_DEBUG
#ifdef _MSC_VER
#include <stdio.h>
#define btAssert(x)                                                      \
	{                                                                    \
		if (!(x))                                                        \
		{                                                                \
			btDbgWarning("Assert %s:%u (%s)\n", __FILE__, __LINE__, #x); \
			__debugbreak();                                              \
		}                                                                \
	}
#define btAssertMsg(x, str)                                                      \
	{                                                                            \
		if (!(x))                                                                \
		{                                                                        \
			btDbgWarning("Assert %s:%u (%s) %s\n", __FILE__, __LINE__, #x, str); \
			__debugbreak();                                                      \
		}                                                                        \
	}
#else  //_MSC_VER
#include <assert.h>
#define btAssert assert
#define btAssertMsg(x, str) assert(x)
#endif  //_MSC_VER
#else
#define btAssert(x)
#define btAssertMsg(x, str)
#endif

//btFullAssert is optional, slows down a lot
#define btFullAssert(x)

#define btLikely(_c) _c
#define btUnlikely(_c) _c

#else  // _WIN32

#if defined(__CELLOS_LV2__)
#define SIMD_FORCE_INLINE inline __attribute__((always_inline))
#define ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
#define ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
#define ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))
#ifndef assert
#include <assert.h>
#endif

#ifdef BT_DEBUG
#ifdef __SPU__
#define btAssert(x)                                                      \
	{                                                                    \
		if (!(x))                                                        \
		{                                                                \
			btDbgWarning("Assert %s:%u (%s)\n", __FILE__, __LINE__, #x); \
			spu_hcmpeq(0, 0);                                            \
		}                                                                \
	}
#define btAssertMsg(x, str)                                                      \
	{                                                                            \
		if (!(x))                                                                \
		{                                                                        \
			btDbgWarning("Assert %s:%u (%s) %s\n", __FILE__, __LINE__, #x, str); \
			spu_hcmpeq(0, 0);                                                    \
		}                                                                        \
	}
#else
#define btAssert assert
#define btAssertMsg(x, str) assert(x)
#endif
#else
#define btAssert(x)
#define btAssertMsg(x, str)
#endif

//btFullAssert is optional, slows down a lot
#define btFullAssert(x)

#define btLikely(_c) _c
#define btUnlikely(_c) _c

#else  // __CELLOS_LV2__

#ifdef USE_LIBSPE2

#define SIMD_FORCE_INLINE __inline
#define ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
#define ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
#define ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))

#ifndef assert
#include <assert.h>
#endif

#ifdef BT_DEBUG
#define btAssert assert
#else
#define btAssert(x)
#endif
//btFullAssert is optional, slows down a lot
#define btFullAssert(x)

#define btLikely(_c) __builtin_expect((_c), 1)
#define btUnlikely(_c) __builtin_expect((_c), 0)

#else
//non-windows systems

#if (defined(__APPLE__) && (!defined(BT_USE_DOUBLE_PRECISION)))
#if defined(__i386__) || defined(__x86_64__)
#define BT_USE_SIMD_VECTOR3
#define BT_USE_SSE
//BT_USE_SSE_IN_API is enabled on Mac OSX by default, because memory is automatically aligned on 16-byte boundaries
//if apps run into issues, we will disable the next line
#define BT_USE_SSE_IN_API
#ifdef BT_USE_SSE
// include appropriate SSE level
#if defined(__SSE4_1__)
#include <smmintrin.h>
#elif defined(__SSSE3__)
#include <tmmintrin.h>
#elif defined(__SSE3__)
#include <pmmintrin.h>
#else
#include <emmintrin.h>
#endif
#endif  //BT_USE_SSE
#elif defined(__ARM_NEON__)
#ifdef __clang__
#define BT_USE_NEON 1
#define BT_USE_SIMD_VECTOR3

#if defined BT_USE_NEON && defined(__clang__)
#include <arm_neon.h>
#endif  //BT_USE_NEON
#endif  //__clang__
#endif  //__arm__

#define SIMD_FORCE_INLINE inline __attribute__((always_inline))
/// todo: check out alignment methods for other platforms/compilers
#define ATTRIBUTE_ALIGNED16(a) a __attribute__((aligned(16)))
#define ATTRIBUTE_ALIGNED64(a) a __attribute__((aligned(64)))
#define ATTRIBUTE_ALIGNED128(a) a __attribute__((aligned(128)))
#ifndef assert
#include <assert.h>
#endif

#if defined(DEBUG) || defined(_DEBUG)
#if defined(__i386__) || defined(__x86_64__)
#include <stdio.h>
#define btAssert(x)                                                      \
	{                                                                    \
		if (!(x))                                                        \
		{                                                                \
			btDbgWarning("Assert %s:%u (%s)\n", __FILE__, __LINE__, #x); \
			asm volatile("int3");                                        \
		}                                                                \
	}
#define btAssertMsg(x, str)                                                      \
	{                                                                            \
		if (!(x))                                                                \
		{                                                                        \
			btDbgWarning("Assert %s:%u (%s) %s\n", __FILE__, __LINE__, #x, str); \
			asm volatile("int3");                                                \
		}                                                                        \
	}
#else  //defined (__i386__) || defined (__x86_64__)
#define btAssert assert
#define btAssertMsg(x, str) assert(x)
#endif  //defined (__i386__) || defined (__x86_64__)
#else   //defined(DEBUG) || defined (_DEBUG)
#define btAssert(x)
#define btAssertMsg(x, str)
#endif  //defined(DEBUG) || defined (_DEBUG)

//btFullAssert is optional, slows down a lot
#define btFullAssert(x)
#define btLikely(_c) _c
#define btUnlikely(_c) _c

#else
// Linux
#define BT_USE_SIMD_VECTOR3
#define BT_USE_SSE
#define BT_HAS_ALIGNED_ALLOCATOR

// Disabled because linux does not allocate memory on 16-byte boundaries by default.
//#define BT_USE_SSE_IN_API

#ifdef BT_USE_SSE
// include appropriate SSE level
#if defined(__SSE4_1__)
#include <smmintrin.h>
#elif defined(__SSSE3__)
#include <tmmintrin.h>
#elif defined(__SSE3__)
#include <pmmintrin.h>
#else
#include <emmintrin.h>
#endif
#endif  //BT_USE_SSE

#define SIMD_FORCE_INLINE inline
/// todo: check out alignment methods for other platforms/compilers
/// define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
/// define ATTRIBUTE_ALIGNED64(a) a __attribute__ ((aligned (64)))
/// define ATTRIBUTE_ALIGNED128(a) a __attribute__ ((aligned (128)))
#define ATTRIBUTE_ALIGNED16(a) a
#define ATTRIBUTE_ALIGNED64(a) a
#define ATTRIBUTE_ALIGNED128(a) a
#ifndef assert
#include <assert.h>
#endif

#if defined(DEBUG) || defined(_DEBUG)
#define btAssert assert
#define btAssertMsg(x, str) assert(x)
#else
#define btAssert(x)
#define btAssertMsg(x, str)
#endif

//btFullAssert is optional, slows down a lot
#define btFullAssert(x)
#define btLikely(_c) _c
#define btUnlikely(_c) _c
#endif  //__APPLE__

#endif  // LIBSPE2

#endif  //__CELLOS_LV2__
#endif

#include "btAlignedAllocator.h"

#ifdef BT_DEBUG

// Declared twice for debugging information that may be passed through.
#define BT_DECLARE_ALIGNED_ALLOCATOR()                                                                                                                                                      \
	SIMD_FORCE_INLINE void* operator new(size_t sizeInBytes, int blockType, const char* pFileName, int nLine) { return btDbgAlignedAlloc(sizeInBytes, 16, blockType, pFileName, nLine); }   \
	SIMD_FORCE_INLINE void operator delete(void* ptr, int blockType) { btDbgAlignedFree(ptr, blockType); }                                                                                  \
	SIMD_FORCE_INLINE void operator delete(void* ptr, int blockType, const char*, int) { btDbgAlignedFree(ptr, blockType); }                                                                \
	SIMD_FORCE_INLINE void* operator new(size_t, void* ptr, int, const char*, int) { return ptr; }                                                                                          \
	SIMD_FORCE_INLINE void operator delete(void*, void*, int) {}                                                                                                                            \
	SIMD_FORCE_INLINE void operator delete(void*, void*, int, const char*, int) {}                                                                                                          \
	SIMD_FORCE_INLINE void* operator new[](size_t sizeInBytes, int blockType, const char* pFileName, int nLine) { return btDbgAlignedAlloc(sizeInBytes, 16, blockType, pFileName, nLine); } \
	SIMD_FORCE_INLINE void operator delete[](void* ptr, int blockType) { return btDbgAlignedFree(ptr, blockType); }                                                                         \
	SIMD_FORCE_INLINE void operator delete[](void* ptr, int blockType, const char*, int) { return btDbgAlignedFree(ptr, blockType); }                                                       \
	SIMD_FORCE_INLINE void* operator new[](size_t, void* ptr, int, const char*, int) { return ptr; }                                                                                        \
	SIMD_FORCE_INLINE void operator delete[](void*, void*, int) {}                                                                                                                          \
	SIMD_FORCE_INLINE void operator delete[](void*, void*, int, const char*, int) {}                                                                                                        \
                                                                                                                                                                                            \
	SIMD_FORCE_INLINE void* operator new(size_t sizeInBytes) { return btAlignedAlloc(sizeInBytes, 16); }                                                                                    \
	SIMD_FORCE_INLINE void operator delete(void* ptr) { btAlignedFree(ptr); }                                                                                                               \
	SIMD_FORCE_INLINE void* operator new(size_t, void* ptr) { return ptr; }                                                                                                                 \
	SIMD_FORCE_INLINE void operator delete(void*, void*) {}                                                                                                                                 \
	SIMD_FORCE_INLINE void* operator new[](size_t sizeInBytes) { return btAlignedAlloc(sizeInBytes, 16); }                                                                                  \
	SIMD_FORCE_INLINE void operator delete[](void* ptr) { btAlignedFree(ptr); }                                                                                                             \
	SIMD_FORCE_INLINE void* operator new[](size_t, void* ptr) { return ptr; }                                                                                                               \
	SIMD_FORCE_INLINE void operator delete[](void*, void*) {}

#else

#define BT_DECLARE_ALIGNED_ALLOCATOR()                                                                     \
	SIMD_FORCE_INLINE void* operator new(size_t sizeInBytes) { return btAlignedAlloc(sizeInBytes, 16); }   \
	SIMD_FORCE_INLINE void operator delete(void* ptr) { btAlignedFree(ptr); }                              \
	SIMD_FORCE_INLINE void* operator new(size_t, void* ptr) { return ptr; }                                \
	SIMD_FORCE_INLINE void operator delete(void*, void*) {}                                                \
	SIMD_FORCE_INLINE void* operator new[](size_t sizeInBytes) { return btAlignedAlloc(sizeInBytes, 16); } \
	SIMD_FORCE_INLINE void operator delete[](void* ptr) { btAlignedFree(ptr); }                            \
	SIMD_FORCE_INLINE void* operator new[](size_t, void* ptr) { return ptr; }                              \
	SIMD_FORCE_INLINE void operator delete[](void*, void*) {}

#endif

/// align a pointer to the provided alignment, upwards
/// alignment must be a multiple of 2!
template <typename T>
T* btAlignPointer(T* unalignedPtr, size_t alignment)
{
	struct btConvertPointerSizeT
	{
		union {
			T* ptr;
			size_t integer;
		};
	};
	btConvertPointerSizeT converter;

	const size_t bit_mask = ~(alignment - 1);
	converter.ptr = unalignedPtr;
	converter.integer += alignment - 1;
	converter.integer &= bit_mask;
	return converter.ptr;
}

#endif  // BT_DEFINES_H