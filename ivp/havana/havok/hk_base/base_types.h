#ifndef HK_BASE_BASE_TYPES_H
#define HK_BASE_BASE_TYPES_H

// some of these are wrong
#if defined(_DEBUG) && !defined(HK_DEBUG)
#	define HK_DEBUG
#endif

#ifndef HK_ASSERT
# ifdef HK_DEBUG
#  define HK_ASSERT(a) { if(!(a)) hk_assert(a,#a,__LINE__,__FILE__); }
#  define HK_IF_DEBUG(a) if(a)
# else
#  define HK_ASSERT(a) 
#  define HK_IF_DEBUG(a) if(0)
# endif
#endif

#if defined(HK_DEBUG)
#	define HK_IF_CHECK(a) if (a)
#	define HK_CHECK(a){ if(!(a)) hk_check(a,#a,__LINE__,__FILE__); }
#else
#	define HK_IF_CHECK(a) if (0)
#	define HK_CHECK(a)
#endif

#define HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(a,b)
#define HK_BREAKPOINT() *(int*)0=0

class hkBaseObject
{
public:
	int m_memsize;
	virtual ~hkBaseObject(){;}
};




// simple commonly used types
typedef float hk_real;
typedef double hk_double;

typedef void* hk_client;

typedef signed char			hk_char;
typedef signed short		hk_int16;
typedef signed int			hk_int32;

typedef unsigned char		hk_uchar;
typedef unsigned short		hk_uint16;
typedef unsigned int		hk_uint32;

#if defined(__POWERPC__) && defined(__MWERKS__)
#include <stddef.h>
typedef size_t 			hk_size_t;  // CK: unsigned long int ..
#else
//lwss x64 fix
typedef unsigned long int 	hk_size_t;
//lwss end
#endif

#define HK_BREAK (*((int *)0)) = 0

#define HK_PUBLIC public

#if defined(__GNUC__)
#	define HK_HAVE_PRAGMA_INTERFACE
#	define HK_HAVE_GNU_INLINE_ASSEMBLY
	typedef signed long long	hk_int64;
	typedef unsigned long long	hk_uint64;
#elif defined(WIN32)
#	define HK_HAVE_FORCE_INLINE
#	define HK_HAVE_MSVC_INLINE_ASSEMBLY
	typedef signed __int64		hk_int64;
	typedef unsigned __int64	hk_uint64;
#endif


#define HK_NULL 0

//: Note that M must be a power of two for this to work.
#define HK_NEXT_MULTIPLE_OF(M, P)  ( ((P) + ((M)-1)) & (~((M)-1)) )

void hk_assert(bool test, const char* cond, int line, const char* file);
void hk_check (bool test, const char* cond, int line, const char* file);

// return values for just simple functions
enum hk_result
{
	HK_OK,
	HK_FAULT
};

typedef bool hk_bool;
#define HK_FALSE false
#define HK_TRUE  true

typedef hk_real		hk_time;
typedef hk_uint16	hk_sorted_set_store_index;

typedef hk_int32	hk_sorted_set_index;
typedef hk_int32	hk_array_index;
typedef hk_uint16	hk_array_store_index;
typedef hk_uint32	hk_id;

#ifdef HK_HAVE_FORCE_INLINE
#	define inline __forceinline
#endif
#define HK_TEMPLATE_INLINE inline

#if defined(__i386__) || defined(WIN32)
#	define HK_HAVE_QUERY_PERFORMANCE_TIMER
#endif

#if !defined(HK_ALIGNED_VARIABLE)
#	if defined(HK_PS2)
#		define HK_ALIGNED_VARIABLE(NAME,ALIGNMENT) NAME __attribute__((aligned(ALIGNMENT)))
#	elif defined(HK_PIII_SSE)
#		define HK_ALIGNED_VARIABLE(NAME,ALIGNMENT) __declspec(align(ALIGNMENT))
#	else //no special alignment
#		define HK_ALIGNED_VARIABLE(NAME,ALIGNMENT) NAME
#	endif 
#endif //HK_ALIGNED_VARIABLE

#endif //HK_BASE_BASE_TYPES_H

