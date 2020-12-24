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

#ifndef BT_SCALAR_H
#define BT_SCALAR_H

#ifdef BT_MANAGED_CODE
//Aligned data types not supported in managed code
#pragma unmanaged
#endif

#include <math.h>
#include <stdlib.h>  //size_t for MSVC 6.0
#include <float.h>

// Due to the brilliant design choices of bullet, we'll have to include this here.
#include "btDefines.h"

///The btScalar type abstracts floating point numbers, to easily switch between double and single floating point precision.
#if defined(BT_USE_DOUBLE_PRECISION)
typedef double btScalar;
//this number could be bigger in double precision
#define BT_LARGE_FLOAT 1e30
#else
typedef float btScalar;
//keep BT_LARGE_FLOAT*BT_LARGE_FLOAT < FLT_MAX
#define BT_LARGE_FLOAT 1e18f
#endif

#ifdef BT_USE_SSE
typedef __m128 btSimdFloat4;
#endif  //BT_USE_SSE

#if defined(BT_USE_SSE)
//#if defined BT_USE_SSE_IN_API && defined (BT_USE_SSE)
#ifdef _WIN32

#ifndef BT_NAN
static int btNanMask = 0x7F800001;
#define BT_NAN (*(float *)&btNanMask)
#endif

#ifndef BT_INFINITY
static int btInfinityMask = 0x7F800000;
#define BT_INFINITY (*(float *)&btInfinityMask)
inline int btGetInfinityMask()  // suppress stupid compiler warning
{
	return btInfinityMask;
}
#endif

//use this, in case there are clashes (such as xnamath.h)
#ifndef BT_NO_SIMD_OPERATOR_OVERLOADS
SIMD_FORCE_INLINE __m128 operator+(const __m128 A, const __m128 B)
{
	return _mm_add_ps(A, B);
}

SIMD_FORCE_INLINE __m128 operator-(const __m128 A, const __m128 B)
{
	return _mm_sub_ps(A, B);
}

SIMD_FORCE_INLINE __m128 operator*(const __m128 A, const __m128 B)
{
	return _mm_mul_ps(A, B);
}

SIMD_FORCE_INLINE __m128 operator/(const __m128 A, const __m128 B)
{
	return _mm_div_ps(A, B);
}
#endif  //BT_NO_SIMD_OPERATOR_OVERLOADS

#define btCastfTo128i(a) (_mm_castps_si128(a))
#define btCastfTo128d(a) (_mm_castps_pd(a))
#define btCastiTo128f(a) (_mm_castsi128_ps(a))
#define btCastdTo128f(a) (_mm_castpd_ps(a))
#define btCastdTo128i(a) (_mm_castpd_si128(a))
#define btAssign128(r0, r1, r2, r3) _mm_setr_ps(r0, r1, r2, r3)

#else  //_WIN32

#define btCastfTo128i(a) ((__m128i)(a))
#define btCastfTo128d(a) ((__m128d)(a))
#define btCastiTo128f(a) ((__m128)(a))
#define btCastdTo128f(a) ((__m128)(a))
#define btCastdTo128i(a) ((__m128i)(a))
#define btAssign128(r0, r1, r2, r3) \
	(__m128) { r0, r1, r2, r3 }
#define BT_INFINITY INFINITY
#define BT_NAN NAN
#endif  //_WIN32
#else   //BT_USE_SSE_IN_API

#ifdef BT_USE_NEON
#include <arm_neon.h>

typedef float32x4_t btSimdFloat4;
#define BT_INFINITY INFINITY
#define BT_NAN NAN
#define btAssign128(r0, r1, r2, r3) \
	(float32x4_t) { r0, r1, r2, r3 }
#else  // BT_USE_NEON
#ifndef BT_INFINITY
struct btInfMaskConverter
{
	union {
		float mask;
		int intmask;
	};
	btInfMaskConverter(int _mask = 0x7F800000)
		: intmask(_mask)
	{
	}
};
static btInfMaskConverter btInfinityMask = 0x7F800000;
#define BT_INFINITY (btInfinityMask.mask)
inline int btGetInfinityMask()  //suppress stupid compiler warning
{
	return btInfinityMask.intmask;
}
#endif  // BT_INFINITY
#endif  // BT_USE_NEON

#endif  // BT_USE_SSE_IN_API

#ifdef BT_USE_NEON
#include <arm_neon.h>

typedef float32x4_t btSimdFloat4;
#define BT_INFINITY INFINITY
#define BT_NAN NAN
#define btAssign128(r0, r1, r2, r3) \
	(float32x4_t) { r0, r1, r2, r3 }
#endif  // BT_USE_NEON

#if defined(BT_USE_DOUBLE_PRECISION) || defined(BT_FORCE_DOUBLE_FUNCTIONS)

SIMD_FORCE_INLINE btScalar btSqrt(btScalar x)
{
	return sqrt(x);
}
SIMD_FORCE_INLINE btScalar btFabs(btScalar x) { return fabs(x); }
SIMD_FORCE_INLINE btScalar btCos(btScalar x) { return cos(x); }
SIMD_FORCE_INLINE btScalar btSin(btScalar x) { return sin(x); }
SIMD_FORCE_INLINE btScalar btTan(btScalar x) { return tan(x); }
SIMD_FORCE_INLINE btScalar btAcos(btScalar x)
{
	if (x < btScalar(-1)) x = btScalar(-1);
	if (x > btScalar(1)) x = btScalar(1);
	return acos(x);
}
SIMD_FORCE_INLINE btScalar btAsin(btScalar x)
{
	if (x < btScalar(-1)) x = btScalar(-1);
	if (x > btScalar(1)) x = btScalar(1);
	return asin(x);
}
SIMD_FORCE_INLINE btScalar btAtan(btScalar x) { return atan(x); }
SIMD_FORCE_INLINE btScalar btAtan2(btScalar x, btScalar y) { return atan2(x, y); }
SIMD_FORCE_INLINE btScalar btExp(btScalar x) { return exp(x); }
SIMD_FORCE_INLINE btScalar btLog(btScalar x) { return log(x); }
SIMD_FORCE_INLINE btScalar btPow(btScalar x, btScalar y) { return pow(x, y); }
SIMD_FORCE_INLINE btScalar btFmod(btScalar x, btScalar y) { return fmod(x, y); }

#else

SIMD_FORCE_INLINE btScalar btSqrt(btScalar y)
{
#ifdef USE_APPROXIMATION
#ifdef __LP64__
	float xhalf = 0.5f * y;
	int i = *(int *)&y;
	i = 0x5f375a86 - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - xhalf * y * y);
	y = y * (1.5f - xhalf * y * y);
	y = y * (1.5f - xhalf * y * y);
	y = 1 / y;
	return y;
#else
	double x, z, tempf;
	unsigned long *tfptr = ((unsigned long *)&tempf) + 1;
	tempf = y;
	*tfptr = (0xbfcdd90a - *tfptr) >> 1; /* estimate of 1/sqrt(y) */
	x = tempf;
	z = y * btScalar(0.5);
	x = (btScalar(1.5) * x) - (x * x) * (x * z); /* iteration formula     */
	x = (btScalar(1.5) * x) - (x * x) * (x * z);
	x = (btScalar(1.5) * x) - (x * x) * (x * z);
	x = (btScalar(1.5) * x) - (x * x) * (x * z);
	x = (btScalar(1.5) * x) - (x * x) * (x * z);
	return x * y;
#endif
#else
	return sqrtf(y);
#endif
}
SIMD_FORCE_INLINE btScalar btFabs(btScalar x) { return fabsf(x); }
SIMD_FORCE_INLINE btScalar btCos(btScalar x) { return cosf(x); }
SIMD_FORCE_INLINE btScalar btSin(btScalar x) { return sinf(x); }
SIMD_FORCE_INLINE btScalar btTan(btScalar x) { return tanf(x); }
SIMD_FORCE_INLINE btScalar btAcos(btScalar x)
{
	if (x < btScalar(-1))
		x = btScalar(-1);
	if (x > btScalar(1))
		x = btScalar(1);
	return acosf(x);
}
SIMD_FORCE_INLINE btScalar btAsin(btScalar x)
{
	if (x < btScalar(-1))
		x = btScalar(-1);
	if (x > btScalar(1))
		x = btScalar(1);
	return asinf(x);
}
SIMD_FORCE_INLINE btScalar btAtan(btScalar x) { return atanf(x); }
SIMD_FORCE_INLINE btScalar btAtan2(btScalar x, btScalar y) { return atan2f(x, y); }
SIMD_FORCE_INLINE btScalar btExp(btScalar x) { return expf(x); }
SIMD_FORCE_INLINE btScalar btLog(btScalar x) { return logf(x); }
SIMD_FORCE_INLINE btScalar btPow(btScalar x, btScalar y) { return powf(x, y); }
SIMD_FORCE_INLINE btScalar btFmod(btScalar x, btScalar y) { return fmodf(x, y); }

#endif

#define SIMD_PI btScalar(3.1415926535897932384626433832795029)
#define SIMD_2_PI (btScalar(2.0) * SIMD_PI)
#define SIMD_HALF_PI (SIMD_PI * btScalar(0.5))
#define SIMD_RADS_PER_DEG (SIMD_2_PI / btScalar(360.0))
#define SIMD_DEGS_PER_RAD (btScalar(360.0) / SIMD_2_PI)
#define SIMDSQRT12 btScalar(0.7071067811865475244008443621048490)

#define btRecipSqrt(x) ((btScalar)(btScalar(1.0) / btSqrt(btScalar(x)))) /* reciprocal square root */
#define btRecip(x) (btScalar(1.0) / btScalar(x))

#ifdef BT_USE_DOUBLE_PRECISION
#define SIMD_EPSILON DBL_EPSILON
#define SIMD_INFINITY DBL_MAX
#define BT_ONE 1.0
#define BT_ZERO 0.0
#define BT_TWO 2.0
#define BT_HALF 0.5
#else
#define SIMD_EPSILON FLT_EPSILON
#define SIMD_INFINITY FLT_MAX
#define BT_ONE 1.0f
#define BT_ZERO 0.0f
#define BT_TWO 2.0f
#define BT_HALF 0.5f
#endif

SIMD_FORCE_INLINE btScalar btAtan2Fast(btScalar y, btScalar x)
{
	btScalar coeff_1 = SIMD_PI / 4.0f;
	btScalar coeff_2 = 3.0f * coeff_1;
	btScalar abs_y = btFabs(y);
	btScalar angle;
	if (x >= 0.0f)
	{
		btScalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	}
	else
	{
		btScalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return (y < 0.0f) ? -angle : angle;
}

SIMD_FORCE_INLINE bool btFuzzyZero(btScalar x) { return btFabs(x) < SIMD_EPSILON; }

SIMD_FORCE_INLINE bool btEqual(btScalar a, btScalar eps)
{
	return (((a) <= eps) && !((a) < -eps));
}
SIMD_FORCE_INLINE bool btGreaterEqual(btScalar a, btScalar eps)
{
	return (!((a) <= eps));
}

SIMD_FORCE_INLINE int btIsNegative(btScalar x)
{
	return x < btScalar(0.0) ? 1 : 0;
}

SIMD_FORCE_INLINE btScalar btRadians(btScalar x) { return x * SIMD_RADS_PER_DEG; }
SIMD_FORCE_INLINE btScalar btDegrees(btScalar x) { return x * SIMD_DEGS_PER_RAD; }

#define BT_DECLARE_HANDLE(name) \
	typedef struct name##__     \
	{                           \
		int unused;             \
	} * name

#ifndef btFsel
SIMD_FORCE_INLINE btScalar btFsel(btScalar a, btScalar b, btScalar c)
{
	return a >= 0 ? b : c;
}
#endif
#define btFsels(a, b, c) (btScalar) btFsel(a, b, c)

SIMD_FORCE_INLINE bool btMachineIsLittleEndian()
{
	long int i = 1;
	const char *p = (const char *)&i;
	if (p[0] == 1)  // Lowest address contains the least significant byte
		return true;
	else
		return false;
}

/*
SIMD_FORCE_INLINE bool btIsFinite(btScalar x)
{
	return isfinite(x);
}

SIMD_FORCE_INLINE bool btIsNaN(btScalar x)
{
	return isnan(x);
}
*/

///btSelect avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
SIMD_FORCE_INLINE unsigned btSelect(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero)
{
	// Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
	// Rely on positive value or'ed with its negative having sign bit on
	// and zero value or'ed with its negative (which is still zero) having sign bit off
	// Use arithmetic shift right, shifting the sign bit through all 32 bits
	unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
	unsigned testEqz = ~testNz;
	return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}

SIMD_FORCE_INLINE int btSelect(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
	unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
	unsigned testEqz = ~testNz;
	return static_cast<int>((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}

SIMD_FORCE_INLINE float btSelect(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef BT_HAVE_NATIVE_FSEL
	return (float)btFsel((btScalar)condition - btScalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
	return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero;
#endif
}

template <typename T>
SIMD_FORCE_INLINE void btSwap(T &a, T &b)
{
	T tmp = a;
	a = b;
	b = tmp;
}

//PCK: endian swapping functions
// FIXME: Should these really be here?
SIMD_FORCE_INLINE unsigned btSwapEndian(unsigned val)
{
	return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8) | ((val & 0x000000ff) << 24));
}

SIMD_FORCE_INLINE unsigned short btSwapEndian(unsigned short val)
{
	return static_cast<unsigned short>(((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
}

SIMD_FORCE_INLINE unsigned btSwapEndian(int val)
{
	return btSwapEndian((unsigned)val);
}

SIMD_FORCE_INLINE unsigned short btSwapEndian(short val)
{
	return btSwapEndian((unsigned short)val);
}

// btSwapFloat uses using char pointers to swap the endianness
// btSwapFloat/btSwapDouble will NOT return a float, because the machine might 'correct' invalid floating point values
// Not all values of sign/exponent/mantissa are valid floating point numbers according to IEEE 754.
// When a floating point unit is faced with an invalid value, it may actually change the value, or worse, throw an exception.
// In most systems, running user mode code, you wouldn't get an exception, but instead the hardware/os/runtime will 'fix' the number for you.
// so instead of returning a float/double, we return integer/long long integer
SIMD_FORCE_INLINE unsigned int btSwapEndianFloat(float d)
{
	unsigned int a = 0;
	unsigned char *dst = (unsigned char *)&a;
	unsigned char *src = (unsigned char *)&d;

	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];
	return a;
}

// unswap using char pointers
SIMD_FORCE_INLINE float btUnswapEndianFloat(unsigned int a)
{
	float d = 0.0f;
	unsigned char *src = (unsigned char *)&a;
	unsigned char *dst = (unsigned char *)&d;

	dst[0] = src[3];
	dst[1] = src[2];
	dst[2] = src[1];
	dst[3] = src[0];

	return d;
}

// swap using char pointers
SIMD_FORCE_INLINE void btSwapEndianDouble(double d, unsigned char *dst)
{
	unsigned char *src = (unsigned char *)&d;

	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];
}

// unswap using char pointers
SIMD_FORCE_INLINE double btUnswapEndianDouble(const unsigned char *src)
{
	double d = 0.0;
	unsigned char *dst = (unsigned char *)&d;

	dst[0] = src[7];
	dst[1] = src[6];
	dst[2] = src[5];
	dst[3] = src[4];
	dst[4] = src[3];
	dst[5] = src[2];
	dst[6] = src[1];
	dst[7] = src[0];

	return d;
}

template <typename T>
SIMD_FORCE_INLINE void btSetZero(T *a, int n)
{
	T *acurr = a;
	size_t ncurr = n;
	while (ncurr > 0)
	{
		*(acurr++) = 0;
		--ncurr;
	}
}

SIMD_FORCE_INLINE btScalar btLargeDot(const btScalar *a, const btScalar *b, int n)
{
	btScalar p0, q0, m0, p1, q1, m1, sum;
	sum = 0;
	n -= 2;
	while (n >= 0)
	{
		p0 = a[0];
		q0 = b[0];
		m0 = p0 * q0;
		p1 = a[1];
		q1 = b[1];
		m1 = p1 * q1;
		sum += m0;
		sum += m1;
		a += 2;
		b += 2;
		n -= 2;
	}
	n += 2;
	while (n > 0)
	{
		sum += (*a) * (*b);
		a++;
		b++;
		n--;
	}
	return sum;
}

// returns normalized value in range [-SIMD_PI, SIMD_PI]
SIMD_FORCE_INLINE btScalar btNormalizeAngle(btScalar angleInRadians)
{
	angleInRadians = btFmod(angleInRadians, SIMD_2_PI);
	if (angleInRadians < -SIMD_PI)
	{
		return angleInRadians + SIMD_2_PI;
	}
	else if (angleInRadians > SIMD_PI)
	{
		return angleInRadians - SIMD_2_PI;
	}
	else
	{
		return angleInRadians;
	}
}

///rudimentary class to provide type info
struct btTypedObject
{
	btTypedObject(int objectType)
		: m_objectType(objectType)
	{
	}

	inline int getObjectType() const
	{
		return m_objectType;
	}

	int m_objectType;
};

// btSimdScalar class
#ifdef BT_USE_SSE

ATTRIBUTE_ALIGNED16(struct)
btSimdScalar
{
	//BT_DECLARE_ALIGNED_ALLOCATOR();

	// Data
	union {
		btSimdFloat4 m_vec128;
		btScalar m_floats[4];
		int m_ints[4];
	};

	SIMD_FORCE_INLINE btSimdScalar()
	{
	}

	SIMD_FORCE_INLINE btSimdScalar(float fl)
		: m_vec128(_mm_set1_ps(fl))
	{
	}

	SIMD_FORCE_INLINE btSimdScalar(__m128 v128)
		: m_vec128(v128)
	{
	}

	SIMD_FORCE_INLINE btSimdFloat4 get128()
	{
		return m_vec128;
	}

	SIMD_FORCE_INLINE const btSimdFloat4 get128() const
	{
		return m_vec128;
	}

	SIMD_FORCE_INLINE void set128(btSimdFloat4 v128)
	{
		m_vec128 = v128;
	}

	SIMD_FORCE_INLINE operator btSimdFloat4()
	{
		return m_vec128;
	}

	SIMD_FORCE_INLINE operator const btSimdFloat4() const
	{
		return m_vec128;
	}

	SIMD_FORCE_INLINE operator btScalar() const
	{
		return m_floats[0];
	}

	// Operators

	SIMD_FORCE_INLINE btSimdScalar operator|(const btSimdScalar &other) const
	{
		return btSimdScalar(_mm_or_ps(get128(), other.get128()));
	}

	SIMD_FORCE_INLINE btSimdScalar operator&(const btSimdScalar &other) const
	{
		return btSimdScalar(_mm_and_ps(get128(), other.get128()));
	}

	SIMD_FORCE_INLINE btSimdScalar operator*(const btSimdScalar &other) const
	{
		return btSimdScalar(_mm_mul_ps(get128(), other.get128()));
	}

	SIMD_FORCE_INLINE btSimdScalar operator*(const btScalar &other) const
	{
		return btSimdScalar(_mm_mul_ps(get128(), _mm_set1_ps(other)));
	}

	SIMD_FORCE_INLINE btSimdScalar operator/(const btSimdScalar &other) const
	{
		return btSimdScalar(_mm_div_ps(get128(), other.get128()));
	}

	SIMD_FORCE_INLINE btSimdScalar operator/(const btScalar &other) const
	{
		return btSimdScalar(_mm_div_ps(get128(), _mm_set1_ps(other)));
	}

	SIMD_FORCE_INLINE btSimdScalar operator+(const btSimdScalar &other) const
	{
		return btSimdScalar(_mm_add_ps(get128(), other.get128()));
	}

	SIMD_FORCE_INLINE btSimdScalar operator+(const btScalar &other) const
	{
		return btSimdScalar(_mm_add_ps(get128(), _mm_set1_ps(other)));
	}

	SIMD_FORCE_INLINE btSimdScalar operator-(const btSimdScalar &other) const
	{
		return btSimdScalar(_mm_sub_ps(get128(), other.get128()));
	}

	SIMD_FORCE_INLINE btSimdScalar operator-(const btScalar &other) const
	{
		return btSimdScalar(_mm_sub_ps(get128(), _mm_set1_ps(other)));
	}

	SIMD_FORCE_INLINE btSimdScalar &operator+=(const btSimdScalar &other)
	{
		*this = *this + other;
		return *this;
	}

	SIMD_FORCE_INLINE btSimdScalar &operator-=(const btSimdScalar &other)
	{
		*this = *this - other;
		return *this;
	}
};

SIMD_FORCE_INLINE
btSimdScalar operator-(const float &v1, const btSimdScalar &v2)
{
	return btSimdScalar(_mm_sub_ps(_mm_set1_ps(v1), v2.get128()));
}

SIMD_FORCE_INLINE
btSimdScalar operator+(const float &v1, const btSimdScalar &v2)
{
	return btSimdScalar(_mm_add_ps(_mm_set1_ps(v1), v2.get128()));
}

/*
///@brief Return the elementwise product of two btSimdScalar
SIMD_FORCE_INLINE
btSimdScalar operator*(const btSimdScalar& v1, const btSimdScalar& v2) 
{
	return btSimdScalar(_mm_mul_ps(v1.get128(), v2.get128()));
}

SIMD_FORCE_INLINE
btSimdScalar operator/(const btSimdScalar &v1, const btSimdScalar &v2)
{
	return btSimdScalar(_mm_div_ps(v1.get128(), v2.get128()));
}

///@brief Return the elementwise sum of two btSimdScalar
SIMD_FORCE_INLINE
btSimdScalar operator+(const btSimdScalar& v1, const btSimdScalar& v2) 
{
	return btSimdScalar(_mm_add_ps(v1.get128(), v2.get128()));
}

///@brief Return the elementwise difference of two btSimdScalar
SIMD_FORCE_INLINE
btSimdScalar operator-(const btSimdScalar &v1, const btSimdScalar &v2)
{
	return btSimdScalar(_mm_sub_ps(v1.get128(), v2.get128()));
}
*/

#else
typedef btSimdScalar btScalar;
#endif

#endif  //BT_SCALAR_H