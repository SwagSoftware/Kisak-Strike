#ifndef HK_MATH_MATH_H
#define HK_MATH_MATH_H

#include <hk_base/base.h>
#include <hk_math/types.h>

#include <math.h>

#	define HK_PI        3.14159265358979323846f	/* pi */
#	define HK_PI_2      1.57079632679489661923f	/* pi/2 */

#define HK_REAL_MAX 1e16f
#define HK_REAL_EPS 1e-16f		/* the minumum resolution of real */
#define HK_REAL_RES 1e-7f		/* resolution of hk_real of  relative to 1.0f */
class hk_Math
{
	public:
		static inline hk_double fabsd( hk_double );

		static inline hk_real sqrt( hk_real );
		static inline hk_real sqrt_inv( hk_real );
		static inline hk_real fast_sqrt( hk_real );
		static inline hk_real fast_sqrt_inv( hk_real );

		static inline hk_real fabs( hk_real );
		static inline hk_real tan( hk_real );
		static inline hk_real sin( hk_real );
		static inline hk_real cos( hk_real );
		static inline hk_real atan2( hk_real sinus, hk_real cosinus);
		static inline hk_real fast_approx_atan2( hk_real sinus, hk_real cosinus); // 0.01 error
		static inline hk_real fast_approx_atan2_normized( hk_real sinus, hk_real cosinus); // 0.01 error // input has to be normized
		
		static inline hk_real asin( hk_real );
		static inline hk_real acos( hk_real );
		static inline hk_real max( hk_real, hk_real );
		static inline hk_real min( hk_real, hk_real );

		static inline hk_real floor( hk_real );
		static inline hk_real ceil( hk_real );
		static inline hk_real clamp( hk_real v, hk_real min, hk_real max );
		static inline hk_real pow( hk_real, hk_real );
		static inline hk_real exp( hk_real );

		static void srand01( unsigned seedVal );
		static inline hk_real _rand01();
		static	      hk_real rand01();
		static inline int int_log2( hk_real ); // integer part of log2
		static unsigned int hk_random_seed;
};

#include <hk_math/math.inl>

#endif /* HK_MATH_MATH_H */
