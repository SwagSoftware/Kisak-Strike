#ifdef HK_PS2
#	include <hk_math/math_ps2.inl>
#else	//HK_PS2

#ifdef _WIN32
inline hk_double hk_Math::fabsd( hk_double r ) { return hk_double(::fabs(r)); }

inline hk_real hk_Math::sqrt( hk_real r) { return hk_real(::sqrt(r)); }
inline hk_real hk_Math::sqrt_inv( hk_real r) { return 1.0f / hk_real(::sqrt(r)); }

inline hk_real hk_Math::fast_sqrt( hk_real r) { return hk_real(::sqrt(r)); }
inline hk_real hk_Math::fast_sqrt_inv( hk_real r) { return 1.0f / hk_real(::sqrt(r)); }

inline hk_real hk_Math::fabs( hk_real r) { return hk_real(::fabs(r)); }
inline hk_real hk_Math::tan( hk_real r) { return hk_real(::tan(r)); }
inline hk_real hk_Math::sin( hk_real r) { return hk_real(::sin(r)); }
inline hk_real hk_Math::cos( hk_real r) { return hk_real(::cos(r)); }
inline hk_real hk_Math::atan2( hk_real a, hk_real b)  { return hk_real(::atan2(a,b)); }
inline hk_real hk_Math::asin( hk_real r) { return hk_real(::asin(r)); }
inline hk_real hk_Math::acos( hk_real r) { return hk_real(::acos(r)); }
inline hk_real hk_Math::max( hk_real a, hk_real b) { return a>b ? a : b; }
inline hk_real hk_Math::min( hk_real a, hk_real b) { return a<b ? a : b; }

inline hk_real hk_Math::exp( hk_real e) { return hk_real(::exp(e)); }

inline hk_real hk_Math::floor( hk_real r ) { return hk_real(::floor(r)); }
inline hk_real hk_Math::ceil( hk_real r) { return hk_real(::ceil(r)); }
inline hk_real hk_Math::pow( hk_real r, hk_real p) { return hk_real(::pow(r,p)); }

#elif _LINUX

//lwss - haha, these were infinitely recursing for some reason. Adding f postfix to the function names.
inline hk_double hk_Math::fabsd( hk_double r ) { return hk_double(fabs(r)); }

inline hk_real hk_Math::sqrt( hk_real r) { return hk_real(sqrtf(r)); }
inline hk_real hk_Math::sqrt_inv( hk_real r) { return 1.0f / hk_real(sqrt(r)); }

inline hk_real hk_Math::fast_sqrt( hk_real r) { return hk_real(sqrt(r)); }
inline hk_real hk_Math::fast_sqrt_inv( hk_real r) { return 1.0f / hk_real(sqrt(r)); }

inline hk_real hk_Math::fabs( hk_real r) { return hk_real(fabsf(r)); }
inline hk_real hk_Math::tan( hk_real r) { return hk_real(tanf(r)); }
inline hk_real hk_Math::sin( hk_real r) { return hk_real(sinf(r)); }
inline hk_real hk_Math::cos( hk_real r) { return hk_real(cosf(r)); }
inline hk_real hk_Math::atan2( hk_real a, hk_real b)  { return hk_real(atan2f(a,b)); }
inline hk_real hk_Math::asin( hk_real r) { return hk_real(asinf(r)); }
inline hk_real hk_Math::acos( hk_real r) { return hk_real(acosf(r)); }
inline hk_real hk_Math::max( hk_real a, hk_real b) { return a>b ? a : b; }
inline hk_real hk_Math::min( hk_real a, hk_real b) { return a<b ? a : b; }

inline hk_real hk_Math::exp( hk_real e) { return hk_real(expf(e)); }

inline hk_real hk_Math::floor( hk_real r ) { return hk_real(floorf(r)); }
inline hk_real hk_Math::ceil( hk_real r) { return hk_real(ceilf(r)); }
inline hk_real hk_Math::pow( hk_real r, hk_real p) { return hk_real(powf(r,p)); }

//lwss end
#endif

inline hk_real hk_Math::clamp( hk_real r, hk_real mn, hk_real mx)
{
	HK_ASSERT(mn<=mx);
	return ((r<mn)
				? mn
				: ((r>mx) ? mx : r));
}

inline int hk_Math::int_log2( hk_real ) { return 0; }


inline hk_real hk_Math::_rand01()
{
	// BSD rand function
	const unsigned a = 1103515245;
	const unsigned c = 12345;
	const unsigned m = unsigned(-1) >> 1;
	hk_random_seed = (a * hk_random_seed + c ) & m;
	return hk_real(hk_random_seed) / m;
}

inline hk_real hk_Math::fast_approx_atan2_normized( hk_real y, hk_real x)
{
	//const hk_real delta_at_pi_4 = HK_PI * 0.25f - hk_Math::sin(HK_PI * 0.25f);
	//const hk_real f = delta_at_pi_4 / hk_Math::sin(HK_PI * 0.25f) / hk_Math::sin(HK_PI * 0.25f)/ hk_Math::sin(HK_PI * 0.25f);
	const hk_real f = 0.2214414775f;
	hk_real r;
	if ( x > y ){
		if ( x > - y){
			r = y;
			r += y * y * y * f;
		}else{
			r = x - HK_PI_2;
			r += x * x * x * f;
		}
	}else{
		if ( x > - y){
			r = HK_PI_2 - x;
			r -= x * x * x * f;
		}else{
			if ( y > 0 ){
				r = HK_PI - y;
			}else{
				r = -y - HK_PI;
			}
			r -= y * y * y * f;
		}
	}
	return r;
}

inline hk_real hk_Math::fast_approx_atan2( hk_real y, hk_real x)
{
	//const hk_real delta_at_pi_4 = HK_PI * 0.25f - hk_Math::sin(HK_PI * 0.25f);
	//const hk_real f = delta_at_pi_4 / hk_Math::sin(HK_PI * 0.25f) / hk_Math::sin(HK_PI * 0.25f)/ hk_Math::sin(HK_PI * 0.25f);
	const hk_real f = 0.2214414775f;
	hk_real r;
	hk_real q = hk_Math::sqrt_inv( x * x + y * y );
	if ( x > y ){
		if ( x > - y){
			y *= q;
			r = y;
			r += y * y * y * f;
		}else{
			x *= q;
			r = x - HK_PI_2;
			r += x * x * x * f;
		}
	}else{
		if ( x > - y){
			x *= q;
			r = HK_PI_2 - x;
			r -= x * x * x * f;
		}else{
			y *= q;
			if ( y > 0 ){
				r = HK_PI - y;
			}else{
				r = -y - HK_PI;
			}
			r -= y * y * y * f;
		}
	}
	return r;
}


#endif// HK_PS2

