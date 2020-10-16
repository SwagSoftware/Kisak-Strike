#ifdef WIN32
#include <float.h>
#endif

#if defined(IVP_NO_DOUBLE) && !defined(SUN)
#	include <math.h>

#   if defined(WIN32) || defined(PSXII) || defined(LINUX)
    union p_float_ieee {	    IVP_FLOAT val;
	    struct {
		    unsigned int valh:23;   unsigned int exp:8;   unsigned int signum:1;
	    } ln;    };
#else
    union p_float_ieee {	    IVP_FLOAT val;
	    struct {
		    unsigned int signum:1; unsigned int exp:8;   ;unsigned int valh:23;   
	    } ln;    };
#endif
    #define IVP_EXP_FOR_ONE 0x7f
    inline int PFM_LD(float a){ return ((p_float_ieee *)&(a))->ln.exp - IVP_EXP_FOR_ONE; };
#else
#	if defined(LINUX) || defined(WIN32)

union p_double_ieee {
	IVP_DOUBLE val;
	struct {
		int val;
		unsigned int valh:20;
		unsigned int exp:11;
		unsigned int signum:1;
	} ln;
  struct {
    int l;
    int h;
  } ln2;
};
#define IVP_EXP_FOR_ONE 0x3ff
inline int PFM_LD(double a){ return ((p_double_ieee *)&(a))->ln.exp - IVP_EXP_FOR_ONE; };
#	endif

#	if defined(SUN) || defined(SUN4) || defined(__POWERPC__) || defined(GEKKO) 
union p_double_ieee {
	double val;
	struct {
		unsigned int signum:1;
		unsigned int exp:11;
		unsigned int valh:20;
		int val;
	} ln;
        struct {
		int h;
		int l;
	} ln2;
};

#	define P_EXP_FOR_ONE 0x3ff
	inline int PFM_LD(double a){ return ((p_double_ieee *)&(a))->ln.exp - P_EXP_FOR_ONE; };
#	endif
#endif


class IVP_Fast_Math {
public:
#if defined(PSXII)
/// Calculates the dot product of the calling vector with v.
/// \param v   
inline    static IVP_DOUBLE isqrt(IVP_DOUBLE x, int /*resolution_steps*/)
{
	float u = 1.0f;
	__asm__ __volatile__ ("
	.set noreorder
		rsqrt.s	%0, %1, %0
	.set reorder
	" : "+f" (x) : "f" (u));
	return x;
}


/// Calculates the dot product of the calling vector with v.
/// \param v   
inline    static IVP_DOUBLE sqrt(IVP_DOUBLE x)
{
	__asm__ __volatile__ ("
	.set noreorder
		sqrt.s	%0, %0
	.set reorder
	" : "+f" (x) :);
	return x;
}

#elif defined(IVP_NO_DOUBLE)
  static IVP_DOUBLE isqrt(IVP_DOUBLE square, int /*resolution_steps*/){
    return 1.0f/IVP_Inline_Math::ivp_sqrtf(square);
  }

  static IVP_DOUBLE sqrt(IVP_DOUBLE x){
    return IVP_Inline_Math::ivp_sqrtf(x);
  }

#else
  // fast 1/sqrt(x),
  // resolution for resolution_steps
  // 0 -> 1e-3
  // 1 -> 1e-7
  // 2 -> 1e-14
  // 3 -> 1e-16
    static double isqrt(double square, int resolution_steps){
	p_double_ieee *ie = (p_double_ieee *)&square;
	IVP_ASSERT(IVP_Inline_Math::fabsd(square) > 0.0f);
	p_double_ieee h; h.val = 1.0f;
	h.ln2.h = ((0x07ff00000 - ie->ln2.h) >>1 ) + 0x1ff00000;
	IVP_DOUBLE squareh = square * 0.5f;
	IVP_DOUBLE inv_sqrt = h.val;
	
	inv_sqrt += inv_sqrt * (0.5f - inv_sqrt * inv_sqrt * squareh);
	inv_sqrt += inv_sqrt * (0.5f - inv_sqrt * inv_sqrt * squareh);
	if (resolution_steps > 0)	inv_sqrt += inv_sqrt * (0.5f - ( inv_sqrt * inv_sqrt * squareh ));
	if (resolution_steps > 1)	inv_sqrt += inv_sqrt * (0.5f - ( inv_sqrt * inv_sqrt * squareh ));
	if (resolution_steps > 2)	inv_sqrt += inv_sqrt * (0.5f - ( inv_sqrt * inv_sqrt * squareh ));

	IVP_ASSERT( IVP_Inline_Math::fabsd( 1.0f - inv_sqrt * inv_sqrt * square) < 0.001f );
	return inv_sqrt;
    }
    static IVP_DOUBLE sqrt(IVP_DOUBLE x) {
	return ::sqrt(x);
    }
#endif
};
