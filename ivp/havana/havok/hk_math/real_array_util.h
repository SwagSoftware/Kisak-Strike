#ifndef HK_MATH_REAL_ARRAY_UTIL_H
#define HK_MATH_REAL_ARRAY_UTIL_H

#include <hk_base/base.h>

//: Utility class for numerical operations on arrays of reals
// This class is optimised to use vector units and prefetches
// for maximum performance on platforms which support it.
// Note that all array arguments are assumed to have suitable
// alignment for the vector unit. If this is not the case, these
// routines will cause a crash.
class hk_Real_Array_Util
{
    public:

		static hk_real dot(const hk_real* x, const hk_real* y);
};

#endif /* HK_MATH_REAL_ARRAY_UTIL_H */

