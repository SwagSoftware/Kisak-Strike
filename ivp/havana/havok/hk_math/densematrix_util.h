// TKBMS v1.0 -----------------------------------------------------
//
// PLATFORM		: ALL
// PRODUCT		: VPHYSICS
// VISIBILITY	: INTERNAL
//
// ------------------------------------------------------TKBMS v1.0

#ifndef HK_MATH_DENSE_MATRIX_UTIL_H
#define HK_MATH_DENSE_MATRIX_UTIL_H

class hk_Dense_Vector;
class hk_Dense_Matrix;

class hk_Dense_Matrix_Util
{
	public:

		static void mult( const hk_Dense_Matrix& m, const hk_Dense_Vector& in, hk_Dense_Vector& out );
		static void mult( const hk_Dense_Matrix& m, const hk_real *in, hk_real *out );
		static inline void mult_3_symmetric( const hk_Fixed_Dense_Matrix<3>& m, const hk_Vector3& in, hk_Vector3& out );

		static hk_result invert(hk_Dense_Matrix& m, hk_real tolerance);
		static hk_result invert_6x6(hk_Fixed_Dense_Matrix<6>& m, hk_real tolerance);
		static hk_result invert_5x5(hk_Fixed_Dense_Matrix<5>& m, hk_real tolerance);
		static hk_result invert_4x4(hk_Fixed_Dense_Matrix<4>& m, hk_real tolerance);
		static hk_result invert_3x3_symmetric(hk_Dense_Matrix& m, hk_real tolerance);
		static hk_result invert_2x2(const hk_Dense_Matrix& in, hk_Dense_Matrix& out, hk_real tolerance);

		static hk_result solve(hk_Dense_Matrix& m, hk_Dense_Vector& v, hk_real tolerance);
			//: solve, destroy matrix m in the process

		static void print(const hk_Dense_Matrix &m);
};

#include <hk_math/densematrix_util.inl>

#endif /*HK_MATH_DENSE_MATRIX_UTIL_H*/
