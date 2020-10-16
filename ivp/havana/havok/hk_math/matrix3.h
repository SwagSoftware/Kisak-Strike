#ifndef HAK_MATH_MATRIX3_H
#define HAK_MATH_MATRIX3_H

#ifndef HK_MATH_VECMATH_H
#error Include <hk_math/vecmath.h> Do not include this file directly.
#endif // HK_MATH_VECMATH_H

//: A generic 3x3 matrix
class hk_Matrix3
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Matrix3)

		inline hk_Matrix3();
			//: Empty default constructor
		inline ~hk_Matrix3() {}
			//:: Empty default destructor

		inline void set_elem(int row, int column, hk_real value);
			//: Set element (row,column) to value
			// No range checking is done.

		inline hk_real operator() (int r, int c) const;
			//: Get element (row, column)
			// No range checking is done.

		inline void _set_rows( const hk_Vector3& r0,
					   const hk_Vector3& r1,
					   const hk_Vector3& r2);
			//: Set all values of the matrix at once, rowwise.

		void set_rows( const hk_Vector3& r0,
					   const hk_Vector3& r1,
					   const hk_Vector3& r2);
			//: Set all values of the matrix at once, rowwise.

		void set_cols( const hk_Vector3& r0,
					   const hk_Vector3& r1,
					   const hk_Vector3& r2);
			//: Set all values of the matrix at once, rowwise.

		inline void _get_rows( hk_Vector3& r0, hk_Vector3& r1, hk_Vector3& r2);
		inline void _get_row( int row, hk_Vector3& r);

		void set_zero();

		void set_diagonal( hk_real m00, hk_real m11, hk_real m22 );

		void set_identity_rotation();
			//: Sets this to be the identity
			// i.e. all diagonal elements set to 1, all
			// nondiagonal elements set 0.

		bool is_identity_rotation() const;

		void set_cross_skew( const hk_Vector3& r, const hk_Matrix3& R );
			//: r x R is accomplished by converting r to the skew symetric matrix r~
			// and then muliplying r~*R.

		void rotate ( int axis, hk_real angle );
			//: rotates the matrix in place 
			// Note: very very slow function

		hk_result invert(hk_real epsilon);
			//: Attempt to invert the matrix in place.
			// If the matrix is invertible, it is overwritten with
			// its inverse and the routine returns HK_OK. Otherwise
			// the matrix is left unchanged and HK_FAULT is returned.

		void set_rotated_diagonal_matrix( const hk_Rotation &r, const hk_Vector3 &diagonal_matrix3 );
			//: this = r * matrix3(diagonal_matrix3)  *  r.transpose()

		void transpose();

		void set_mul3( const hk_Matrix3& a, const hk_Matrix3& b );

		void set_mul3_inv2( const hk_Matrix3& a, const hk_Rotation& b );
		//: this = a * b.transpose()

		void set_mul3_inv( const hk_Rotation& a, const hk_Matrix3& b );
			//: this = a.transpose() * b

		void operator +=( const hk_Matrix3& a );
		void operator -=( const hk_Matrix3& a );

		inline hk_real *get_elem_address(int r, int c);
		
		inline hk_Vector3& get_column(int x);

		inline const hk_Vector3& get_column(int x) const;

	protected:

		hk_real		HK_ALIGNED_VARIABLE(m_elems[12],16);

};

#endif /* HAK_MATH_MATRIX3_H */

