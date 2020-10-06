#ifndef HK_SPATIAL_MATRIX_H
#define HK_SPATIAL_MATRIX_H

#include <hk_math/vecmath.h>

class hk_Spatial_Vector;

class hk_Spatial_Matrix
{
	public:

		inline hk_Spatial_Matrix();
			//: construct an uninitialized spatial matrix

		void set_spatial_inertia_tensor( const hk_Matrix3& inertia_tensor, const hk_real mass );
			//: form the spatial inertia tensor from a cartesian inertia tensor and mass

		void set_spatial_transform( const hk_Matrix3& R, const hk_Vector3& r );
			//: form a spatial transform between frames F and G from a rotation matrix
			// taking a vector from F to G and an offset vector from the center of F to G.  
			// the offset vector must be in G's frame of reference

		void set_spatial_transform( const hk_Transform& cartesian_frame );
			//: form a spatial tranform between frames F and G from a cartesian transform
			// between F and G

		void set_identity();
			//: set diagonal elements of the 6x6 matrix to 1
		
		void set_mul( hk_Spatial_Matrix& a, hk_Spatial_Matrix& b );
			//: mulitply two spatial matrixes together
		
		void set_vector_mul_vector( const hk_Spatial_Vector &a, const hk_Spatial_Vector &b );
			//: M = a*!b    where !b = spatial transpose

		void set_add( hk_Spatial_Matrix& a, hk_Spatial_Matrix& b );
			//: add two spatial matrixes together

		void set_sub( hk_Spatial_Matrix& a, hk_Spatial_Matrix& b );
			//: subtract two spatial matrixes

		inline hk_real operator() (int r, int c) const;
			//: Get element (row, column)
			// No range checking is done.

		void linear_solve( hk_Spatial_Vector &x, const hk_Spatial_Vector &b );

		void operator +=( const hk_Spatial_Matrix& a );
		void operator -=( const hk_Spatial_Matrix& a );
		void operator *=( hk_real a );

	private:
	
		//!me temporary linear system solver
		void gausse_siedel_solve( hk_real A[][6], int dim, hk_real *x, hk_real *b );

		hk_Matrix3 m_Block[2][2];

	private:
		friend class hk_Spatial_Vector;
};

#include <hk_math/spatial_matrix.inl>

#endif //HK_SPATIAL_MATRIX_H
