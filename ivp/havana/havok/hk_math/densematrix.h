#ifndef HK_MATH_DENSEMATRIX_H
#define HK_MATH_DENSEMATRIX_H

#include <hk_math/types.h>
#include <hk_math/vecmath.h>


//: Prototype matrix class for high speed computation.
// This class is subject to change.
class hk_Dense_Matrix
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Dense_Matrix)

		inline hk_Dense_Matrix(hk_real *mem, int r, int c, int lda);

		inline hk_real& operator() (int r, int c) { return m_elt[r+c*m_lda]; }
		inline const hk_real& operator() (int r, int c) const { return m_elt[r + c*m_lda]; }
			//:

		inline void set_zero() { makeZero(); }
		inline int get_num_rows() const { return m_rows; }
		inline int get_num_cols() const { return m_cols; }
		inline int get_lda()     const { return m_lda;  }

		inline void set_size(int r,int c) { m_rows=r; m_cols=c; }

		hk_real* get_real_pointer() { return m_elt; }
		const hk_real* get_const_real_pointer() const { return m_elt; }

		// havok compat XXX remove
		inline void makeZero();
		int getNumRows() const { return m_rows; }
		int getNumCols() const { return m_cols; }
		int getLda()     const { return m_lda;  }
		inline void setNumRows(int r) { m_rows = r; }
		inline void setNumCols(int c) { m_cols = c; }
		
		hk_real* getRealPointer() { return m_elt; }
		const hk_real* getConstRealPointer() const { return m_elt; }

		void mult_vector( hk_real *x_vector, hk_real *result_vector )const; //both vectors are aligned

	protected:

		//inline hk_Dense_Matrix(hk_real *mem, int r, int c, int lda);

	protected:

		hk_real* m_elt;

		int m_rows;
		int m_cols;
		int m_lda;
};


/* Generic fixed size types of dense matrix */

template <int N>
class hk_Fixed_Dense_Matrix : public hk_Dense_Matrix
{
	private:

		hk_real m_elt_buffer[ N * HK_NEXT_MULTIPLE_OF(4,N) ];

	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Fixed_Dense_Matrix<N> )

		inline hk_Fixed_Dense_Matrix()
			: hk_Dense_Matrix(m_elt_buffer, N, N, (N+3)&(~3)) { }

		inline int get_num_rows() const { return N; }
		inline int get_num_cols() const { return N; }
		inline int get_lda()     const { return HK_NEXT_MULTIPLE_OF(4,N);  }

		inline hk_real& operator() (int r, int c) { return m_elt_buffer[r+c * HK_NEXT_MULTIPLE_OF(4,N)]; }
		inline const hk_real& operator() (int r, int c) const { return m_elt_buffer[r + c * HK_NEXT_MULTIPLE_OF(4,N)]; }

		hk_real* get_elems() { return &m_elt_buffer[0]; }
		hk_real* getRealPointer() { return &m_elt_buffer[0]; }
		const hk_real* getConstRealPointer() const { return &m_elt_buffer[0]; }
};


/* Specialized types of dense matrix */

class hk_Dense_Matrix_6x6 : public hk_Dense_Matrix
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Dense_Matrix_6x6 )

		inline hk_Dense_Matrix_6x6()
			: hk_Dense_Matrix( & m_elt_buffer[0], 6, 6, 8 )
		{
		}

	protected:

		hk_real m_elt_buffer[ 6*8 ];
};

class hk_Dense_Matrix_3x3 : public hk_Dense_Matrix
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Dense_Matrix_3x3 )

		inline	hk_Dense_Matrix_3x3 ()
			: hk_Dense_Matrix( m_elt_buffer.get_elem_address(0,0), 3, 3, 4 )
		{
		}

		inline hk_Matrix3& get_matrix3();
		inline void makeZero();

	protected:

		hk_Matrix3 m_elt_buffer;
};



class hk_Dense_Matrix_1x1 : public hk_Dense_Matrix
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Dense_Matrix_1x1 )

		inline	hk_Dense_Matrix_1x1 ()
			: hk_Dense_Matrix( & m_elt_buffer[0], 1, 1, 4 )
		{
		}

		inline void makeZero();

	protected:

		hk_real m_elt_buffer[ 1*4 ];
};

/* Runtime size matrix */

class hk_Dynamic_Dense_Matrix : public hk_Dense_Matrix
{
	public:

		hk_Dynamic_Dense_Matrix(int r, int c);

		~hk_Dynamic_Dense_Matrix();
};

#include <hk_math/densematrix.inl>

#endif /* HK_MATH_DENSEMATRIX_H */

