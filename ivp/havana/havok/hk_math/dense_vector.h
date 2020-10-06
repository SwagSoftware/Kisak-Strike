#ifndef HK_MATH_DENSE_VECTOR_H
#define HK_MATH_DENSE_VECTOR_H

class hk_Dense_Vector
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Dense_Vector)

		inline hk_real& operator() (int i) { HK_ASSERT(i<m_size); return m_elt[i]; }
		inline const hk_real& operator() (int i) const { HK_ASSERT(i<m_size); return m_elt[i]; }

		const hk_real* get_const_real_pointer() const { return m_elt; }
		hk_real* get_real_pointer() { return m_elt; }

		int  get_size() const { return m_size; }
		void set_size(int size)
		{
			HK_ASSERT(size <= m_capacity);
			m_size = size;
		}

		inline void set_zero()
		{
			int i = m_size - 1;
			do {
				m_elt[i] = 0;
				i--;
			} while ( i>= 0);
		}

	protected:

		inline hk_Dense_Vector(hk_real* e, int s, int c)
			: m_elt(e), m_size(s), m_capacity(c) { }

		hk_real* m_elt;

		int m_size;
		int m_capacity;
};

class hk_Preallocated_Dense_Vector : public hk_Dense_Vector
{
	public:
		hk_Preallocated_Dense_Vector(hk_real* e, int n, int c)
			: hk_Dense_Vector(e, n, c) { }

};

template <int N>
class hk_Fixed_Dense_Vector : public hk_Dense_Vector
{
	public:

		HK_DECLARE_NONVIRTUAL_CLASS_ALLOCATOR(HK_MEMORY_CLASS_CONSTRAINT, hk_Fixed_Dense_Vector<N> )

		hk_Fixed_Dense_Vector()
			: hk_Dense_Vector(m_elt_buffer, N, HK_NEXT_MULTIPLE_OF(4,N) ) { }

	// SK says XXX fix me
		inline void set_add_mul( const hk_Fixed_Dense_Vector<N> &a, hk_real factor, const hk_real b[]);
		inline void set_mul_add_mul( hk_real factor_a, const hk_Fixed_Dense_Vector<N> &a, hk_real factor_b, const hk_real b[]);

	private:
		hk_real dummy;
		hk_real m_elt_buffer[ HK_NEXT_MULTIPLE_OF(4,N) ];
};

class hk_Dynamic_Dense_Vector : public hk_Dense_Vector
{
	public:

		hk_Dynamic_Dense_Vector(int n);

		~hk_Dynamic_Dense_Vector();
};

#include <hk_math/dense_vector.inl>

#endif /*HK_MATH_DENSE_VECTOR_H*/

