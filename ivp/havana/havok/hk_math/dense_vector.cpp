#include <hk_math/vecmath.h>
#include <hk_math/dense_vector.h>

hk_Dynamic_Dense_Vector::hk_Dynamic_Dense_Vector(int n)
	: hk_Dense_Vector(0, n, HK_NEXT_MULTIPLE_OF(4,n)) 
{
	m_elt = hk_allocate( hk_real, HK_NEXT_MULTIPLE_OF(4,n), HK_MEMORY_CLASS_DENSE_VECTOR );
}

hk_Dynamic_Dense_Vector::~hk_Dynamic_Dense_Vector()
{
	hk_deallocate( hk_real, m_elt, m_capacity, HK_MEMORY_CLASS_DENSE_VECTOR );
}
