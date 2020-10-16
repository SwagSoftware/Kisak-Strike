
/* constructor / assignment */

#include <hk_math/spatial_matrix.h>

hk_Spatial_Vector::hk_Spatial_Vector()
{
}

hk_Spatial_Vector::hk_Spatial_Vector(hk_real a, hk_real b, hk_real c, hk_real d, hk_real e, hk_real f)
	: top(a,b,c), bottom(d,e,f)
{
}

hk_Spatial_Vector::hk_Spatial_Vector(hk_Vector3 &a, hk_Vector3 &b )
	: top(a.x, a.y, a.z), bottom(b.x, b.y, b.z)
{
}

hk_Spatial_Vector::hk_Spatial_Vector( const hk_Spatial_Vector& v)
{
	*this = v;
}

/* accumulation operators - fast! */

hk_Spatial_Vector& hk_Spatial_Vector::operator= (const hk_Spatial_Vector& v)
{
	top.x = v.top.x;
	top.y = v.top.y;
	top.z = v.top.z;
	bottom.x = v.bottom.x;
	bottom.y = v.bottom.y;
	bottom.z = v.bottom.z;
	return *this;
}

void hk_Spatial_Vector::operator+= (const hk_Spatial_Vector& v)
{
	top.x += v.top.x;
	top.y += v.top.y;
	top.z += v.top.z;
	bottom.x += v.bottom.x;
	bottom.y += v.bottom.y;
	bottom.z += v.bottom.z;
}

void hk_Spatial_Vector::operator-= (const hk_Spatial_Vector& v)
{
	top.x -= v.top.x;
	top.y -= v.top.y;
	top.z -= v.top.z;
	bottom.x -= v.bottom.x;
	bottom.y -= v.bottom.y;
	bottom.z -= v.bottom.z;
}

void hk_Spatial_Vector::operator*= ( hk_real a )
{
	top.x *= a;
	top.y *= a;
	top.z *= a;
	bottom.x *= a;
	bottom.y *= a;
	bottom.z *= a;
}


void hk_Spatial_Vector::set(hk_real a, hk_real b, hk_real c, hk_real d, hk_real e, hk_real f)
{
	top.x = a;	
	top.y = b;	
	top.z = c;	
	bottom.x = d;
	bottom.y = e;
	bottom.z = f;
}

void hk_Spatial_Vector::set_top( const hk_Vector3 &a )
{
	top = a;
}

void hk_Spatial_Vector::set_bottom( const hk_Vector3 &b )
{
	bottom = b;
}

void hk_Spatial_Vector::set_zero()
{
	top.x = 0;	
	top.y = 0;	
	top.z = 0;	
	bottom.x = 0;
	bottom.y = 0;
	bottom.z = 0;
}

/* arithmetic operations - fast! */

void hk_Spatial_Vector::set_add(const hk_Spatial_Vector& a, const hk_Spatial_Vector& b)
{
	top.x = a.top.x + b.top.x;
	top.y = a.top.y + b.top.y;
	top.z = a.top.z + b.top.z;
	bottom.x = a.bottom.x + b.bottom.x;
	bottom.y = a.bottom.y + b.bottom.y;
	bottom.z = a.bottom.z + b.bottom.z;
}

void hk_Spatial_Vector::set_sub(const hk_Spatial_Vector& a, const hk_Spatial_Vector& b)
{
	top.x = a.top.x - b.top.x;
	top.y = a.top.y - b.top.y;
	top.z = a.top.z - b.top.z;
	bottom.x = a.bottom.x - b.bottom.x;
	bottom.y = a.bottom.y - b.bottom.y;
	bottom.z = a.bottom.z - b.bottom.z;
}

void hk_Spatial_Vector::set_mul(hk_real r, const hk_Spatial_Vector& a)
{
	top.x = a.top.x * r;
	top.y = a.top.y * r;
	top.z = a.top.z * r;
	bottom.x = a.bottom.x * r;
	bottom.y = a.bottom.y * r;
	bottom.z = a.bottom.z * r;	
}

void hk_Spatial_Vector::add_mul(hk_real r, const hk_Spatial_Vector& v)
{
	top.x += v.top.x*r;
	top.y += v.top.y*r;
	top.z += v.top.z*r;
	bottom.x += v.bottom.x*r;
	bottom.y += v.bottom.y*r;
	bottom.z += v.bottom.z*r;
}

/* arithmetic operators - slow! */
hk_Spatial_Vector hk_Spatial_Vector::operator+ (const hk_Spatial_Vector& v)
{
	return hk_Spatial_Vector( top.x + v.top.x, top.y + v.top.y, top.z + v.top.z,
							bottom.x + v.bottom.x, bottom.y + v.bottom.y, bottom.z + v.bottom.z );

}

hk_Spatial_Vector hk_Spatial_Vector::operator- (const hk_Spatial_Vector& v)
{
	return hk_Spatial_Vector( top.x - v.top.x, top.y - v.top.y, top.z - v.top.z,
							bottom.x - v.bottom.x, bottom.y - v.bottom.y, bottom.z - v.bottom.z );

}

// spatial dot product.  
hk_real hk_Spatial_Vector::dot(const hk_Spatial_Vector& a) const
{
	return ( bottom.x*a.top.x + bottom.y*a.top.y + bottom.z*a.top.z +
              top.x*a.bottom.x + top.y*a.bottom.y + top.z*a.bottom.z );
}


void hk_Spatial_Vector::_set_transformed_pos (const hk_Spatial_Matrix& T, const hk_Spatial_Vector& v)
{

#ifdef HK_NO_VECTOR_UNIT

	hk_real *element = &top.x;

	for( int i = 0; i < 6; i++ ){
		*(element + i) = T( i, 0 )*v.top.x    + T( i, 1 )*v.top.y    + T( i, 2 )*v.top.z +
						 T( i, 3 )*v.bottom.x + T( i, 4 )*v.bottom.y + T( i, 5 )*v.bottom.z;
	}

#else

	hk_Vector3 work_a, work_b;

	work_a.set_mul3( T.m_Block[0][0], v.top );
	work_b.set_mul3( T.m_Block[0][1], v.bottom );
	top.set_add( work_a, work_b );
	work_a.set_mul3( T.m_Block[1][0], v.top );
	work_b.set_mul3( T.m_Block[1][1], v.bottom );
	bottom.set_add( work_a, work_b );

#endif
} 

/* element access */
hk_real& hk_Spatial_Vector::operator() (int a)
{
	if( a < 3 )
		return top(a);
	else
		return bottom(a-3);
}

const hk_real& hk_Spatial_Vector::operator() (int a) const
{
	if( a < 3 )
		return top(a);
	else
		return bottom(a-3);
}



