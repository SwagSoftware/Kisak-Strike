#ifdef HK_PS2
#	include <hk_math/vector3/vector3ps2.inl>
#else	//HK_PS2

/* constructor / assignment */

inline hk_Vector3::hk_Vector3()
{
}

inline hk_Vector3::hk_Vector3(hk_real a, hk_real b, hk_real c)
	: x(a), y(b), z(c)
{
}

inline hk_Vector3::hk_Vector3(const double* d)
	: x(hk_real(d[0])), y(hk_real(d[1])), z(hk_real(d[2]))
{
}

inline hk_Vector3::hk_Vector3(const float* d)
	: x(hk_real(d[0])), y(hk_real(d[1])), z(hk_real(d[2]))
{
}

inline hk_Vector3::hk_Vector3( const hk_Vector3& v)
{
	*this = v;
}

inline void hk_Vector3::operator= (const hk_Vector3& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
}



void hk_Vector3::operator=  (const class Havok::Vector3&v)
{
	*this = ((const hk_Vector3 *)&v)[0];
}

/* expression templates */

struct hkVector3ExpressionPlus
{
	hkVector3ExpressionPlus(const hk_Vector3& a, const hk_Vector3& b) : m_a(a), m_b(b) { }
	const hk_Vector3& m_a;
	const hk_Vector3& m_b;
};
struct hkVector3ExpressionMinus
{
	hkVector3ExpressionMinus(const hk_Vector3& a, const hk_Vector3& b) : m_a(a), m_b(b) { }
	const hk_Vector3& m_a;
	const hk_Vector3& m_b;
};

inline void hk_Vector3::operator= (ConsT hkVector3ExpressionPlus HK_REF v)
{
	x = v.m_a.x + v.m_b.x;
	y = v.m_a.y + v.m_b.y;
	z = v.m_a.z + v.m_b.z;
}
inline hk_Vector3::hk_Vector3(ConsT hkVector3ExpressionPlus HK_REF v) { operator= (v); }

inline void hk_Vector3::operator= (ConsT hkVector3ExpressionMinus HK_REF v)
{
	x = v.m_a.x - v.m_b.x;
	y = v.m_a.y - v.m_b.y;
	z = v.m_a.z - v.m_b.z;
}
inline hk_Vector3::hk_Vector3(ConsT hkVector3ExpressionMinus HK_REF  v) { operator= (v); }

void hk_Vector3::operator+= (const hk_Vector3& v)
{
	hk_real a = x + v.x;
	hk_real b = y + v.y;
	hk_real c = z + v.z;
	x = a;
	y = b;
	z = c;
}

void hk_Vector3::operator-= (const hk_Vector3& v)
{
	hk_real a = x - v.x;
	hk_real b = y - v.y;
	hk_real c = z - v.z;
	x = a;
	y = b;
	z = c;
}

void hk_Vector3::operator*= (hk_real s)
{              
	hk_real a = x * s;
	hk_real b = y * s;
	hk_real c = z * s;
	x = a;
	y = b;
	z = c;
}


void hk_Vector3::set_mul3( const hk_Diagonal_Matrix& a, const hk_Vector3& b )
{
	hk_real ax = a.x * b.x;
	hk_real ay = a.y * b.y;
	hk_real az = a.z * b.z;
	x = ax;
	y = ay;
	z = az;
}

inline void hk_Vector3::set(hk_real a, hk_real b, hk_real c)
{
	x = a;
	y = b;
	z = c;
}

inline void hk_Vector3::set_zero()
{
	x = y = z = 0;
}

void hk_Vector3::set_add(const hk_Vector3& v0, const hk_Vector3& v1)
{
	hk_real a = v0.x + v1.x;
	hk_real b = v0.y + v1.y;
	hk_real c = v0.z + v1.z;
	x = a;
	y = b;
	z = c;
}

void hk_Vector3::set_interpolate(const hk_Vector3& v0, const hk_Vector3& v1, hk_real t)
{
	hk_real s = 1.0f - t;
	hk_real a = s * v0.x + t * v1.x;
	hk_real b = s * v0.y + t * v1.y;
	hk_real c = s * v0.z + t * v1.z;
	x = a;
	y = b;
	z = c;
}

void hk_Vector3::set_sub(const hk_Vector3& v0, const hk_Vector3& v1)
{
	hk_real a = v0.x - v1.x;
	hk_real b = v0.y - v1.y;
	hk_real c = v0.z - v1.z;
	x = a;
	y = b;
	z = c;
}

void hk_Vector3::set_cross(const hk_Vector3& v1, const hk_Vector3& v2)
{
	hk_real a  = v1(1)*v2(2);
    hk_real a2 = v2(1)*v1(2);
    hk_real b  = v1(2)*v2(0);	
    hk_real b2 = v2(2)*v1(0);	a -= a2;
    hk_real c  = v1(0)*v2(1);
    hk_real c2 = v2(0)*v1(1);	b -= b2;
    x = a;
	y = b;						c -= c2;
    z = c;  
}

void hk_Vector3::set_mul(hk_real r, const hk_Vector3& a)
{
	x = r*a.x;
	y = r*a.y;
	z = r*a.z;
}

void hk_Vector3::add_mul(hk_real r, const hk_Vector3& a)
{
	x += r*a.x;
	y += r*a.y;
	z += r*a.z;
}

void hk_Vector3::set_mul3( const hk_Matrix3& r, const hk_Vector3& v )
{
    hk_real a = r(0,0)*v(0) + r(0,1)*v(1) + r(0,2)*v(2);
    hk_real b = r(1,0)*v(0) + r(1,1)*v(1) + r(1,2)*v(2);
    hk_real c = r(2,0)*v(0) + r(2,1)*v(1) + r(2,2)*v(2);
	x = a;
	y = b;
	z = c;
}


/**** friend operators *** */

inline hk_Vector3 operator* (hk_real s, const hk_Vector3& v)
{
	return hk_Vector3(s*v.x, s*v.y, s*v.z);
}


/*** transforms ***/

void hk_Vector3::_set_rotated_dir(const hk_Rotation& r, const hk_Vector3& v)
{
	set_mul3( r, v );
}


void hk_Vector3::_set_transformed_pos(const hk_Transform& r, const hk_Vector3& v)
{
#if 0 /* a not working try to use inline assembler */
	__asm {
            mov         eax,r
			mov         ecx,v
            movaps      xmm1,xmmword ptr [eax + 0h]
            movaps      xmm2,xmmword ptr [eax + 10h]
            movaps      xmm3,xmmword ptr [eax + 20h]
            movaps      xmm4,xmmword ptr [eax + 30h]

			movaps      xmm5,xmmword ptr [ecx]
			movaps		xmm6,xmm5
			movaps		xmm7,xmm5

			shufps      xmm5,xmm5,0
			shufps      xmm6,xmm6,55h
			shufps      xmm7,xmm7,0AAh

			mulps       xmm5,xmm1
			mulps       xmm6,xmm2
			mulps       xmm7,xmm3

			mov			eax, this
			addps       xmm7,xmm5
			addps       xmm7,xmm6
			addps       xmm5,xmm4
			movaps      xmmword ptr [eax],xmm7
	}

#elif 1 && defined(PIII_SIMD_ENABLE)
	const hk_real *pm = &r.get_column(0).x;
	//__m128 c0 = _mm_load_ps( &r.get_column(0).x );
	__m128 vec = _mm_load_ps( &v.x );

	__m128 v0 = _mm_shuffle_ps( vec, vec, _MM_SHUFFLE( 0,0,0,0) );
	__m128 c0 = _mm_load_ps( pm );
	__m128 c1 = _mm_load_ps( pm + 4 );	
	__m128 c2 = _mm_load_ps( pm + 8 );
	__m128 c3 = _mm_load_ps( pm + 12 );

	c0 = _mm_mul_ps( v0, c0 );

	__m128 v1 = _mm_shuffle_ps( vec, vec, _MM_SHUFFLE( 1,1,1,1) );
	c1 = _mm_mul_ps( v1, c1 );
	c0 = _mm_add_ps( c0, c1 );

	__m128 v2 = _mm_shuffle_ps( vec, vec, _MM_SHUFFLE( 2,2,2,2) );
	c2 = _mm_mul_ps( v2, c2 );

	c2 = _mm_add_ps( c2, c3 );

	c0 = _mm_add_ps( c0, c2 );

	_mm_store_ps( &x, c0 );
#else
    hk_real a = r(0,0)*v(0) + r(0,1)*v(1) + r(0,2)*v(2) + r(0,3);
    hk_real b = r(1,0)*v(0) + r(1,1)*v(1) + r(1,2)*v(2) + r(1,3);
    hk_real c = r(2,0)*v(0) + r(2,1)*v(1) + r(2,2)*v(2) + r(2,3);

	x = a;
	y = b;
	z = c;
#endif
}


void hk_Vector3::_set_rotated_inv_dir(const hk_Rotation& r, const hk_Vector3& v)
{
    hk_real a = r(0,0)*v(0) + r(1,0)*v(1) + r(2,0)*v(2);
    hk_real b = r(0,1)*v(0) + r(1,1)*v(1) + r(2,1)*v(2);
    hk_real c = r(0,2)*v(0) + r(1,2)*v(1) + r(2,2)*v(2);
	x = a;
	y = b; 
	z = c;
}

void hk_Vector3::_set_transformed_inv_pos(const hk_Transform& r, const hk_Vector3& v)
{
    hk_real v0 = v(0) - r(0,3);
    hk_real v1 = v(1) - r(1,3);
    hk_real v2 = v(2) - r(2,3);

    hk_real a = r(0,0)*v0 + r(1,0)*v1 + r(2,0)*v2;
    hk_real b = r(0,1)*v0 + r(1,1)*v1 + r(2,1)*v2;
    hk_real c = r(0,2)*v0 + r(1,2)*v1 + r(2,2)*v2;
	x = a;
	y = b; 
	z = c;
}

hk_real hk_Vector3::dot(const hk_Vector3& a) const
{
	return (x * a.x) + ( y * a.y ) + ( z * a.z );
}


hk_real hk_Vector3::length() const
{
	return hk_Math::sqrt((x * x) + ( y * y ) + ( z * z ));
}

hk_real hk_Vector3::length_inv() const
{
	hk_real l2 = (x*x) + (y*y) + (z*z);
	return l2 ? hk_Math::sqrt_inv(l2) : 0;
}

hk_real hk_Vector3::length_squared() const
{
	return (x * x) + ( y * y ) + ( z * z );
}


hk_real hk_Vector3::distance_squared_to( const hk_Vector3& v) const
{
	hk_real a = x - v.x;
	hk_real b = y - v.y;
	hk_real c = z - v.z;
	return (a * a) + (b * b) + (c * c);
}

hk_real hk_Vector3::distance_to( const hk_Vector3& v) const
{
	return hk_Math::sqrt( distance_squared_to(v));
}

void hk_Vector3::normalize()
{
	*this *= this->length_inv();
}

hk_real hk_Vector3::normalize_with_length()
{
	hk_real len = this->length();
	*this *= (1.0f)/len;
	return len;
}

hk_real& hk_Vector3::operator() (int a)
{
	return (&x)[a];
}

const hk_real& hk_Vector3::operator() (int a) const
{
	return (&x)[a];
}

/* expressions */

inline Const hkVector3ExpressionPlus operator+ (const hk_Vector3& a, const hk_Vector3& b)
{
	return hkVector3ExpressionPlus(a,b);
}
inline Const hkVector3ExpressionMinus operator- (const hk_Vector3& a, const hk_Vector3& b)
{
	return hkVector3ExpressionMinus(a,b);
}

#endif //HK_PS2

