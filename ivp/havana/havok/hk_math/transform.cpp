#include <hk_math/vecmath.h>

hk_Transform::hk_Transform(const hk_Quaternion &q, const hk_Vector3 &t)
{
	this->set(q);
	m_translation = t;
}

void hk_Transform::set_transform(const hk_Quaternion &q, const hk_Vector3 &t)
{
	this->set(q);
	m_translation = t;
}

void hk_Transform::set_cols4(	const hk_Vector3& c0,
								const hk_Vector3& c1,
								const hk_Vector3& c2,
								const hk_Vector3& c3)
{
	get_column(0) = c0;
	get_column(1) = c1;
	get_column(2) = c2;
	m_translation = c3;
}

void hk_Transform::set_identity_transform()
{
	get_column(0).set(1,0,0);
	get_column(1).set(0,1,0);
	get_column(2).set(0,0,1);
	m_translation.set(0,0,0);
}


void hk_Transform::get_4x4_column_major(hk_real* p) const
{
    //lwss - this is literally only used in one place and it's just a redefine of memcpy
	//hkString::memcpy(p, &get_column(0), 16*sizeof(hk_real));
    memcpy(p, &get_column(0), 16*sizeof(hk_real));
    //lwss end

	p[3]  = 0;
	p[7]  = 0;
	p[11] = 0;
	p[15] = 1;
}


void hk_Transform::set_interpolate( hk_QTransform &a, hk_QTransform &b , hk_real t)
{
	_set_interpolate(a,b,t);
}
