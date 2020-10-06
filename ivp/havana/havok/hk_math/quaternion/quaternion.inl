
hk_Quaternion::hk_Quaternion()
{
}

hk_Quaternion::hk_Quaternion(const hk_Rotation& r)
{
	this->set(r);
}

void hk_Quaternion::set_identity()
{
	this->set4(0,0,0,1);
}

hk_Quaternion::hk_Quaternion(const hk_Vector3& axis, hk_real angle)
{
	this->set_axis_angle(axis,angle);
}

void hk_Quaternion::set_mul(hk_real r, const hk_Quaternion& q)
{
	hk_Vector4::set_mul4(r,q);
}
void hk_Quaternion::add_mul(hk_real r, const hk_Quaternion& q)
{
	hk_Vector4::add_mul4(r,q);
}

void hk_Quaternion::set_mul(const hk_Quaternion& q0, const hk_Quaternion& q1)
{
	hk_Vector3::set_cross(q0, q1);
	hk_Vector3::add_mul(q0.w, q1);
	hk_Vector3::add_mul(q1.w, q0);
	this->w = q0.get_real() * q1.get_real() - q0.dot( q1 );
}

void hk_Quaternion::normalize_quaternion()
{
	this->normalize4();	
}

inline hk_real& hk_Quaternion::operator() (int a)
{
	return hk_Vector3::operator()(a);
}

inline const hk_real& hk_Quaternion::operator() (int a) const
{
	return hk_Vector3::operator()(a);
}

inline hk_real hk_Quaternion::get_real() const
{
	return hk_Vector3::operator()(3);
}
inline void hk_Quaternion::set_real(hk_real r)
{
	hk_Vector3::operator()(3) = r;
}


inline const hk_Vector3& hk_Quaternion::get_imag() const
{
	return *this;
}
inline void hk_Quaternion::set_imag(const hk_Vector3& i)
{
	//[ cannot use hk_Vector3::operator=(i), since that may zap imag //]
	this->x = i.x;
	this->y = i.y;
	this->z = i.z;
}

hk_Quaternion::hk_Quaternion(hk_real ix, hk_real iy, hk_real iz, hk_real iw)
{
	this->x = ix;
	this->y = iy;
	this->z = iz;
	this->w = iw;
}
