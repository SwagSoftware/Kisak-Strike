
hk_Vector4::hk_Vector4()
{
}

 hk_Vector4::hk_Vector4(hk_real a, hk_real b, hk_real c, hk_real d)
{
	x = a;
	y = b;
	z = c;
	w = d;
}

void hk_Vector4::operator=(const hk_Vector4& v)
{
	x = v.x;
	y = v.y;
	z = v.z;
	w = v.w;
}

 void hk_Vector4::operator+= (const hk_Vector4& v)
{
	hk_real a = x + v.x;
	hk_real b = y + v.y;
	hk_real c = z + v.z;
	hk_real d = w + v.w;
	x = a;
	y = b;
	z = c;
	w = d;
}

 void hk_Vector4::operator-= (const hk_Vector4& v)
{
	hk_real a = x - v.x;
	hk_real b = y - v.y;
	hk_real c = z - v.z;
	hk_real d = w - v.w;
	x = a;
	y = b;
	z = c;
	w = d;
}

 void hk_Vector4::operator*= (hk_real r)
{
	hk_real a = r * x;
	hk_real b = r * y;
	hk_real c = r * z;
	hk_real d = r * w;
	x = a;
	y = b;
	z = c;
	w = d;
}

 void hk_Vector4::set4(hk_real a, hk_real b, hk_real c, hk_real d)
{
	x = a;
	y = b;
	z = c;
	w = d;
}
 void hk_Vector4::set_zero4()
{
	x = y = z = w = 0;
}

 void hk_Vector4::set_add4(const hk_Vector4& v0, const hk_Vector4& v1)
{
	hk_real a = v0.x + v1.x;
	hk_real b = v0.y + v1.y;
	hk_real c = v0.z + v1.z;
	hk_real d = v0.w + v1.w;
	x = a;
	y = b;
	z = c;
	w = d;
}

 void hk_Vector4::set_sub4(const hk_Vector4& v0, const hk_Vector4& v1)
{
	hk_real a = v0.x - v1.x;
	hk_real b = v0.y - v1.y;
	hk_real c = v0.z - v1.z;
	hk_real d = v0.w - v1.w;
	x = a;
	y = b;
	z = c;
	w = d;
}

 void hk_Vector4::set_mul4(hk_real r, const hk_Vector4& v)
{
	hk_real a = r * v.x;
	hk_real b = r * v.y;
	hk_real c = r * v.z;
	hk_real d = r * v.w;
	x = a;
	y = b;
	z = c;
	w = d;
}

 void hk_Vector4::add_mul4(hk_real r, const hk_Vector4& v)
{
	hk_real a = x + r * v.x;
	hk_real b = y + r * v.y;
	hk_real c = z + r * v.z;
	hk_real d = w + r * v.w;
	x = a;
	y = b;
	z = c;
	w = d;
}


 hk_real hk_Vector4::dot4(const hk_Vector4& a) const
{
	return x*a.x + y*a.y + z*a.z + w*a.w;
}

 hk_real hk_Vector4::length4() const
{
	return hk_Math::sqrt_inv(x*x + y*y + z*z + w*w);
}

 hk_real hk_Vector4::length_inv4() const
{
	return hk_Math::sqrt_inv(x*x + y*y + z*z + w*w);
}

 hk_real hk_Vector4::length_squared4() const
{
	return x*x + y*y + z*z + w*w;
}

 void hk_Vector4::normalize4()
{
	*this *= length_inv4();
}

