hk_QTransform::hk_QTransform()
{
}

hk_QTransform::hk_QTransform(const hk_Vector3& axis, hk_real angle,
							const hk_Vector3& translate)
	: m_rotation(axis,angle), m_translation(translate)
{
}

void hk_QTransform::set_identity()
{
	m_rotation.set_identity();
	m_translation.set_zero();
}

void hk_QTransform::add_mul( hk_real r, const hk_QTransform& t)
{
	m_rotation.add_mul(r, t.m_rotation);
	m_translation.add_mul( r, t.m_translation);
}

const hk_Vector3& hk_QTransform::get_translation() const
{
	return m_translation;
}

const hk_Quaternion& hk_QTransform::get_rotation() const
{
	return m_rotation;
}


void hk_QTransform::set_translation(const hk_Vector3& p)
{
	m_translation = p;
}

void hk_QTransform::set_interpolate(	const hk_QTransform& a,
										const hk_QTransform& b,
										hk_real t)
{
	m_rotation.set_slerp( a.m_rotation, b.m_rotation, t);
	m_translation.set_interpolate(a.m_translation, b.m_translation, t);
}
