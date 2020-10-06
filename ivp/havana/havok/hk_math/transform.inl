
hk_Transform::hk_Transform()
{
}

hk_Vector3& hk_Transform::get_translation()
{
	return m_translation;
}

const hk_Vector3& hk_Transform::get_translation() const
{
	return m_translation;
}

void hk_Transform::set_translation(const hk_Vector3& t0)
{
	m_translation = t0;
}


void hk_Transform::set_qtransform( const hk_QTransform& q )
{
	hk_Rotation::set(q.m_rotation);
	m_translation = q.m_translation;
}

void hk_Transform::set_rotation( const hk_Quaternion& q )
{
	hk_Rotation::set(q);
}


void hk_Transform::_set_interpolate( hk_QTransform &a, hk_QTransform &b , hk_real t)
{
	hk_Quaternion qt; 
	qt.set_slerp(	a.get_rotation(), b.get_rotation(), t );
	this->set_rotation(qt);
	this->m_translation.set_interpolate( a.get_translation(), b.get_translation(), t );
}

