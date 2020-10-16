
void hk_Ray::set_zero()
{
    m_origin.set_zero();
    m_direction.set_zero();
}

void hk_Ray::set(const hk_Vector3& o, const hk_Vector3& d)
{
    m_origin = o;
    m_direction = d;
}
