
void hk_Car_Wheel_Constraint::set_wheel_motor( hk_real angular_velocity, hk_real max_force)
{
	m_wheel_limit.set_motor( angular_velocity, max_force );

}
