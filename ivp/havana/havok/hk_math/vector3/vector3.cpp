#include <hk_math/vecmath.h>

void hk_Vector3::set_transformed_pos(const hk_Transform& t, const hk_Vector3& v)
{
	_set_transformed_pos(t,v);
}

void hk_Vector3::set_transformed_inv_pos(const hk_Transform& t, const hk_Vector3& v)
{
	_set_transformed_inv_pos(t,v);
}

void hk_Vector3::set_rotated_dir(const hk_Rotation& t, const hk_Vector3& v)
{
	_set_rotated_dir(t,v);
}

void hk_Vector3::set_rotated_inv_dir(const hk_Rotation& t, const hk_Vector3& v)
{
	_set_rotated_inv_dir(t,v);
}

