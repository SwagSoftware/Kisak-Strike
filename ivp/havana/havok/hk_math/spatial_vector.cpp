#include <hk_math/vecmath.h>
#include <hk_math/spatial_vector.h>

void hk_Spatial_Vector::set_transformed_pos (const hk_Spatial_Matrix& T, const hk_Spatial_Vector& v)
{
	_set_transformed_pos( T, v );
}

void hk_Spatial_Vector::set_mul(const hk_Spatial_Matrix& T, const hk_Spatial_Vector& v)
{
	_set_transformed_pos( T, v );
}

