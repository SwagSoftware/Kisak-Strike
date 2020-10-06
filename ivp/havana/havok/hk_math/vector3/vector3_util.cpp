#include <hk_math/vecmath.h>
#include <hk_math/vector3/vector3_util.h>

hk_Vector3 hk_Vector3_Util::random_vec()
{
	hk_Vector3 result;
	result.x = hk_Math::rand01();
	result.y = hk_Math::rand01();
	result.z = hk_Math::rand01();

	return result;
}

hk_Vector3 hk_Vector3_Util::perp_vec( const hk_Vector3 &dir )
{
	hk_Vector3 res;

	if ( hk_Math::fabs(dir.x) < 0.57f )
	{
		res.set_cross( hk_Vector3(1,0,0), dir );
	}
	else
	{
		res.set_cross( hk_Vector3(0,1,0), dir );
	}
	res.normalize();
	return res;
}

