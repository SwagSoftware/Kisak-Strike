#include <hk_math/vecmath.h>

static const hk_real HK_QUATERNION_DELTA = 1e-3f;

void hk_Quaternion::set(const hk_Rotation& r)
{
	hk_real trace = r(0,0) + r(1,1) + r(2,2);

	// This is an exceptional case:
	// if trace==-1.0 since this means real=sqrt(trace+1) =0.0
	// hence we can't use real to compute the imaginary terms
	// if trace is CLOSE to -1.0, then the normal algorithm is
	// subject to numerical error.
	// Either way, we should use an alternate algorithm.
	// Please see doc "Numerical Problem In Quaterion-Matrix Conversion.doc"
	// in SourceSafe.
	if( trace > 0 )
	{
		// else we calculate simply:
		hk_real s = hk_Math::sqrt( trace + 1.0f );
		hk_real t = 0.5f / s;
		this->x = (r(2,1)-r(1,2)) * t;
		this->y = (r(0,2)-r(2,0)) * t;
		this->z = (r(1,0)-r(0,1)) * t;
		this->w = 0.5f*s;
	}
	else
	{
		const int next[] = {1,2,0};
		hk_real q[4];
		int i=0;

		if(r(1,1) > r(0,0)) i=1;
		if(r(2,2) > r(i,i)) i=2;

		int j = next[i];
		int k = next[j];

		hk_real s = hk_Math::sqrt(r(i,i) - (r(j,j)+r(k,k)) + 1.0f);
		hk_real t = 0.5f / s;

		q[i] = 0.5f * s;
		q[3] = (r(k,j)-r(j,k)) * t;
		q[j] = (r(j,i)+r(i,j)) * t;
		q[k] = (r(k,i)+r(i,k)) * t;

		this->x = q[0];
		this->y = q[1];
		this->z = q[2];
		this->w = q[3];
	}
	//XXX need to renormalize here? havok 1.0 didnt
}

void hk_Quaternion::set_axis_angle(const hk_Vector3& axis, hk_real angle)
{
	HK_ASSERT( hk_Math::fabs(axis.length()-1.0f) < 0.01f);
	hk_real half_angle = 0.5f*angle;
	hk_Vector3::set_mul( hk_Math::sin(half_angle), axis);
	this->w = hk_Math::cos(half_angle);
}

void hk_Quaternion::set_slerp(const hk_Quaternion& q0, const hk_Quaternion& q1, hk_real t)
{
	//HK_ASSERT((t>=0)&&(t<=1));

	hk_real cos_theta = q0.dot4(q1);

	// If B is on the oposite hemisphere use -B instead of B
	hk_real sign_of_t1 = 1.0f;
	if (cos_theta < 0.0f)
	{
		cos_theta = -cos_theta;
		sign_of_t1 = -1.0f;
	}

	if (cos_theta < 1.0f - HK_QUATERNION_DELTA)
	{
		hk_real theta = hk_Math::acos(cos_theta);
		// use sqrt_inv(1+c^2) instead of 1.0/sin(theta) 
		hk_real i_sin_theta = hk_Math::sqrt_inv(1.0f - cos_theta*cos_theta);
		hk_real t_theta = t*theta;

		hk_real t0 = hk_Math::sin(theta-t_theta) * i_sin_theta;
		hk_real t1 = hk_Math::sin(t_theta)       * i_sin_theta;

		this->set_mul4( t0,            q0);
		this->add_mul4( t1*sign_of_t1, q1);
		this->normalize_quaternion();
	}
	else
	{
		// If q0 is nearly the same as q1 we just linearly interpolate
		hk_real t0 = 1.0f - t;
		hk_real t1 = t;

		this->set_mul4( t0,            q0);
		this->add_mul4( t1*sign_of_t1, q1);

		this->normalize_quaternion();
	}	
}

