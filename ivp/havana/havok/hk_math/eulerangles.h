#ifndef HK_MATH_EULERANGLES_H
#define HK_MATH_EULERANGLES_H

// <hk_math/vector3/vector3.h>

//: Rotation represented by three rotation angles about the xyz axes.
class hk_Euler_Angles : public hk_Vector3
{
	public:

		//:
		enum hk_EULER_ANGLES_ORDER
		{
			HK_ORDER_XYZ,
			HK_ORDER_XZY
			//XXX fill me in
		};

		//void set_euler_angles( const hk_Rotation& r, hk_euler_angles_order o);
			//:
		void set_euler_angles( const hk_Quaternion& q, hk_EULER_ANGLES_ORDER o);

		static void create_rot_axis_and_angles(
			 const hk_Rotation &tref_ws_os, const hk_Rotation &tatt_ws_os, 
			 int i0, int i1, int i2,
			 // in
			 hk_Matrix3 &taxis_ws_os_out, hk_Vector3 &angles_out);
			 //out 

};

#endif /* HK_MATH_EULERANGLES_H */
