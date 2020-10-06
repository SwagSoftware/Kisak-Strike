// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#define IVP_QUAT_DELTA 1e-6f     // error tolerance

void IVP_U_Quat::get_angles(IVP_U_Float_Point *angles_out)
{
  IVP_DOUBLE f = 2.0f;
  angles_out->k[0] = f * IVP_Inline_Math::asind(x);
  angles_out->k[1] = f * IVP_Inline_Math::asind(y);
  angles_out->k[2] = f * IVP_Inline_Math::asind(z);
}

void IVP_U_Quat::set_fast_multiple(const  IVP_U_Point *angles, IVP_DOUBLE factor){
	IVP_DOUBLE f = factor * .5f;
	x = IVP_Inline_Math::sind(angles->k[0]*f);
	y = IVP_Inline_Math::sind(angles->k[1]*f);
	z = IVP_Inline_Math::sind(angles->k[2]*f);

	IVP_DOUBLE n = x*x + y*y + z*z; //@@CB
	w = IVP_Inline_Math::sqrtd(1.0f - ((n > 1.0f) ? 1.0f : n)); //@@CB
//	w = IVP_Inline_Math::sqrtd(1.0f - n);
}

inline IVP_FLOAT ivp_very_fast_sin(IVP_FLOAT x){
    return x - x*x*x * (1.0f / 6.0f);
}




void IVP_U_Quat::set_very_fast_multiple(const  IVP_U_Float_Point *angles, IVP_DOUBLE factor){
  IVP_DOUBLE f = factor * .5f;
  x = ivp_very_fast_sin(angles->k[0]*f);
  y = ivp_very_fast_sin(angles->k[1]*f);
  z = ivp_very_fast_sin(angles->k[2]*f);
  w = IVP_Inline_Math::sqrtd(1.0f - (x*x + y*y + z*z));

}

void IVP_U_Quat::set_fast_multiple_with_clip(const  IVP_U_Float_Point *angles, IVP_DOUBLE factor){
  IVP_DOUBLE f = factor * .5f;
  x = IVP_Inline_Math::ivp_sinf(angles->k[0]*f);
  y = IVP_Inline_Math::ivp_sinf(angles->k[1]*f);
  z = IVP_Inline_Math::ivp_sinf(angles->k[2]*f);

  IVP_DOUBLE qlen = x*x + y*y + z*z;
  if (qlen > 1.0f){ // reverse quat needed
      IVP_DOUBLE ilen = (1.0f - P_RES_EPS) / IVP_Inline_Math::sqrtd(qlen);
      x *= ilen;
      y *= ilen;
      z *= ilen;
      qlen =  x*x + y*y + z*z;
  }
  w = IVP_Inline_Math::sqrtd(1.0f - qlen);
}

void IVP_U_Quat::set(IVP_DOUBLE rot_x, IVP_DOUBLE rot_y, IVP_DOUBLE rot_z)
{
  IVP_DOUBLE f = .5f;
  x = IVP_Inline_Math::sind(rot_x*f);
  y = IVP_Inline_Math::sind(rot_y*f);
  z = IVP_Inline_Math::sind(rot_z*f);
  w = IVP_Inline_Math::sqrtd(1.0f - (x*x + y*y + z*z));
}


void IVP_U_Quat::set_matrix(IVP_DOUBLE m[4][4]) const {
  const IVP_U_Quat *quat=this;
  IVP_DOUBLE wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

  x2 = quat->x + quat->x; y2 = quat->y + quat->y; z2 = quat->z + quat->z;
  xx = quat->x * x2;   xy = quat->x * y2;   xz = quat->x * z2;
  yy = quat->y * y2;   yz = quat->y * z2;   zz = quat->z * z2;
  wx = quat->w * x2;   wy = quat->w * y2;   wz = quat->w * z2;

  m[0][0] = 1.0f - (yy + zz);
  m[0][1] = xy - wz;
  m[0][2] = xz + wy;
  m[0][3] = 0.0f;
 
  m[1][0] = xy + wz;
  m[1][1] = 1.0f - (xx + zz);
  m[1][2] = yz - wx;
  m[1][3] = 0.0f;

  m[2][0] = xz - wy;
  m[2][1] = yz + wx;
  m[2][2] = 1.0f - (xx + yy);
  m[2][3] = 0.0f;

  m[3][0] = 0.0f;
  m[3][1] = 0.0f;
  m[3][2] = 0.0f;
  m[3][3] = 1.0f;    
}

void IVP_U_Quat::set_matrix(IVP_U_Matrix3 *mat)const  {
#if !defined(IVP_USE_PS2_VU0_)
  const IVP_U_Quat *quat=this;
  IVP_DOUBLE wx, wy, wz, xx, yy, yz, xy, xz, zz, x2, y2, z2;

  x2 = quat->x + quat->x; y2 = quat->y + quat->y; z2 = quat->z + quat->z;
  xx = quat->x * x2;   xy = quat->x * y2;   xz = quat->x * z2;
  yy = quat->y * y2;   yz = quat->y * z2;   zz = quat->z * z2;
  wx = quat->w * x2;   wy = quat->w * y2;   wz = quat->w * z2;

  mat->set_elem(0,0, 1.0f - (yy + zz));
  mat->set_elem(0,1, xy - wz);
  mat->set_elem(0,2, xz + wy);
 
  mat->set_elem(1,0, xy + wz);
  mat->set_elem(1,1, 1.0f - (xx + zz));
  mat->set_elem(1,2, yz - wx);

  mat->set_elem(2,0, xz - wy);
  mat->set_elem(2,1, yz + wx);
  mat->set_elem(2,2, 1.0f - (xx + yy));
#else
   	asm __volatile__
	("
	    lqc2    vf4,0x0(%1)   #quat
	    vadd.xyzw vf5, vf4, vf4	    # vf5 x2,y2,z2,w2
	    vaddw.xyz vf10,vf0,vf0	    # vf10  1 1 1 2
	    #nop
	    #nop

	    vmulw.xyzw vf9, vf4, vf5	    # vf9 wx,wy,wz,ww
	    vmulx.xyzw vf6, vf4, vf5	    # vf6 xx,xy,xz,xw
	    vmuly.xyzw vf7, vf4, vf5	    # vf7 yx,yy,yz,yw
	    vmulz.xyzw vf8, vf4, vf5	    # vf8 zx,zy,zz,zw

	    vsubz.y  vf12, vf6, vf9	    # xy - wz
	    vaddz.x  vf13, vf7, vf9	    # xy + wz
	    vsuby.x  vf14, vf8, vf9	    # xz - wy

	    vsuby.x  vf12, vf10, vf7	    # 1.0 - yy
	    vsubx.y  vf13, vf10, vf6	    # 1.0 - xx
	    vsubx.z  vf14, vf10, vf6	    # 1.0 - xx

	    vaddy.z  vf12, vf6, vf9	    # xz + wy
	    vsubx.z  vf13, vf7, vf9	    # yz - wx
	    vaddx.y  vf14, vf8, vf9	    # yz + wx

	    vsubz.x  vf12, vf12, vf8	    # 1.0 - yy - zz
	    vsubz.y  vf13, vf13, vf8	    # 1.0 - xx - zz
	    vsuby.z  vf14, vf14, vf7	    # 1.0 - xx - yy	    

		
	    sqc2    vf12,0x0(%0)
	    sqc2    vf13,0x10(%0)
	    sqc2    vf14,0x20(%0)
	"
	: /*no output */
	: "r" (mat) , "r" (this)
	: "memory" );
#endif
}

//  Comments: remember matrix (in OGL) is represented in COLUMN major form
void IVP_U_Quat::set_quaternion(const IVP_U_Matrix3 *mat) {

    IVP_U_Quat *quat=this;
    const IVP_U_Matrix3 &m = *mat;

    IVP_DOUBLE tr = m.get_elem(0,0) + m.get_elem(1,1) + m.get_elem(2,2);

    // check the diagonal

    if (tr > 0.0f) 
    {
	IVP_DOUBLE s = IVP_Inline_Math::sqrtd (tr + 1.0f);

	quat->w = 0.5f * s;
    
	s = 0.5f / s;

	quat->x = (m.get_elem(2,1) - m.get_elem(1,2))*s;
	quat->y = (m.get_elem(0,2) - m.get_elem(2,0))*s;
	quat->z = (m.get_elem(1,0) - m.get_elem(0,1))*s;
    } else {		
	  
	// diagonal is negative
      IVP_DOUBLE  q[4];
      int    i, j, k;
      int nxt[3] = {1, 2, 0};
    
	i = 0;
	  
	if (m.get_elem(1,1) > m.get_elem(0,0)) i = 1;
	if (m.get_elem(2,2) > m.get_elem(i,i)) i = 2;
    
	j = nxt[i];
	k = nxt[j];

	IVP_DOUBLE s = IVP_Inline_Math::sqrtd ( m.get_elem(i,i) - (m.get_elem(j,j) + m.get_elem(k,k)) + 1.0f);
      
	q[i] = s * 0.5f;

	if (s != 0.0f) s = 0.5f / s; //knappe Abfrage?

	q[3] = (m.get_elem(k,j) - m.get_elem(j,k)) * s;
	q[j] = (m.get_elem(j,i) + m.get_elem(i,j)) * s;
	q[k] = (m.get_elem(k,i) + m.get_elem(i,k)) * s;

	quat->x = q[0];
	quat->y = q[1];
	quat->z = q[2];
	quat->w = q[3];
    }
    quat->normize_quat();
}

//  Comments: remember matrix (in OGL) is represented in COLUMN major form
void IVP_U_Quat::set_quaternion(const IVP_DOUBLE m[4][4]) {
  IVP_U_Quat *quat=this;
  IVP_DOUBLE  tr, s;

  tr = m[0][0] + m[1][1] + m[2][2];

  // check the diagonal

  if (tr > 0.0f) 
  {
    s = IVP_Inline_Math::sqrtd (tr + 1.0f);

    quat->w = s *.5f;
    
	s = 0.5f / s;

	quat->x = (m[1][2] - m[2][1]) * s;
	quat->y = (m[2][0] - m[0][2]) * s;
	quat->z = (m[0][1] - m[1][0]) * s;

  } else {		
    IVP_DOUBLE  q[4];
    int    i, j, k;

    int nxt[3] = {1, 2, 0};
	  
	  // diagonal is negative
    
	  i = 0;
	  
      if (m[1][1] > m[0][0]) i = 1;
	  if (m[2][2] > m[i][i]) i = 2;
    
	  j = nxt[i];
      k = nxt[j];

      s = IVP_Inline_Math::sqrtd ((m[i][i] - (m[j][j] + m[k][k])) + 1.0f);
      
	  q[i] = s * 0.5f;

      if (s != 0.0f) s = 0.5f / s;

	  q[3] = (m[j][k] - m[k][j]) * s;
      q[j] = (m[i][j] + m[j][i]) * s;
      q[k] = (m[i][k] + m[k][i]) * s;

	  quat->x = q[0];
	  quat->y = q[1];
	  quat->z = q[2];
	  quat->w = q[3];
  }
}

// **************************************************************************
//  Action:	Smoothly (spherically, shortest path on a quaternion sphere) 
//			interpolates between two UNIT quaternion positions
//
//		slerp(p,q,t) = (p*sin((1-t)*omega) + q*sin(t*omega)) / sin(omega)
//
//***********************************************************************EDOC*/

void IVP_U_Quat::set_interpolate_smoothly(const IVP_U_Quat * from,const IVP_U_Quat * to, IVP_DOUBLE t){
	IVP_U_Quat *res=this;
	
        // calc cosine
        IVP_DOUBLE cosom = from->x * to->x + from->y * to->y + from->z * to->z
	                   + from->w * to->w;

        // adjust signs (if necessary)
	IVP_FLOAT sign;
	if ( cosom > 0.0f ){
	  sign = 1.0f;
	}else{
	  cosom = -cosom;
	  sign = -1.0f;
        }

        // calculate coefficients

	// #+# get rid of sin and cos
        if ( cosom < 1.0f - 0.001f /*IVP_QUAT_DELTA*/ ){ // 0.033 * 180/PI  degrees 
	  IVP_DOUBLE          scale0, scale1;
	  // standard case (slerp)
	  IVP_DOUBLE omega = IVP_Inline_Math::acosd(cosom);
	  IVP_DOUBLE i_sinom = 1.0f / IVP_Inline_Math::sqrtd(1.0f - cosom * cosom);
	  //IVP_DOUBLE i_sinom = 1.0f / IVP_Inline_Math::sind(omega);
	  scale0 = IVP_Inline_Math::sind((1.0f - t) * omega) * i_sinom;
	  scale1 = IVP_Inline_Math::sind(t * omega) * i_sinom * sign;
	  // calculate final values
	  res->x = scale0 * from->x + scale1 * to->x;
	  res->y = scale0 * from->y + scale1 * to->y;
	  res->z = scale0 * from->z + scale1 * to->z;
	  res->w = scale0 * from->w + scale1 * to->w;
#if 0
	IVP_DOUBLE len = res->acos_quat(res);
	printf("angle quat_error: %G %G %G	\n", t,cosom,  1.0f - len);
#endif		
        } else {        
			    // "from" and "to" quaternions are very close 
			    //  ... so we can do a linear interpolation
	  // calculate final values
	  res->x = from->x + t * (sign * to->x - from->x);
	  res->y = from->y + t * (sign * to->y - from->y);
	  res->z = from->z + t * (sign * to->z - from->z);
	  res->w = from->w + t * (sign * to->w - from->w);
	  res->normize_correct_step(3);
        }
}

/*SDOC***********************************************************************

  Name:		set_interpolate_linear

  Action:   Linearly interpolates between two quaternion positions

  Comments: fast but not as nearly as smooth as set_interpolate_smoothly

  ***/

void IVP_U_Quat::set_interpolate_linear(const IVP_U_Quat * from,const  IVP_U_Quat * to, IVP_DOUBLE t) {
        IVP_DOUBLE           to1[4];
        IVP_DOUBLE          cosom;
        IVP_DOUBLE          scale0, scale1;

        IVP_U_Quat *res=this;

        // calc cosine
        cosom = from->x * to->x + from->y * to->y + from->z * to->z
			       + from->w * to->w;

        // adjust signs (if necessary)
        if ( cosom < 0.0f )
		{
			to1[0] = - to->x;
			to1[1] = - to->y;
			to1[2] = - to->z;
			to1[3] = - to->w;

        } else  {

			to1[0] = to->x;
			to1[1] = to->y;
			to1[2] = to->z;
			to1[3] = to->w;
        }

 
		// interpolate linearly
        scale0 = 1.0f - t;
        scale1 = t;
 
		// calculate final values
		res->x = scale0 * from->x + scale1 * to1[0];
		res->y = scale0 * from->y + scale1 * to1[1];
		res->z = scale0 * from->z + scale1 * to1[2];
		res->w = scale0 * from->w + scale1 * to1[3];
}

/*SDOC***********************************************************************

  Name:		gluQuatNormalize_EXT

  Action:   Normalizes quaternion (i.e. w^2 + x^2 + y^2 + z^2 = 1)

  Params:   GL_QUAT* (quaternion)

***********************************************************************EDOC*/
void IVP_U_Quat::normize_quat() {
    IVP_DOUBLE	dist, square;
    IVP_U_Quat *quat=this;
	square = quat->x * quat->x + quat->y * quat->y + quat->z * quat->z
		+ quat->w * quat->w;
	
	if (square > P_DOUBLE_EPS){
	  dist = (IVP_DOUBLE)(1.0f / IVP_Inline_Math::sqrtd(square));
	  quat->x *= dist;
	  quat->y *= dist;
	  quat->z *= dist;
	  quat->w *= dist;
	}

}


void IVP_U_Quat::fast_normize_quat() {
    IVP_DOUBLE	square;
    IVP_U_Quat *quat=this;
    square = quat->x * quat->x + quat->y * quat->y + quat->z * quat->z + quat->w * quat->w;
    if ( IVP_Inline_Math::fabsd ( 1.0f - (square) ) > P_DOUBLE_RES ){
	IVP_DOUBLE factor = 1.5f - 0.5f * square;
	goto loop;
	while ( IVP_Inline_Math::fabsd ( 1.0f - (factor * factor * square) ) > P_DOUBLE_RES ){
	loop:
	    factor += 0.5f * (1.0f - ( factor * factor * square ));
	}
	quat->x *= factor;
	quat->y *= factor;
	quat->z *= factor;
	quat->w *= factor;
    }

}


/*SDOC***********************************************************************

  Name:		gluQuatInverse_EXT

  Action:   Inverts quaternion's rotation ( q^(-1) )

  Params:   GL_QUAT* (quaternion)

  Returns:  nothing

  Comments: none
Returns the inverse of the quaternion (1/q).  check conjugate
***********************************************************************EDOC*/
void IVP_U_Quat::invert_quat() {
        IVP_U_Quat *quat=this;
	IVP_DOUBLE norm, invNorm;

	norm = quat->x * quat->x + quat->y * quat->y + quat->z * quat->z
		               + quat->w * quat->w;
	
	invNorm = (IVP_DOUBLE) (1.0f / norm);
	
	quat->x = -quat->x * invNorm;
	quat->y = -quat->y * invNorm;
	quat->z = -quat->z * invNorm;
	quat->w =  quat->w * invNorm;
}

void IVP_U_Quat::set_invert_unit_quat(const IVP_U_Quat *q1) {
        IVP_U_Quat *quat=this;
	
	quat->x = -q1->x;
	quat->y = -q1->y;
	quat->z = -q1->z;
	quat->w =  q1->w;
}

/************************************************************************

  Name:		set_from_rotation_vectors

  Action:   Constructs quaternion to rotate from one direction vector to 
			another

  Params:   GLIVP_FLOAT (x1, y1, z1 - from vector), 
			GLIVP_FLOAT (x2, y2, z2 - to vector), GL_QUAT* (resulting quaternion)

  Returns:  nothing

  Comments: Two vectors have to be UNIT vectors (so make sure you normalize
			them before calling this function
		       
************************************************************************/
void IVP_U_Quat::set_from_rotation_vectors(IVP_DOUBLE x1,IVP_DOUBLE y1, IVP_DOUBLE z1, IVP_DOUBLE x2, IVP_DOUBLE y2, IVP_DOUBLE z2)
{
    IVP_U_Quat *quat=this;
    IVP_DOUBLE tx, ty, tz, temp, dist;

    IVP_DOUBLE	cost, len, ss;

	// get dot product of two vectors
	IVP_DOUBLE s1,s2,s3;
	s2=y2 * y1;
	s3=z1 * z2;
	s1=x1 * x2;
    cost =  s1 + s2 + s3;

    // check if parallel
    if (cost > 0.99999f) {
	quat->x = quat->y = quat->z = 0.0f;
	quat->w = 1.0f;
	return;
    }
    else if (cost < -0.99999f) {		// check if opposite

	// check if we can use cross product of from vector with [1, 0, 0]
	tx = 0.0f;
	ty = x1;
	tz = -y1;

	len = IVP_Inline_Math::sqrtd(ty*ty + tz*tz);

	if (len < IVP_QUAT_DELTA)
	{
		// nope! we need cross product of from vector with [0, 1, 0]
		tx = -z1;
		ty = 0.0f;
		tz = x1;
	}

	// normalize
	temp = tx*tx + ty*ty + tz*tz;

    dist = (IVP_DOUBLE)(1.0f / IVP_Inline_Math::sqrtd(temp));

    tx *= dist;
    ty *= dist;
    tz *= dist;
	
	quat->x = tx;
	quat->y = ty;
	quat->z = tz;
	quat->w = 0.0f;

	return;
    }

	// ... else we can just cross two vectors

	tx = y1 * z2 - z1 * y2;
	ty = z1 * x2 - x1 * z2;
	tz = x1 * y2 - y1 * x2;

	temp = tx*tx + ty*ty + tz*tz;

    dist = (IVP_DOUBLE)(1.0f / IVP_Inline_Math::sqrtd(temp));

    tx *= dist;
    ty *= dist;
    tz *= dist;


    // we have to use half-angle formulae (sin^2 t = ( 1 - cos (2t) ) /2)
	
	ss = (IVP_DOUBLE)IVP_Inline_Math::sqrtd(0.5f * (1.0f - cost));

	tx *= ss;
	ty *= ss;
    tz *= ss;

    // scale the axis to get the normalized quaternion
    quat->x = tx;
    quat->y = ty;
    quat->z = tz;

    // cos^2 t = ( 1 + cos (2t) ) / 2
    // w part is cosine of half the rotation angle
    quat->w = IVP_Inline_Math::sqrtd(0.5f * (1.0f + cost));

}

/*SDOC***********************************************************************

  Name:		gluQuatMul_EXT

  Action:   Multiplies two quaternions

  Params:   GL_QUAT ( q1 * q2 = res)

  Returns:  nothing

  Comments: NOTE: multiplication is not commutative

***********************************************************************EDOC*/
void IVP_U_Quat::set_mult_quat(const IVP_U_Quat* q1,const  IVP_U_Quat* q2) {
  this->inline_set_mult_quat(q1,q2);
}



/*SDOC***********************************************************************

  Name:		gluQuatDiv_EXT

  Action:   Divide two quaternions

  Params:   GL_QUAT* (q1 / q2 = res)

  Returns:  nothing

  Comments: none

***********************************************************************EDOC*/
void IVP_U_Quat::set_div_unit_quat(const IVP_U_Quat* q1,const  IVP_U_Quat* q2) {
    IVP_U_Quat *res = this;
    IVP_U_Quat q;
    q.set_invert_unit_quat(q2);
    res->inline_set_mult_quat( q1,&q);
}

void IVP_U_Quat::set_invert_mult(const IVP_U_Quat* q1,const  IVP_U_Quat* q2) {
    IVP_U_Quat *res = this;
    IVP_U_Quat q;
    q.set_invert_unit_quat(q1);
    res->inline_set_mult_quat( &q,q2);
}

#if 0


/*SDOC***********************************************************************

  Name:		gluEulerToQuat_EXT

  Action:   Converts representation of a rotation from Euler angles to
			quaternion representation

  Params:   GLIVP_FLOAT (roll), GLIVP_FLOAT (pitch), GLIVP_FLOAT (yaw), GL_QUAT* (quat)

  Returns:  nothing

  Comments: remember:	roll  - rotation around X axis
						pitch - rotation around Y axis
						yaw   - rotation around Z axis
			
			rotations are performed in the following order:
					yaw -> pitch -> roll

			Qfinal = Qyaw Qpitch Qroll

***********************************************************************EDOC*/
void APIENTRY gluEulerToQuat_EXT(GLIVP_FLOAT roll, GLIVP_FLOAT pitch, GLIVP_FLOAT yaw, 
													GL_QUAT * quat)
{
	
	IVP_FLOAT cr, cp, cy, sr, sp, sy, cpcy, spsy;

	cr = cosf(roll/2);
	cp = cosf(pitch/2);
	cy = cosf(yaw/2);

	sr = sinf(roll/2);
	sp = sinf(pitch/2);
	sy = sinf(yaw/2);
	
	cpcy = cp * cy;
	spsy = sp * sy;

	quat->w = cr * cpcy + sr * spsy;
	quat->x = sr * cpcy - cr * spsy;
	quat->y = cr * sp * cy + sr * cp * sy;
	quat->z = cr * cp * sy - sr * sp * cy;

}


/*SDOC***********************************************************************

  Name:		gluQuatGetValue_EXT

  Action:   Disassembles quaternion to an axis and an angle

  Params:   GL_QUAT* (quaternion), GLIVP_FLOAT* (x, y, z - axis), GLIVP_FLOAT (angle)

  Returns:  nothing

  Comments: NOTE: vector has been split into x, y, z so that you do not have
			to change your vector library (i.e. greater portability)

			NOTE2: angle is in RADIANS

***********************************************************************EDOC*/
void APIENTRY gluQuatGetValue_EXT(GL_QUAT *quat, GLIVP_FLOAT *x, GLIVP_FLOAT *y, 
											GLIVP_FLOAT *z, GLIVP_FLOAT *radians)
{
    GLIVP_FLOAT	len;
    GLIVP_FLOAT tx, ty, tz;

	// cache variables
	tx = quat->x;
	ty = quat->y;
	tz = quat->z;

	len = tx * tx + ty * ty + tz * tz;

    if (len > IVP_QUAT_DELTA) 
	{
		*x = tx * (1.0f / len);
		*y = ty * (1.0f / len);
		*z = tz * (1.0f / len);
	    *radians = (GLIVP_FLOAT)(2.0f * acosf(quat->w));
    }

    else {
		*x = 0.0f;
		*y = 0.0f;
		*z = 1.0f;
	    *radians = 0.0f;
    }
}


/*SDOC***********************************************************************

  Name:		gluQuatSetValue_EXT

  Action:   Assembles quaternion from an axis and an angle

  Params:   GL_QUAT* (quaternion), GLIVP_FLOAT (x, y, z - axis), GLIVP_FLOAT (angle)

  Returns:  nothing

  Comments: NOTE: vector has been split into x, y, z so that you do not have
			to change your vector library (i.e. greater portability)

			NOTE2: angle has to be in RADIANS

***********************************************************************EDOC*/
void APIENTRY gluQuatSetValue_EXT(GL_QUAT *quat, GLIVP_FLOAT x, GLIVP_FLOAT y, 
												GLIVP_FLOAT z, GLIVP_FLOAT angle)
{
	GLIVP_FLOAT temp, dist;

	// normalize
	temp = x*x + y*y + z*z;

    dist = (GLIVP_FLOAT)(1.0f / IVP_Inline_Math::sqrtd(temp));

    x *= dist;
    y *= dist;
    z *= dist;

	quat->x = x;
	quat->y = y;
	quat->z = z;

	quat->w = (GLIVP_FLOAT)cos(angle / 2.0f);
	
}



/*SDOC***********************************************************************

  Name:		gluQuatScaleAngle_EXT

  Action:   Scales the rotation angle of a quaternion

  Params:   GL_QUAT* (quaternion), GLIVP_FLOAT (scale value)

  Returns:  nothing

  Comments: none

***********************************************************************EDOC*/
void APIENTRY gluQuatScaleAngle_EXT(GL_QUAT * quat, GLIVP_FLOAT scale)
{
    GLIVP_FLOAT x, y, z;	// axis
    GLIVP_FLOAT angle;		// and angle

	gluQuatGetValue_EXT(quat, &x, &y, &z, &angle);

    gluQuatSetValue_EXT(quat, x, y, z, (angle * scale));
}


/*SDOC***********************************************************************

  Name:		gluQuatCopy_EXT

  Action:   copies q1 into q2

  Params:   GL_QUAT* (q1 and q2)

  Returns:  nothing

  Comments: none

***********************************************************************EDOC*/
void APIENTRY gluQuatCopy_EXT(GL_QUAT* q1, GL_QUAT* q2)
{
	q2->x = q1->x;
	q2->y = q1->y;
	q2->z = q1->z;
	q2->w = q1->w;
}



/*SDOC***********************************************************************

  Name:		gluQuatSquare_EXT

  Action:   Square quaternion

  Params:   GL_QUAT* (q1 * q1 = res)

  Returns:  nothing

  Comments: none

***********************************************************************EDOC*/
void APIENTRY gluQuatSquare_EXT(GL_QUAT* q1, GL_QUAT* res)
{
	GLIVP_FLOAT  tt;


	tt = 2 * q1->w;
	res->x = tt * q1->x;
	res->y = tt * q1->y;
	res->z = tt * q1->z;
	res->w = (q1->w * q1->w - q1->x * q1->x - q1->y * q1->y - q1->z * q1->z);
}


/*SDOC***********************************************************************

  Name:		gluQuatSqrt_EXT

  Action:   Find square root of a quaternion

  Params:   GL_QUAT* (sqrt(q1) = res)

  Returns:  nothing

  Comments: none

***********************************************************************EDOC*/
void APIENTRY gluQuatSqrt_EXT(GL_QUAT* q1, GL_QUAT* res)
{
	GLIVP_FLOAT  length, m, r1, r2;
	GL_QUAT r;

	length = IVP_Inline_Math::sqrtd (q1->w * q1->w + q1->x * q1->x + q1->y * q1->y);
	if (length != 0.0f) 
		length = 1.0f / length; 
	else length = 1.0f;

	r.x = q1->x * length;
	r.y = q1->z * length;
	r.z = 0.0f;
	r.w = q1->w * length;

	m = 1.0f / IVP_Inline_Math::sqrtd (r.w * r.w + r.x * r.x);
	r1 = IVP_Inline_Math::sqrtd ((1.0f + r.y) * 0.5f);
	r2 = IVP_Inline_Math::sqrtd ((1.0f - r.y) * 0.5f);

	res->x = IVP_Inline_Math::sqrtd (length) * r2 * r.x * m;
	res->y = IVP_Inline_Math::sqrtd (length) * r1;
	res->z = q1->z;
	res->w = IVP_Inline_Math::sqrtd (length) * r1 * r.w * m;

}


/*SDOC***********************************************************************

  Name:		gluQuatDot_EXT

  Action:   Computes the dot product of two unit quaternions

  Params:   GL_QUAT (first and second quaternion)

  Returns:  (GLIVP_FLOAT) Dot product

  Comments: Quaternion has to be normalized (i.e. it's a unit quaternion)

***********************************************************************EDOC*/
GLIVP_FLOAT APIENTRY gluQuatDot_EXT(GL_QUAT* q1, GL_QUAT* q2)
{
  return (GLIVP_FLOAT)(q1->w * q2->w + q1->x * q2->x + q1->y * q2->y+q1->z*q2->z);
}


/*SDOC***********************************************************************

  Name:		gluQuatLength_EXT

  Action:   Calculates the length of a quaternion

  Params:   GL_QUAT* (quaternion)

  Returns:  GLIVP_FLOAT (length)

  Comments: none

***********************************************************************EDOC*/
GLIVP_FLOAT APIENTRY gluQuatLength_EXT(GL_QUAT* q1)
{
  return IVP_Inline_Math::sqrtd (q1->w * q1->w + q1->x * q1->x + q1->y * q1->y + q1->z * q1->z);
}


/*SDOC***********************************************************************

  Name:		gluQuatNegate_EXT

  Action:   Negates vector part of a quaternion

  Params:   GL_QUAT (source and destination quaternion)

  Returns:  nothing

  Comments: Source quaternion does NOT have to be normalized 

***********************************************************************EDOC*/
void APIENTRY gluQuatNegate_EXT(GL_QUAT* q1, GL_QUAT* q2)
{
	gluQuatCopy_EXT(q1, q2);

	gluQuatNormalize_EXT(q2);
	q2->x = -q2->x;
	q2->y = -q2->y;
	q2->z = -q2->z;
}

/*SDOC***********************************************************************

  Name:		gluQuatExp_EXT

  Action:   Calculates exponent of a quaternion

  Params:   GL_QUAT* (Source and destination quaternion)

  Returns:  nothing

  Comments: none

***********************************************************************EDOC*/
void APIENTRY gluQuatExp_EXT(GL_QUAT* q1, GL_QUAT* q2)
{
	GLIVP_FLOAT  len1, len2;

	len1 = (GLIVP_FLOAT) IVP_Inline_Math::sqrtd (q1->x * q1->x + q1->y * q1->y + q1->z * q1->z);
	if (len1 > 0.0f) 
		len2 = (GLIVP_FLOAT)sin(len1) / len1; 
	else 
		len2 = 1.0f;

	q2->x = q1->x * len2;
	q2->y = q1->y * len2;
	q2->z = q1->z * len2;
	q2->w = cos (len1);
}


/*SDOC***********************************************************************

  Name:		gluQuatLog_EXT

  Action:   Calculates natural logarithm of a quaternion

  Params:   GL_QUAT* (Source and destination quaternion)

  Returns:  nothing

  Comments: none

***********************************************************************EDOC*/
void APIENTRY gluQuatLog_EXT(GL_QUAT* q1, GL_QUAT* q2)
{
	GLIVP_FLOAT  length;

	length = sqrt (q1->x * q1->x + q1->y * q1->y + q1->z * q1->z);

	//make sure we do not divide by 0
	if (q1->w != 0.0f) 
		length = atan (length / q1->w); 
	else length = (GLIVP_FLOAT)IVP_PI/2;

	q2->w = 0.0f;
	q2->x = q1->x * length;
	q2->y = q1->y * length;
	q2->z = q1->z * length;
}

/*SDOC***********************************************************************

  Name:		gluQuatLnDif_EXT

  Action:   Computes the "natural log difference" of two quaternions,
			q1 and q2 as  ln(qinv(q1)*q2)

  Params:   GL_QUAT* (Source quaternions  and a destination quaternion)

  Returns:  nothing

  Comments: none

***********************************************************************EDOC*/
void APIENTRY gluQuatLnDif_EXT(GL_QUAT *q1, GL_QUAT *q2, GL_QUAT *res)
{

	GL_QUAT inv, dif, temp;
	GLIVP_FLOAT  len, len1, s;

	qt_inverse (a, &inv);
	qt_mul (&inv, b, &dif);
	len = sqrt (dif.x*dif.x + dif.y*dif.y + dif.z*dif.z);
	s = qt_dot (a, b);
	if (s != 0.0f) len1 = atan (len / s); else len1 = IVP_PI/2;
	if (len != 0.0f) len1 /= len;
	temp.w = 0.0f;
	temp.x = dif.x * len1;
	temp.y = dif.y * len1;
	temp.z = dif.z * len1;
	qt_copy (&temp, out);
}



// cleanup stuff we changed
#if defined (WIN32)
#pragma warning( default : 4244 )	// set it to default again
#endif
#endif






