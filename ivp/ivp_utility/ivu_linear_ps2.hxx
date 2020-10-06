// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PROTECTED


#ifndef WIN32
#	pragma interface
#endif



#if 1


inline void IVP_U_Float_Point::set_negative(const IVP_U_Float_Point *p_source){
    IVP_DOUBLE a = p_source->k[0];
    IVP_DOUBLE b = p_source->k[1];	a = -a;
    IVP_DOUBLE c = p_source->k[2];	b = -b;
                                c = -c;
    k[0]= (IVP_FLOAT)a;
    k[1]= (IVP_FLOAT)b;
    k[2]= (IVP_FLOAT)c;
}


inline void IVP_U_Float_Point::mult(IVP_DOUBLE factor){
    IVP_DOUBLE a,b,c;
    a = k[0] * factor;
    b = k[1] * factor;
    c = k[2] * factor;
    k[0] = (IVP_FLOAT)a; k[1] = (IVP_FLOAT)b; k[2] = (IVP_FLOAT)c;
}



inline void IVP_U_Float_Point::set_pairwise_mult (const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2){	// pairwise multiple 
    IVP_DOUBLE a,b,c;
    a = v1->k[0] * v2->k[0];
    b = v1->k[1] * v2->k[1];
    c = v1->k[2] * v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}


inline void IVP_U_Float_Point::add(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2){
    IVP_DOUBLE a = v1->k[0] + v2->k[0];
    IVP_DOUBLE b = v1->k[1] + v2->k[1];
    IVP_DOUBLE c = v1->k[2] + v2->k[2];
    k[0] = (IVP_FLOAT)a;k[1] = (IVP_FLOAT)b;k[2] = (IVP_FLOAT)c;  
}


inline void IVP_U_Float_Point::subtract(const IVP_U_Float_Point *v1,const IVP_U_Float_Point *v2){
    IVP_DOUBLE a,b,c;
    a = v1->k[0] - v2->k[0];
    b = v1->k[1] - v2->k[1];
    c = v1->k[2] - v2->k[2];

    k[0] = (IVP_FLOAT)a;
    k[1] = (IVP_FLOAT)b;
    k[2] = (IVP_FLOAT)c;
}



inline IVP_DOUBLE IVP_U_Float_Point::quad_length() const {
    return (k[0]*k[0] + k[1]*k[1] + k[2]*k[2]);
}


#else



inline void IVP_U_Float_Point::set_negative(const IVP_U_Float_Point *p_source){
    IVP_DOUBLE a = p_source->k[0];
    IVP_DOUBLE b = p_source->k[1];	a = -a;
    IVP_DOUBLE c = p_source->k[2];	b = -b;
                                c = -c;
    k[0]= (IVP_FLOAT)a;
    k[1]= (IVP_FLOAT)b;
    k[2]= (IVP_FLOAT)c;
}


inline void IVP_U_Float_Point::mult(IVP_DOUBLE factor){
 	asm __volatile__("
	lqc2    vf4,0x0(%0)
        mfc1    $8,%1
        qmtc2    $8,vf5
	vmulx.xyzw	vf6,vf4,vf5
	sqc2    vf6,0x0(%0)
	": : "r" (this) , "f" (factor):"$8");
}


inline void IVP_U_Float_Point::set_pairwise_mult (const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2){	// pairwise multiple 
	asm __volatile__("
	lqc2    vf4,0x0(%1)
	lqc2    vf5,0x0(%2)
	vmul.xyz	vf6,vf4,vf5
	sqc2    vf6,0x0(%0)
	": : "r" (this) , "r" (v1), "r" (v2));
}


inline void IVP_U_Float_Point::add(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2){
	asm __volatile__("
	lqc2     vf4,0x0(%1)
	lqc2     vf5,0x0(%2)
	vadd.xyz vf6,vf4,vf5
	sqc2     vf6,0x0(%0)
	": : "r" (this) , "r" (v1), "r" (v2));
}



inline void IVP_U_Float_Point::subtract(const IVP_U_Float_Point *v1,const IVP_U_Float_Point *v2){
	asm __volatile__("
	lqc2    vf4,0x0(%1)
	lqc2    vf5,0x0(%2)
	vsub.xyz	vf6,vf4,vf5
	sqc2    vf6,0x0(%0)
	"
	: /*no output*/
	: "r" (this) , "r" (v1), "r" (v2)
	: "memory");
}


inline IVP_DOUBLE IVP_U_Float_Point::quad_length() const {
    return (k[0]*k[0] + k[1]*k[1] + k[2]*k[2]);
}

}
#endif



inline IVP_DOUBLE IVP_U_Float_Point::quad_distance_to(const IVP_U_Float_Point *p)const{
#if 1
    IVP_DOUBLE a = k[0] - p->k[0];
    IVP_DOUBLE b = k[1] - p->k[1];
    IVP_DOUBLE c = k[2] - p->k[2];
    a *=a; b*= b; c*=c;
    return a+b+c;
#else
    register float ret;
	asm __volatile__("
	lqc2    vf4,0x0(%1)
	lqc2    vf5,0x0(%2)
	vsub.xyz vf5,vf4,vf4
	vmul.xyz vf5,vf5,vf5
	vaddy.x vf5,vf5,vf5
	vaddz.x vf5,vf5,vf5
	qmfc2   $2 ,vf5
	mtc1    $2,%0
	"
	: "=r" (ret) 
	:"r" (this) ,"r" (p) 
	:"$2", "memory" );
	return ret;
#endif
}


inline void IVP_U_Float_Point::set_to_zero() {
	asm __volatile__("
	sqc2    vf0,0x0(%0)
	"
	:/*no output*/
	: "r" (this)
	: "memory" );
}

inline void IVP_U_Float_Point::set(const IVP_U_Float_Point *p_source){
#if 0
    IVP_DOUBLE a = p_source->k[0];
    IVP_DOUBLE b = p_source->k[1];
    IVP_DOUBLE c = p_source->k[2];
    k[0]= (IVP_FLOAT)a;
    k[1]= (IVP_FLOAT)b;
    k[2]= (IVP_FLOAT)c;
#else
	asm __volatile__("
	lqc2    vf4,0x0(%1)
	sqc2    vf4,0x0(%0)
	"
	: /* no output */
	: "r" (this) , "r" (p_source)
	: "memory");
#endif
}


inline void IVP_U_Float_Point::add_multiple (const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2, IVP_DOUBLE factor2){	// vektor addition
#if 0
    IVP_DOUBLE a,b,c;

    a = v2->k[0] * factor2;
    b = v2->k[1] * factor2;    a += v1->k[0];
    c = v2->k[2] * factor2;    b += v1->k[1];
                               c += v1->k[2];
    k[0] = a;
    k[1] = b;
    k[2] = c;
#else
    asm __volatile__("
	lqc2    vf4,0x0(%2)
        mfc1    $8,%3
        qmtc2    $8,vf5
	vmulx.xyz vf6,vf4,vf5

	lqc2    vf4,0x0(%1)
	vadd.xyz vf6,vf4,vf6
	sqc2    vf6,0x0(%0)
	"
	: /* no output */
	: "r" (this), "r" (v1) , "r" (v2), "f" (factor2)
	: "$8" , "memory");
#endif
}


inline IVP_DOUBLE IVP_U_Float_Point::dot_product(const IVP_U_Float_Point *v2) const {
#if 1
        return( k[0]*v2->k[0] + k[1]*v2->k[1] + k[2]*v2->k[2] );
#else
	register float ret;
	asm __volatile__("
	lqc2    vf4,0x0(%1)
	lqc2    vf5,0x0(%2)
	vmul.xyz vf5,vf4,vf5
	vaddy.x vf5,vf5,vf5
	vaddz.x vf5,vf5,vf5
	qmfc2   $2 ,vf5
	mtc1    $2,%0
	": "=r" (ret) :"r" (this) ,"r" (v2) :"$2" );
	return ret;
#endif
};

inline void IVP_U_Float_Point::set_multiple(const IVP_U_Float_Point *v, IVP_DOUBLE f){
#if 0
    IVP_DOUBLE a = v->k[0] * f;
    IVP_DOUBLE b = v->k[1] * f;
    IVP_DOUBLE c = v->k[2] * f;
    k[0]= (IVP_FLOAT)a;
    k[1]= (IVP_FLOAT)b;
    k[2]= (IVP_FLOAT)c;
#else
 	asm __volatile__("
	lqc2    vf4,0x0(%1)
        mfc1    $8,%2
        qmtc2    $8,vf5
	vmulx.xyz	vf6,vf4,vf5
	sqc2    vf6,0x0(%0)
	"
	: /* no output */
	: "r" (this) , "r" (v), "f" (f):
	"$8", "memory");
#endif
}


inline void IVP_U_Float_Point::inline_subtract_and_mult(const IVP_U_Float_Point *v1,const IVP_U_Float_Point *v2, IVP_DOUBLE factor){	// vektor addition
#if 0
    IVP_DOUBLE a,b,c;
    a = (v1->k[0] - v2->k[0]);
    b = (v1->k[1] - v2->k[1]);	a *= factor;
    c = (v1->k[2] - v2->k[2]);	b *= factor;
    c *=factor;
    k[0] = a;
    k[1] = b;
    k[2] = c;
#else
    asm __volatile__("
	lqc2    vf4,0x0(%1)
	lqc2    vf6,0x0(%2)
	vsub.xyz vf4, vf4,vf6
        mfc1    $8,%3
        qmtc2    $8,vf5
	vmulx.xyz vf6,vf4,vf5
	sqc2    vf6,0x0(%0)
	"
	: /* no output */
	: "r" (this), "r" (v1) , "r" (v2), "f" (factor)
	:"$8", "memory");
#endif
}


//////////////////////////////////////////////////////////////////////

inline IVP_DOUBLE IVP_U_Hesse::get_dist(const IVP_U_Float_Point *p) const {
    return this->dot_product(p) + hesse_val;
}

inline IVP_DOUBLE IVP_U_Float_Hesse::get_dist(const IVP_U_Float_Point *p) const {
    return this->dot_product(p) + hesse_val;
}


void IVP_U_Matrix3::inline_vimult3( const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out ) const{
#if 0
    IVP_DOUBLE a = get_elem(0,0)*p_in->k[0] + get_elem(1,0)*p_in->k[1] + get_elem(2,0)*p_in->k[2];
    IVP_DOUBLE b = get_elem(0,1)*p_in->k[0] + get_elem(1,1)*p_in->k[1] + get_elem(2,1)*p_in->k[2];
    IVP_DOUBLE c = get_elem(0,2)*p_in->k[0] + get_elem(1,2)*p_in->k[1] + get_elem(2,2)*p_in->k[2];
	p_out->k[0] = (IVP_FLOAT)a;p_out->k[1] = (IVP_FLOAT)b; p_out->k[2] = (IVP_FLOAT)c;
#else
	asm __volatile__("
	lqc2    vf8,0x0(%2)
	lqc2    vf4,0x0(%1)
	vmulax.xyz	ACC,   vf4,vf8
	lqc2    vf5,0x10(%1)
	vmadday.xyz	ACC,   vf5,vf8
	lqc2    vf6,0x20(%1)
	vmaddz.xyz	vf12,   vf6,vf8
	sqc2    vf12,0x0(%0)
	"
	: /* no output */
	: "r" (p_out) , "r" (this) ,"r" (p_in)
	: "memory" );
#endif
};

inline void IVP_U_Matrix::inline_vimult4( const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out )const{
#if 0
    IVP_DOUBLE a = p_in->k[0] - vv.k[0];
    IVP_DOUBLE b = p_in->k[1] - vv.k[1];
    IVP_DOUBLE c = p_in->k[2] - vv.k[2];
    
    p_out->k[0] = get_elem(0,0)*a + get_elem(1,0)*b + get_elem(2,0)*c;
    p_out->k[1] = get_elem(0,1)*a + get_elem(1,1)*b + get_elem(2,1)*c;
    p_out->k[2] = get_elem(0,2)*a + get_elem(1,2)*b + get_elem(2,2)*c;
#else
	asm __volatile__("
	lqc2    vf7,0x30(%1)
	lqc2    vf8,0x0(%2)
	vsub.xyz vf8, vf8, vf7
	lqc2    vf4,0x0(%1)
	lqc2    vf5,0x10(%1)
	vmulax.xyz	ACC,   vf4,vf8
	lqc2    vf6,0x20(%1)
	vmadday.xyz	ACC,   vf5,vf8
	vmaddz.xyz	vf12,   vf6,vf8
	sqc2    vf12,0x0(%0)
	"
	:/*no output*/
	: "r" (p_out) , "r" (this) ,"r" (p_in)
	:"memory");
#endif
};


void IVP_U_Matrix3::inline_vmult3( const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out ) const{
#if 0
	IVP_DOUBLE a = get_elem(0,0)*p_in->k[0] + get_elem(0,1)*p_in->k[1] + get_elem(0,2)*p_in->k[2];
	IVP_DOUBLE b = get_elem(1,0)*p_in->k[0] + get_elem(1,1)*p_in->k[1] + get_elem(1,2)*p_in->k[2];
	IVP_DOUBLE c = get_elem(2,0)*p_in->k[0] + get_elem(2,1)*p_in->k[1] + get_elem(2,2)*p_in->k[2];
	p_out->k[0] = (IVP_FLOAT)a;p_out->k[1] = (IVP_FLOAT)b; p_out->k[2] = (IVP_FLOAT)c;
#else
	asm __volatile__("
	lqc2    vf8,0x0(%2)
	lqc2    vf4,0x0(%1)
	lqc2    vf5,0x10(%1)

	lqc2    vf6,0x20(%1)

	vmul.xyz vf4,vf4,vf8
	vmul.xyz vf5,vf5,vf8
	vmul.xyz vf6,vf6,vf8

	vaddy.x vf4,vf4,vf4
	vaddx.y vf5,vf5,vf5
	vaddy.z vf6,vf6,vf6

	vaddz.x vf12,vf4,vf4
	vaddz.y vf12,vf5,vf5
	vaddx.z vf12,vf6,vf6

	sqc2    vf12,0x0(%0)
	"
	: /*no output */
	: "r" (p_out) , "r" (this) ,"r" (p_in)
	: "memory" );
#endif
};


inline void IVP_U_Matrix::inline_vmult4(const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out)const{
#if 0
       IVP_DOUBLE h = p_in->k[0];
    IVP_DOUBLE a = h * get_elem(0,0);
    IVP_DOUBLE b = h * get_elem(1,0);
    IVP_DOUBLE c = h * get_elem(2,0);
    IVP_DOUBLE x,y;
    h = p_in->k[1]; x = h*get_elem(0,1);	y = h*get_elem(1,1);	a += x;	x = h*get_elem(2,1);	 b+=y, c+= x;
    h = p_in->k[2]; x = h*get_elem(0,2);	y = h*get_elem(1,2);	a += x;	x = h*get_elem(2,2);	 b+=y, c+= x;
    a += vv.k[0];    b += vv.k[1];    c += vv.k[2];
    p_out->k[0] = (IVP_FLOAT)a;p_out->k[1] = (IVP_FLOAT)b; p_out->k[2] = (IVP_FLOAT)c;
#else
 	asm __volatile__("
	lqc2    vf8,0x0(%2)
	lqc2    vf4,0x0(%1)
	lqc2    vf5,0x10(%1)

	lqc2    vf6,0x20(%1)

	vmul.xyz vf4,vf4,vf8
	vmul.xyz vf5,vf5,vf8
	vmul.xyz vf6,vf6,vf8

	lqc2    vf7,0x30(%1)

	vaddy.x vf4,vf4,vf4
	vaddx.y vf5,vf5,vf5
	vaddy.z vf6,vf6,vf6

	vaddz.x vf12,vf4,vf4
	vaddz.y vf12,vf5,vf5
	vaddx.z vf12,vf6,vf6

	vadd.xyz vf12, vf12, vf7

	sqc2    vf12,0x0(%0)
	"
	: /*no output*/
	: "r" (p_out) , "r" (this) ,"r" (p_in)
	: "memory");
#endif
}


void IVP_U_Float_Point::inline_calc_cross_product(const IVP_U_Float_Point *v1,const IVP_U_Float_Point *v2)
{
#if 0
    IVP_DOUBLE a  = v1->k[1] * v2->k[2];
    IVP_DOUBLE a2 = v2->k[1] * v1->k[2];
    IVP_DOUBLE b  = v1->k[2] * v2->k[0];	
    IVP_DOUBLE b2 = v2->k[2] * v1->k[0];	a -= a2;
    IVP_DOUBLE c  = v1->k[0] * v2->k[1];
    IVP_DOUBLE c2 = v2->k[0] * v1->k[1];	b -= b2;
    k[0] = (IVP_FLOAT)a;k[1] = (IVP_FLOAT)b;	c -= c2;
    k[2] = (IVP_FLOAT)c;
#else
 	asm __volatile__
	("
		lqc2    vf4,0x0(%1)
		lqc2    vf5,0x0(%2)
		vopmula.xyz	ACC,vf4,vf5
		vopmsub.xyz	vf6,vf5,vf4
		sqc2    vf6,0x0(%0)
	"
	: /*no output */
	: "r" (this) , "r" (v1) ,"r" (v2)
	: "memory" );
#endif
}

void IVP_U_Float_Point::inline_calc_cross_product_and_normize(const IVP_U_Float_Point *v1,const IVP_U_Float_Point *v2)
{
#if 0
    IVP_DOUBLE a  = v1->k[1] * v2->k[2];
    IVP_DOUBLE a2 = v2->k[1] * v1->k[2];
    IVP_DOUBLE b  = v1->k[2] * v2->k[0];	
    IVP_DOUBLE b2 = v2->k[2] * v1->k[0];	a -= a2;
    IVP_DOUBLE c  = v1->k[0] * v2->k[1];
    IVP_DOUBLE c2 = v2->k[0] * v1->k[1];	b -= b2;
    k[0] = (IVP_FLOAT)a;k[1] = (IVP_FLOAT)b;	c -= c2;
    k[2] = (IVP_FLOAT)c;
    this->normize();
#else
 	asm __volatile__
	("
		lqc2    vf4,0x0(%1)
		lqc2    vf5,0x0(%2)
		vopmula.xyz	ACC,vf4,vf5
		vopmsub.xyz	vf6,vf5,vf4
		vmul.xyz vf5,vf6,vf6
		vaddy.x vf5,vf5,vf5
		vaddz.x vf5,vf5,vf5

		vrsqrt Q,vf0w,vf5x
		vwaitq
		vmulq.xyz  vf7,vf6,Q

		sqc2    vf7,0x0(%0)
	"
	: /*no output */
	: "r" (this) , "r" (v1) ,"r" (v2)
	: "memory" );
#endif
}

void IVP_U_Float_Point::inline_set_vert_to_area_defined_by_three_points(const IVP_U_Float_Point *tp0,const IVP_U_Float_Point *tp1,const IVP_U_Float_Point *tp2)
{
	unsigned int adress;
	unsigned int ali_adress;

	adress=(unsigned int)tp0;
	ali_adress=(adress & 0xfffffff0);
	IVP_ASSERT(adress==ali_adress);

	adress=(unsigned int)tp1;
	ali_adress=(adress & 0xfffffff0);
	IVP_ASSERT(adress==ali_adress);

	adress=(unsigned int)tp2;
	ali_adress=(adress & 0xfffffff0);
	IVP_ASSERT(adress==ali_adress);
	
#if 0
		IVP_DOUBLE a0, a1, a2, b0, b1, b2;
	// calculate (not-normized) normal
	IVP_DOUBLE fp0 = tp0->k[0];
	IVP_DOUBLE fp1 = tp0->k[1];
	IVP_DOUBLE fp2 = tp0->k[2];
	a0 = tp1->k[0] - fp0;
	a1 = tp1->k[1] - fp1;
	a2 = tp1->k[2] - fp2;
	b0 = tp2->k[0] - fp0;
	b1 = tp2->k[1] - fp1;
	b2 = tp2->k[2] - fp2;

	IVP_DOUBLE c0 = b1*a2 - a1*b2;
	IVP_DOUBLE c1 = b2*a0 - a2*b0;
	IVP_DOUBLE c2 = b0*a1 - a0*b1;
	
	// calculate hesse-area
	this->k[0]=	c0;
	this->k[1]=	c1;
	this->k[2]=	c2;
#else
	asm __volatile__
	("
		lqc2    vf3,0x0(%1)
		lqc2    vf5,0x0(%2)
		lqc2    vf4,0x0(%3)
		vsub.xyz vf4, vf4, vf3
		vsub.xyz vf5, vf5, vf3
		vopmula.xyz	ACC,vf4,vf5
		vopmsub.xyz	vf6,vf5,vf4
		sqc2    vf6,0x0(%0)
	"
	: /*no output */
	: "r" (this) , "r" (tp0) ,"r" (tp1), "r" (tp2)
	: "memory");
#endif
}

inline void IVP_U_Quat::init(){
	asm __volatile__("
	sqc2    vf0,0x0(%0)
	"
	: /* no output */
	: "r" (this)
	: "memory"    );
}

inline IVP_DOUBLE IVP_U_Quat::acos_quat(const IVP_U_Quat* q) const{ // acosinus between two quats
    return x * q->x + y * q->y + z * q->z + w * q->w;
}

inline void IVP_U_Quat::inline_set_mult_quat(const IVP_U_Quat* q1,const  IVP_U_Quat* q2) {
#if 0
  IVP_U_Quat *res=this;
  IVP_DOUBLE xx,yy,zz,ww;
  xx = q1->w * q2->x + q1->x * q2->w + q1->y * q2->z - q1->z * q2->y;
  yy = q1->w * q2->y + q1->y * q2->w + q1->z * q2->x - q1->x * q2->z;
  zz = q1->w * q2->z + q1->z * q2->w + q1->x * q2->y - q1->y * q2->x;
  ww = q1->w * q2->w - q1->x * q2->x - q1->y * q2->y - q1->z * q2->z;
  res->x = xx;
  res->y = yy;
  res->z = zz;
  res->w = ww;
#else
  asm __volatile__
	("
		lqc2    vf4,0x0(%1)	#q1
		lqc2    vf5,0x0(%2)	#q2

	 	vmul.xyzw vf12, vf4,vf5	    #vf12 x1x2 y1y2 z1z2 w1w2
		vmulw.xyz vf8,vf4,vf5	    #vf8  x1w2 y1w2 z1w2 w1w2
		vmulw.xyz vf9,vf5,vf4	    #vf9  w1x2 w1y2 w1z2 w1w2  
		#nop 
		vsubx.w vf12,vf12,vf12	    # vf12.w = ww - xx
		vopmula.xyz	ACC,vf4,vf5
		vopmsub.xyz	vf6,vf5,vf4

		vadd.xyz vf8, vf8, vf9
		vsuby.w vf12,vf12,vf12	    # vf12.w = ww - xx - yy
		#nop nop 
		vadd.xyz vf8, vf8,vf6
		vsubz.w vf8,vf12,vf12	     # vf8.w = ww - xx - yy - zz
		#nop nop nop
		sqc2    vf8,0x0(%0)
	"
	: /*no output */
	: "r" (this) , "r" (q1) ,"r" (q2)
	: "memory"    );
#endif
}

inline void IVP_U_Quat::inline_set_mult_quat(const IVP_U_Quat* q1,const  IVP_U_Float_Quat* q2) {
    this->inline_set_mult_quat( q1, (IVP_U_Quat *)q2 );
}

void IVP_U_Quat::normize_correct_step(int steps) {
    IVP_U_Quat *quat = this;
    IVP_DOUBLE square = quat->x * quat->x + quat->y * quat->y + quat->z * quat->z + quat->w * quat->w;
    square *= 0.5f;
    IVP_DOUBLE factor = 1.5f - square;
    if (steps > 1)    factor += 0.5f - ( factor * factor * square );
    if (steps > 2)    factor += 0.5f - ( factor * factor * square );
    if (steps > 3)    factor += 0.5f - ( factor * factor * square );
    if (steps > 4)    factor += 0.5f - ( factor * factor * square );
    if (steps > 5)    factor += 0.5f - ( factor * factor * square );
    quat->x *= factor;
    quat->y *= factor;
    quat->z *= factor;
    quat->w *= factor;
}

void IVP_U_Float_Quat::set(const IVP_U_Quat *source){
    x = (IVP_FLOAT)source->x;
    y = (IVP_FLOAT)source->y;
    z = (IVP_FLOAT)source->z;
    w = (IVP_FLOAT)source->w;
}


