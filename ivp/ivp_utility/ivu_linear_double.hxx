// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC


inline void IVP_U_Float_Point::set(const IVP_U_Point *p_source){
    IVP_DOUBLE a = p_source->k[0];
    IVP_DOUBLE b = p_source->k[1];
    IVP_DOUBLE c = p_source->k[2];
    k[0]= (IVP_FLOAT)a;
    k[1]= (IVP_FLOAT)b;
    k[2]= (IVP_FLOAT)c;
}

inline void IVP_U_Float_Point::subtract(const IVP_U_Point *v1,const IVP_U_Point *v2){
    IVP_DOUBLE a,b,c;
    a = v1->k[0] - v2->k[0];
    b = v1->k[1] - v2->k[1];
    c = v1->k[2] - v2->k[2];

    k[0] = (IVP_FLOAT)a;
    k[1] = (IVP_FLOAT)b;
    k[2] = (IVP_FLOAT)c;
}

inline void IVP_U_Float_Point::set_multiple(const IVP_U_Point *v, IVP_DOUBLE f){
    IVP_DOUBLE a = v->k[0] * f;
    IVP_DOUBLE b = v->k[1] * f;
    IVP_DOUBLE c = v->k[2] * f;
    k[0]= (IVP_FLOAT)a;
    k[1]= (IVP_FLOAT)b;
    k[2]= (IVP_FLOAT)c;
}

inline void IVP_U_Float_Point::inline_subtract_and_mult(const IVP_U_Point *v1,const IVP_U_Point *v2, IVP_DOUBLE factor){	// vektor addition
    IVP_DOUBLE a,b,c;
    a = (v1->k[0] - v2->k[0]);
    b = (v1->k[1] - v2->k[1]);	a *= factor;
    c = (v1->k[2] - v2->k[2]);	b *= factor;
    c *=factor;
    k[0] = a;
    k[1] = b;
    k[2] = c;
}


inline void IVP_U_Point::set_to_zero() {
    k[0]=0.0f;
    k[1]=0.0f;
    k[2]=0.0f;
}

inline void IVP_U_Point::set(IVP_DOUBLE k0, IVP_DOUBLE k1, IVP_DOUBLE k2){
    k[0] = k0; k[1] = k1; k[2] = k2;
}
inline void IVP_U_Point::set(const IVP_FLOAT p[3]){
    k[0]= p[0];
    k[1]= p[1];
    k[2]= p[2];
}
inline void IVP_U_Point::set(const IVP_U_Quat *q_source){
    IVP_DOUBLE a = q_source->x;
    IVP_DOUBLE b = q_source->y;
    IVP_DOUBLE c = q_source->z;
    k[0]= (IVP_FLOAT)a;
    k[1]= (IVP_FLOAT)b;
    k[2]= (IVP_FLOAT)c;
}

inline void IVP_U_Point::set(const IVP_U_Point *p_source){
    IVP_DOUBLE a = p_source->k[0];
    IVP_DOUBLE b = p_source->k[1];
    IVP_DOUBLE c = p_source->k[2];
    k[0]= a;
    k[1]= b;
    k[2]= c;
}
inline void IVP_U_Point::set(const IVP_U_Float_Point *p_source){
    IVP_DOUBLE a = p_source->k[0];
    IVP_DOUBLE b = p_source->k[1];
    IVP_DOUBLE c = p_source->k[2];
    k[0]= a;
    k[1]= b;
    k[2]= c;
}


inline void IVP_U_Point::set_multiple(const IVP_U_Quat *q_source, IVP_DOUBLE f){
    IVP_DOUBLE a = q_source->x * f;
    IVP_DOUBLE b = q_source->y * f;
    IVP_DOUBLE c = q_source->z * f;
    k[0]= a;
    k[1]= b;
    k[2]= c;
}

inline void IVP_U_Point::set_negative(const IVP_U_Point *p_source){
    IVP_DOUBLE a = p_source->k[0];
    IVP_DOUBLE b = p_source->k[1];	a = -a;
    IVP_DOUBLE c = p_source->k[2];	b = -b;
                                c = -c;
    k[0]= a;
    k[1]= b;
    k[2]= c;
}

inline void IVP_U_Point::mult(IVP_DOUBLE factor){
    IVP_DOUBLE a,b,c;
    a = k[0] * factor;
    b = k[1] * factor;
    c = k[2] * factor;
    k[0] = a; k[1] = b; k[2] = c;
}

inline void IVP_U_Point::add_multiple(const IVP_U_Point *v, IVP_DOUBLE factor){
    IVP_DOUBLE a,b,c;
    a = v->k[0] * factor;
    b = v->k[1] * factor;	a += k[0];
    c = v->k[2] * factor;	b += k[1];
                                c += k[2];
    k[0] = a; k[1] = b; k[2] = c;
}

inline void IVP_U_Point::add_multiple(const IVP_U_Float_Point *v, IVP_DOUBLE factor){
    IVP_DOUBLE a,b,c;
    a = v->k[0] * factor;
    b = v->k[1] * factor;	a += k[0];
    c = v->k[2] * factor;	b += k[1];
                                c += k[2];
    k[0] = a; k[1] = b; k[2] = c;
}

inline void IVP_U_Point::add_multiple (const IVP_U_Point *v1, const IVP_U_Point *v2, IVP_DOUBLE factor2){	// vektor addition
    IVP_DOUBLE a,b,c;

    a = v2->k[0] * factor2;
    b = v2->k[1] * factor2;    a += v1->k[0];
    c = v2->k[2] * factor2;    b += v1->k[1];
                               c += v1->k[2];
    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::add_multiple (const IVP_U_Point *v1, const IVP_U_Float_Point *v2, IVP_DOUBLE factor2){	// vektor addition
    IVP_DOUBLE a,b,c;

    a = v2->k[0] * factor2;
    b = v2->k[1] * factor2;    a += v1->k[0];
    c = v2->k[2] * factor2;    b += v1->k[1];
                               c += v1->k[2];
    k[0] = a;
    k[1] = b;
    k[2] = c;
}


inline void IVP_U_Point::set_pairwise_mult (const IVP_U_Point *v1, const IVP_U_Point *v2){	// pairwise multiple 
    IVP_DOUBLE a,b,c;
    a = v1->k[0] * v2->k[0];
    b = v1->k[1] * v2->k[1];
    c = v1->k[2] * v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::add(const IVP_U_Point *v2){	// vector addition
    IVP_DOUBLE a,b,c;
    a = k[0] + v2->k[0];
    b = k[1] + v2->k[1];
    c = k[2] + v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::add(const IVP_U_Float_Point *v2){	// vector addition
    IVP_DOUBLE a,b,c;
    a = k[0] + v2->k[0];
    b = k[1] + v2->k[1];
    c = k[2] + v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::add(const IVP_U_Point *v1, const IVP_U_Point *v2){
    IVP_DOUBLE a = v1->k[0] + v2->k[0];
    IVP_DOUBLE b = v1->k[1] + v2->k[1];
    IVP_DOUBLE c = v1->k[2] + v2->k[2];
    k[0] = a;k[1] = b;k[2] = c;  
}

inline void IVP_U_Point::add(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2){
    IVP_DOUBLE a = v1->k[0] + v2->k[0];
    IVP_DOUBLE b = v1->k[1] + v2->k[1];
    IVP_DOUBLE c = v1->k[2] + v2->k[2];
    k[0] = a;k[1] = b;k[2] = c;  
}

void IVP_U_Point::set_multiple(const IVP_U_Point *p,IVP_DOUBLE f){
    IVP_DOUBLE a = p->k[0] * f;
    IVP_DOUBLE b = p->k[1] * f;
    IVP_DOUBLE c = p->k[2] * f;
    k[0] = a;k[1] = b;k[2] = c;
}



void IVP_U_Point::set_multiple(const IVP_U_Float_Point *p,IVP_DOUBLE f){
    IVP_DOUBLE a = p->k[0] * f;
    IVP_DOUBLE b = p->k[1] * f;
    IVP_DOUBLE c = p->k[2] * f;
    k[0] = a;k[1] = b;k[2] = c;
}

inline void IVP_U_Point::subtract(const IVP_U_Point *v2){	// vector subtraction
    IVP_DOUBLE a,b,c;
    a = k[0] - v2->k[0];
    b = k[1] - v2->k[1];
    c = k[2] - v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::subtract(const IVP_U_Float_Point *v2){	// vector subtraction
    IVP_DOUBLE a,b,c;
    a = k[0] - v2->k[0];
    b = k[1] - v2->k[1];
    c = k[2] - v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::subtract(const IVP_U_Point *v1,const IVP_U_Point *v2){
    IVP_DOUBLE a,b,c;
    a = v1->k[0] - v2->k[0];
    b = v1->k[1] - v2->k[1];
    c = v1->k[2] - v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::subtract(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2)
{
    // special conversion vector subtraction: (doubleP) = (floatP) - (floatP)
    IVP_DOUBLE a,b,c;
    a = v1->k[0] - v2->k[0];
    b = v1->k[1] - v2->k[1];
    c = v1->k[2] - v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}


inline void IVP_U_Point::subtract(const IVP_U_Point *v1, const IVP_U_Float_Point *v2)
{
    // special conversion vector subtraction: (doubleP) = (doubleP) - (floatP)
    IVP_DOUBLE a,b,c;
    a = v1->k[0] - v2->k[0];
    b = v1->k[1] - v2->k[1];
    c = v1->k[2] - v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::subtract(const IVP_U_Float_Point *v1, const IVP_U_Point *v2)
{
    // special conversion vector subtraction: (doubleP) = (floatP) - (doubleP)
    IVP_DOUBLE a,b,c;
    a = v1->k[0] - v2->k[0];
    b = v1->k[1] - v2->k[1];
    c = v1->k[2] - v2->k[2];

    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::inline_subtract_and_mult(const IVP_U_Point *v1,const IVP_U_Point *v2, IVP_DOUBLE factor){	// vektor addition
    IVP_DOUBLE a,b,c;
    a = (v1->k[0] - v2->k[0]);
    b = (v1->k[1] - v2->k[1]);	a *= factor;
    c = (v1->k[2] - v2->k[2]);	b *= factor;
    c *=factor;
    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline void IVP_U_Point::inline_subtract_and_mult(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2, IVP_DOUBLE factor){	// vektor addition
    IVP_DOUBLE a,b,c;
    a = (v1->k[0] - v2->k[0]);
    b = (v1->k[1] - v2->k[1]);	a *= factor;
    c = (v1->k[2] - v2->k[2]);	b *= factor;
    c *=factor;
    k[0] = a;
    k[1] = b;
    k[2] = c;
}

inline IVP_DOUBLE IVP_U_Point::quad_length() const {
    return (k[0]*k[0] + k[1]*k[1] + k[2]*k[2]);
}

inline IVP_DOUBLE IVP_U_Point::quad_distance_to(const IVP_U_Point *p)const{
    IVP_DOUBLE a = k[0] - p->k[0]; a*=a;
    IVP_DOUBLE b = k[1] - p->k[1]; b*=b;
    IVP_DOUBLE c = k[2] - p->k[2]; c*=c;
    return a+b+c;
}

inline IVP_DOUBLE IVP_U_Point::quad_distance_to(const IVP_U_Float_Point *p)const{
    IVP_DOUBLE a = k[0] - p->k[0]; a*=a;
    IVP_DOUBLE b = k[1] - p->k[1]; b*=b;
    IVP_DOUBLE c = k[2] - p->k[2]; c*=c;
    return a+b+c;
}

inline void IVP_U_Matrix::inline_vimult4( const IVP_U_Point *p_in, IVP_U_Point * p_out )const{
    IVP_DOUBLE a = p_in->k[0] - vv.k[0];
    IVP_DOUBLE b = p_in->k[1] - vv.k[1];
    IVP_DOUBLE c = p_in->k[2] - vv.k[2];
    
    p_out->k[0] = get_elem(0,0)*a + get_elem(1,0)*b + get_elem(2,0)*c;
    p_out->k[1] = get_elem(0,1)*a + get_elem(1,1)*b + get_elem(2,1)*c;
    p_out->k[2] = get_elem(0,2)*a + get_elem(1,2)*b + get_elem(2,2)*c;
};

inline void IVP_U_Matrix::inline_vimult4( const IVP_U_Point *p_in, IVP_U_Float_Point * p_out )const{
    IVP_DOUBLE a = p_in->k[0] - vv.k[0];
    IVP_DOUBLE b = p_in->k[1] - vv.k[1];
    IVP_DOUBLE c = p_in->k[2] - vv.k[2];
    
    p_out->k[0] = get_elem(0,0)*a + get_elem(1,0)*b + get_elem(2,0)*c;
    p_out->k[1] = get_elem(0,1)*a + get_elem(1,1)*b + get_elem(2,1)*c;
    p_out->k[2] = get_elem(0,2)*a + get_elem(1,2)*b + get_elem(2,2)*c;
};

inline void IVP_U_Matrix3::inline_vmult3( const IVP_U_Point *p_in, IVP_U_Point * p_out ) const{
	IVP_DOUBLE a = get_elem(0,0)*p_in->k[0] + get_elem(0,1)*p_in->k[1] + get_elem(0,2)*p_in->k[2];
	IVP_DOUBLE b = get_elem(1,0)*p_in->k[0] + get_elem(1,1)*p_in->k[1] + get_elem(1,2)*p_in->k[2];
	IVP_DOUBLE c = get_elem(2,0)*p_in->k[0] + get_elem(2,1)*p_in->k[1] + get_elem(2,2)*p_in->k[2];
	p_out->k[0] = a;p_out->k[1] = b; p_out->k[2] = c;
};

inline void IVP_U_Matrix3::inline_vimult3( const IVP_U_Point *p_in, IVP_U_Point * p_out )const{
    IVP_DOUBLE a = get_elem(0,0)*p_in->k[0] + get_elem(1,0)*p_in->k[1] + get_elem(2,0)*p_in->k[2];
    IVP_DOUBLE b = get_elem(0,1)*p_in->k[0] + get_elem(1,1)*p_in->k[1] + get_elem(2,1)*p_in->k[2];
    IVP_DOUBLE c = get_elem(0,2)*p_in->k[0] + get_elem(1,2)*p_in->k[1] + get_elem(2,2)*p_in->k[2];
	p_out->k[0] = a;p_out->k[1] = b; p_out->k[2] = c;
}; // eigentlich transformation

inline void IVP_U_Matrix::inline_vmult4(const IVP_U_Point *p_in, IVP_U_Point * p_out)const{
    
    IVP_DOUBLE h = p_in->k[0];
    IVP_DOUBLE a = h * get_elem(0,0);
    IVP_DOUBLE b = h * get_elem(1,0);
    IVP_DOUBLE c = h * get_elem(2,0);
    IVP_DOUBLE x,y;
    h = p_in->k[1]; x = h*get_elem(0,1);	y = h*get_elem(1,1);	a += x;	x = h*get_elem(2,1);	 b+=y, c+= x;
    h = p_in->k[2]; x = h*get_elem(0,2);	y = h*get_elem(1,2);	a += x;	x = h*get_elem(2,2);	 b+=y, c+= x;
    a += vv.k[0];    b += vv.k[1];    c += vv.k[2];
    p_out->k[0] = a;p_out->k[1] = b; p_out->k[2] = c;
}


inline void IVP_U_Matrix::inline_vmult4(const IVP_U_Float_Point *p_in, IVP_U_Point * p_out)const{
    
    IVP_DOUBLE h = p_in->k[0];
    IVP_DOUBLE a = h * get_elem(0,0);
    IVP_DOUBLE b = h * get_elem(1,0);
    IVP_DOUBLE c = h * get_elem(2,0);
    IVP_DOUBLE x,y;
    h = p_in->k[1]; x = h*get_elem(0,1);	y = h*get_elem(1,1);	a += x;	x = h*get_elem(2,1);	 b+=y, c+= x;
    h = p_in->k[2]; x = h*get_elem(0,2);	y = h*get_elem(1,2);	a += x;	x = h*get_elem(2,2);	 b+=y, c+= x;
    a += vv.k[0];    b += vv.k[1];    c += vv.k[2];
    p_out->k[0] = a;p_out->k[1] = b; p_out->k[2] = c;
}

void IVP_U_Point::inline_calc_cross_product(const IVP_U_Point *v1,const IVP_U_Point *v2)
{
    IVP_DOUBLE a = v1->k[1]*v2->k[2];
    IVP_DOUBLE a2 = v2->k[1]*v1->k[2];
    IVP_DOUBLE b = v1->k[2]*v2->k[0];	
    IVP_DOUBLE b2 = v2->k[2]*v1->k[0];	a -= a2;
    IVP_DOUBLE c = v1->k[0]*v2->k[1];
    IVP_DOUBLE c2 = v2->k[0]*v1->k[1];	b -= b2;
    k[0] = a;k[1] = b;			c -= c2;
    k[2] = c;  
}

void IVP_U_Point::inline_set_vert_to_area_defined_by_three_points(const IVP_U_Point *tp0,const IVP_U_Point *tp1,const IVP_U_Point *tp2)
{
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
}

void IVP_U_Point::inline_set_vert_to_area_defined_by_three_points(const IVP_U_Float_Point *tp0,const IVP_U_Float_Point *tp1,const IVP_U_Point *tp2)
{
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
}

void IVP_U_Point::inline_set_vert_to_area_defined_by_three_points(const IVP_U_Float_Point *tp0,const IVP_U_Float_Point *tp1,const IVP_U_Float_Point *tp2){
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
}

void IVP_U_Point::inline_calc_cross_product_and_normize(const IVP_U_Point *v1,const IVP_U_Point *v2)
{
    IVP_DOUBLE a = v1->k[1]*v2->k[2];
    IVP_DOUBLE a2 = v2->k[1]*v1->k[2];
    IVP_DOUBLE b = v1->k[2]*v2->k[0];	
    IVP_DOUBLE b2 = v2->k[2]*v1->k[0];	a -= a2;
    IVP_DOUBLE c = v1->k[0]*v2->k[1];
    IVP_DOUBLE c2 = v2->k[0]*v1->k[1];	b -= b2;
    k[0] = (IVP_FLOAT)a;k[1] = (IVP_FLOAT)b;			c -= c2;
    k[2] = (IVP_FLOAT)c;
    this->normize();
}


inline IVP_DOUBLE IVP_U_Hesse::get_dist(const IVP_U_Point *p) const {
    return this->dot_product(p) + hesse_val;
}

inline IVP_U_Float_Point::IVP_U_Float_Point(const IVP_U_Point *p)    {
  this->set(p);
};
