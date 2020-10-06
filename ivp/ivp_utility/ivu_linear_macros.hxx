// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_U_LINEAR_MACROS_INCLUDED
#define _IVP_U_LINEAR_MACROS_INCLUDED


#ifndef WIN32
#	pragma interface
#endif

#if defined(IVP_USE_PS2_VU0)
#   include "ivu_linear_ps2.hxx"
#elif defined(IVP_PIII)
#   include "ivu_linear_PIII.hxx"
#else
#   include "ivu_linear_software.hxx"
#endif

#if !defined(IVP_NO_DOUBLE)
#   if defined(IVP_WILLAMETTE)
#      include "ivu_linear_willamette.hxx"
#   else
#      include "ivu_linear_double.hxx"
#   endif
#endif


inline void IVP_U_Float_Point::set(IVP_FLOAT k0, IVP_FLOAT k1, IVP_FLOAT k2){
    k[0] = k0; k[1] = k1; k[2] = k2;
}


inline void IVP_U_Float_Point3::set(const IVP_FLOAT p[3]){
    k[0]= p[0];
    k[1]= p[1];
    k[2]= p[2];
}

#if !defined(IVP_NO_DOUBLE)
inline void IVP_U_Float_Point3::set(const IVP_DOUBLE p[3]){
    k[0]= p[0];
    k[1]= p[1];
    k[2]= p[2];
}
#endif

inline void IVP_U_Float_Point::set(const IVP_FLOAT p[3]){
    k[0]= p[0];
    k[1]= p[1];
    k[2]= p[2];
}


inline void IVP_U_Float_Point::subtract(const IVP_U_Float_Point *v2){	// vector subtraction
    this->subtract(this,v2);
}

inline void IVP_U_Float_Point::add(const IVP_U_Float_Point *v2){	// vector subtraction
    this->add(this,v2);
}

inline void IVP_U_Float_Point::add_multiple(const IVP_U_Float_Point *v2, IVP_DOUBLE f){	// vector
    this->add_multiple(this,v2,f);
}

inline void IVP_U_Float_Point::set_multiple(const IVP_U_Quat *q_source, IVP_DOUBLE f){
    this->set_multiple( (IVP_U_Float_Point *)q_source, f);
}



//////////////////////////////////////////////////////////////////////

inline IVP_FLOAT IVP_Inline_Math::save_acosf(IVP_FLOAT angle) {
    //on windows compiler we ge not a number when angle >= 1.0
    return angle >= 1.0f ? 0.0f : IVP_Inline_Math::acosd(angle);
}


inline int IVP_Inline_Math::int_div_2(int a) {
#ifdef PSXII
    //workaround for compiler bug in CodeWarrior1.6
    if(a<0) {
	int h=-a;
	h=h>>1;
	return -h;
    } else {
	return a>>1;
    }
#else
    return a/2;
#endif
}

// an acos approximation whose absolute value is always less than abs(asin(angle))
// it differs not more than 0.0005 when abs(angle) < 0.5f
// and it differs with 0.2 when abs(angle) == 1.0f
inline IVP_FLOAT IVP_Inline_Math::fast_approx_asin(IVP_FLOAT angle) {
    IVP_FLOAT quad_angle = angle * angle;
    IVP_FLOAT triple_angle = angle * quad_angle;
    IVP_FLOAT quint_angle = triple_angle * quad_angle;
    return angle + (1.0f / 6.0f) * triple_angle + (3.0f / 40.0f) * quint_angle; // see mathematics book
}

inline IVP_FLOAT IVP_Inline_Math::approx5_sin(IVP_FLOAT angle) {
    IVP_FLOAT quad_angle = angle * angle;
    IVP_FLOAT triple_angle = angle * quad_angle;
    IVP_FLOAT quint_angle = triple_angle * quad_angle;
    return angle - (1.0f/6.0f) * triple_angle + (1.0f/120.0f) * quint_angle;
}

inline IVP_FLOAT IVP_Inline_Math::approx5_cos(IVP_FLOAT angle) {
    IVP_FLOAT quad_angle = angle * angle;
    return 1.0f - (1.0f/2.0f) * quad_angle + (1.0f/24.0f) * quad_angle * quad_angle;
}

inline IVP_FLOAT IVP_Inline_Math::upper_limit_asin(IVP_FLOAT angle) {
    IVP_FLOAT quad_angle = angle * angle;
    IVP_FLOAT triple_angle = angle * quad_angle;
    IVP_FLOAT quint_angle = triple_angle * quad_angle;
    return angle + (1.0f / 6.0f) * triple_angle + 0.40414f * quint_angle; // see mathematics book
}

// an acos approximation that differs not more than 0.0016 when abs(angle) < 0.57
// and that differs not more than 0.0521 in the whole range
inline IVP_FLOAT IVP_Inline_Math::fast_asin(IVP_FLOAT angle) {
    IVP_FLOAT quad_angle = angle * angle;
    IVP_FLOAT triple_angle = angle * quad_angle;
    IVP_FLOAT quint_angle = triple_angle * quad_angle;
    return angle + 0.12f * triple_angle + 0.29f * quint_angle;
}

// an acos approximation that differs not more than 0.022 from the original acos
// but differs nearly everywhere with an average of about 0.012
inline IVP_FLOAT IVP_Inline_Math::fast_anywhere_asin(IVP_FLOAT angle) {
    IVP_FLOAT quad_angle = angle * angle;
    IVP_FLOAT triple_angle = angle * quad_angle;
    IVP_FLOAT quint_angle = triple_angle * quad_angle;
    return 1.1f * angle - 0.419f * triple_angle + 0.76f * quint_angle;
}

inline IVP_RETURN_TYPE IVP_Inline_Math::invert_2x2_matrix(
    const IVP_DOUBLE a1_in, const IVP_DOUBLE b1_in,
    const IVP_DOUBLE a2_in, const IVP_DOUBLE b2_in,
    IVP_DOUBLE *i_a1_out, IVP_DOUBLE *i_b1_out,
    IVP_DOUBLE *i_a2_out, IVP_DOUBLE *i_b2_out)
{
    IVP_DOUBLE D; // main determinant
    D = (a1_in * b2_in) - (b1_in * a2_in);
    if(D*D < P_DOUBLE_EPS * P_DOUBLE_EPS){
	return IVP_FAULT;  // may be an ordinary result
    }
    
    IVP_DOUBLE DI = 1.0f/D;
    i_a1_out[0] =  b2_in * DI;
    i_b1_out[0] = -a2_in * DI;
    i_a2_out[0] = -b1_in * DI;
    i_b2_out[0] =  a1_in * DI;
    return IVP_OK;
}

//////////////////////////////////////////////////////////////////////


inline void IVP_U_Matrix3::get_row(IVP_COORDINATE_INDEX row, IVP_U_Point *row_out) const {
    row_out->set( &rows[row] );
}

inline void IVP_U_Matrix3::get_row(IVP_COORDINATE_INDEX row, IVP_U_Float_Point *row_out) const {
    row_out->set( &rows[row] );
}

inline void IVP_U_Matrix3::get_col(IVP_COORDINATE_INDEX col, IVP_U_Point *col_out) const {
    col_out->k[0] = get_elem(0,col);
    col_out->k[1] = get_elem(1,col);
    col_out->k[2] = get_elem(2,col);
}

inline void IVP_U_Matrix3::get_col(IVP_COORDINATE_INDEX col, IVP_U_Float_Point *col_out) const {
    col_out->k[0] = get_elem(0,col);
    col_out->k[1] = get_elem(1,col);
    col_out->k[2] = get_elem(2,col);
}

inline void IVP_U_Matrix3::set_row(IVP_COORDINATE_INDEX row, const IVP_U_Point *row_in) {
    rows[row].set( row_in );
}

inline void IVP_U_Matrix3::set_row(IVP_COORDINATE_INDEX row, const IVP_U_Float_Point *row_in) {
    rows[row].set( row_in );
}

inline void IVP_U_Matrix3::set_col(IVP_COORDINATE_INDEX col, const IVP_U_Point *col_in) {
    set_elem( 0,col, col_in->k[0]);
    set_elem( 1,col, col_in->k[1]);
    set_elem( 2,col, col_in->k[2]);
}

inline void IVP_U_Matrix3::set_col(IVP_COORDINATE_INDEX col, const IVP_U_Float_Point *col_in) {
    set_elem( 0,col, col_in->k[0]);
    set_elem( 1,col, col_in->k[1]);
    set_elem( 2,col, col_in->k[2]);
}





inline IVP_U_Point::IVP_U_Point(const IVP_U_Float_Point &p)    {
  this->set(&p);
};

inline IVP_U_Float_Point::IVP_U_Float_Point(const IVP_U_Float_Point *p)    {
  this->set(p);
};


#endif
