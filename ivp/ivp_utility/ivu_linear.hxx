// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_U_LINEAR_INCLUDED
#define _IVP_U_LINEAR_INCLUDED

#ifdef IVP_WINDOWS_ALIGN16
#pragma pack(push,16)
#endif

#ifdef WIN32
#	pragma warning( disable : 4244 ) 
#endif
#ifndef WIN32
#	pragma interface
#endif

class IVP_U_Straight;
class IVP_U_Hesse;
class IVP_U_Quat;
class IVP_U_Point;
class IVP_U_Float_Quat;


/********************************************************************************
 *	Name:	      	IVP_COORDINATE_INDEX
 *	Description:	Use these enums everywhere where you want to specify a coodinate.
 ********************************************************************************/
enum IVP_COORDINATE_INDEX {
    IVP_INDEX_X = 0,  // you can rely on these
    IVP_INDEX_Y = 1,  // numbers - they will
    IVP_INDEX_Z = 2   // never be changed.
};


/********************************************************************************
 *	Name:	      	IVP_U_Float_Point 	
 *	Description:	A float triple used for various purposes.
 ********************************************************************************/

class IVP_U_Float_Point3 {
public:
    IVP_FLOAT k[3];
    inline void set(const IVP_FLOAT p[3]);
#if !defined(IVP_NO_DOUBLE)
    inline void set(const IVP_DOUBLE p[3]);
#endif

	inline void byte_swap() {   ivp_byte_swap4( (uint&) k[0] );
								ivp_byte_swap4( (uint&) k[1] ); 
								ivp_byte_swap4( (uint&) k[2] ); }
};

class IVP_U_Float_Point {
public:
    IVP_FLOAT k[3];
#ifdef IVP_VECTOR_UNIT_FLOAT
    union {
	IVP_FLOAT hesse_val;
    };
#endif

#if !defined(IVP_NO_DOUBLE)
    inline void set(const IVP_U_Point *p);
#endif
    inline void set(const IVP_FLOAT p[3]);
    inline void set(const IVP_U_Float_Point *p);
    inline void set(IVP_FLOAT k0, IVP_FLOAT k1, IVP_FLOAT k2);			

    inline void set_negative(const IVP_U_Float_Point *p_source);	// this = p_source * -1.0
    inline void set_to_zero();					// i.e. set(0.0, 0.0, 0.0)
    
    inline IVP_DOUBLE quad_length()const;				// length of vector square: k[0]*k[0] + k[1]*k[1] + k[2]*k[2]

    inline void mult(IVP_DOUBLE factor);				// this *= factor

    inline void add(const IVP_U_Float_Point *v2); 				// this += v2
    inline void add(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2);	// this = v1 + v2;

    inline void add_multiple(const IVP_U_Float_Point *v, IVP_DOUBLE factor);	// this += v1 * factor
    inline void add_multiple(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2, IVP_DOUBLE factor2);
    
    inline void set_multiple(const IVP_U_Quat *q_source,IVP_DOUBLE f);		// this = q_source.xyz * f
#if !defined(IVP_NO_DOUBLE)
    inline void set_multiple(const IVP_U_Point *v,IVP_DOUBLE f);		// this = q_source.xyz * f
#endif
    inline void set_multiple(const IVP_U_Float_Point *v,IVP_DOUBLE f);		// this = q_source.xyz * f
    inline void set_pairwise_mult(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2);    // this->k[i] = v1->k[i] * v2->k[i]

#if !defined(IVP_NO_DOUBLE)
    inline void subtract(const IVP_U_Point *v1, const IVP_U_Point *v2);		// this = v1 - v2
#endif
    inline void subtract(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2);		// this = v1 - v2
    inline void subtract(const IVP_U_Float_Point *v2);				// this = this - v2;

    inline void inline_subtract_and_mult(const IVP_U_Float_Point *v1,const IVP_U_Float_Point *v2, IVP_DOUBLE factor); // this = (v1-v2) * factor
#if !defined(IVP_NO_DOUBLE)
    inline void inline_subtract_and_mult(const IVP_U_Point *v1,const IVP_U_Point *v2, IVP_DOUBLE factor); // this = (v1-v2) * factor
#endif 
    inline IVP_DOUBLE dot_product(const IVP_U_Float_Point *v2) const;

    inline void inline_calc_cross_product(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2); // this = v1 x v2
    inline void inline_calc_cross_product_and_normize(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2); // this = v1 x v2

    inline void inline_set_vert_to_area_defined_by_three_points(const IVP_U_Float_Point *tp0,const IVP_U_Float_Point *tp1,const IVP_U_Float_Point *tp2);
    void calc_cross_product(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2); // this = v1 x v2

    void line_sqrt(); // Sqrt of each element: k[i] = sqrt(k[i])    
    
    void set_interpolate(const IVP_U_Float_Point *p0,const IVP_U_Float_Point *p1, IVP_DOUBLE s);
    void set_orthogonal_part(const IVP_U_Float_Point *vector,const IVP_U_Float_Point *normal_v);   //project vector on surface given by normal_v

    void rotate(IVP_COORDINATE_INDEX axis, IVP_FLOAT angle);   // rotate the point around origin

    
    IVP_DOUBLE real_length()const;		// real length = sqrt(quad_length)
    IVP_DOUBLE fast_real_length()const;	// real length with 0.1f% error, really fast
    IVP_DOUBLE real_length_plus_normize();	// Normize vector and return former real length.
    inline IVP_DOUBLE quad_distance_to(const IVP_U_Float_Point *p)const; // quad distance from this point to another

    IVP_RETURN_TYPE normize();		// normize vector (-> real_length == 1.0f)
    IVP_RETURN_TYPE fast_normize();	// normize vector (0.1f% error)
    void print(const char *comment = 0) const;

    IVP_U_Float_Point(){;};
    IVP_U_Float_Point(IVP_DOUBLE x, IVP_DOUBLE y,IVP_DOUBLE z){ k[0] = (IVP_FLOAT)x; k[1] = (IVP_FLOAT)y; k[2] = (IVP_FLOAT)z;};
    IVP_U_Float_Point(const IVP_U_Float_Point *p);

#if !defined(IVP_NO_DOUBLE)
    IVP_U_Float_Point(const IVP_U_Point *p);
#endif

	inline void byte_swap() {   ivp_byte_swap4( (uint&) k[0] );
								ivp_byte_swap4( (uint&) k[1] ); 
								ivp_byte_swap4( (uint&) k[2] ); 
						#ifdef IVP_VECTOR_UNIT_FLOAT
								ivp_byte_swap4( (uint&) hesse_val ); 
						#endif
							}

} IVP_ALIGN_16;



/********************************************************************************
 *	Name:	      	IVP_U_Point 	
 *	Description:	A double triple used for various purposes.
 *	Note:		Some functions change 'this', other functions change
 *			parameters (check the const attribute).
 *			The inline implementation can be found in ivu_linear_macros.hxx
 *			and ivu_matrix_macros.hxx
 *	Performance Notes: All functions are optimized (on an assembler basis) to
 *			run best on Pentium and Pentium II CPU.
 ********************************************************************************/

#if defined(IVP_NO_DOUBLE)
class IVP_U_Point: public IVP_U_Float_Point {
public:

#else
class IVP_U_Point {
public:
    IVP_DOUBLE k[3];
#	ifdef IVP_VECTOR_UNIT_DOUBLE    
    union {
	IVP_DOUBLE hesse_val;
    };
#	endif    
    inline IVP_DOUBLE dot_product(const IVP_U_Point *v2) const   {    return( k[0]*v2->k[0] + k[1]*v2->k[1] + k[2]*v2->k[2] );   };
    inline IVP_DOUBLE dot_product(const IVP_U_Float_Point *v2) const   {    return( k[0]*v2->k[0] + k[1]*v2->k[1] + k[2]*v2->k[2] );   };
    
    inline void inline_calc_cross_product(const IVP_U_Point *v1, const IVP_U_Point *v2); // this = v1 x v2
    inline void inline_calc_cross_product_and_normize(const IVP_U_Point *v1, const IVP_U_Point *v2); // this = v1 x v2
    void calc_cross_product(const IVP_U_Point *v1, const IVP_U_Point *v2); // this = v1 x v2

    inline void set_to_zero();							// == set(0.0, 0.0, 0.0)
    inline void set(const IVP_U_Point *p_source);				// this = p_source
    inline void set(const IVP_U_Float_Point *p_source);				// this = (IVP_DOUBLE)p_source
    inline void set(const IVP_U_Quat *q_source);				// this = q_source.xyz
    inline void set(IVP_DOUBLE k0, IVP_DOUBLE k1, IVP_DOUBLE k2 = 0.0f);			
    inline void set(const IVP_FLOAT p[3]);			    


    inline IVP_DOUBLE quad_length()const;	 	// Square length of vector: k[0]*k[0] + k[1]*k[1] + k[2]*k[2].
    inline IVP_DOUBLE quad_distance_to(const IVP_U_Point *p)const; // Square distance from this point to another
    inline IVP_DOUBLE quad_distance_to(const IVP_U_Float_Point *p)const; // Square distance from this point to another.
    
    IVP_DOUBLE real_length()const;		// Returns real length (= sqrt(quad_length)).
    IVP_DOUBLE real_length_plus_normize();	// Normize vector and return former real length.
    IVP_DOUBLE fast_real_length()const;	// Returns real length with max. 0.1f% error, really fast if in cache.
    IVP_RETURN_TYPE normize();		// Normize vector (-> real length == 1.0f), return IVP_FALSE if length too small.
    IVP_RETURN_TYPE fast_normize();	// Normize vector (0.1f% error)
    
    

    inline void add(const IVP_U_Point *v2); 					// this = this + v2;
    inline void add(const IVP_U_Float_Point *v2); 				// this = this + v2;
    inline void add(const IVP_U_Point *v1, const IVP_U_Point *v2);	// this = v1 + v2;
    inline void add(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2);	// this = v1 + v2;
    inline void add_multiple(const IVP_U_Point *v, IVP_DOUBLE factor);		// this += v1 * factor
    inline void add_multiple(const IVP_U_Float_Point *v, IVP_DOUBLE factor);	// this += v1 * factor
    inline void add_multiple(const IVP_U_Point *v1, const IVP_U_Point *v2, IVP_DOUBLE factor2); // this = v1 + factor2*v2  .
    inline void add_multiple(const IVP_U_Point *v1, const IVP_U_Float_Point *v2, IVP_DOUBLE factor2); // this = v1 + factor2*v2  .

    
    inline void subtract(const IVP_U_Point *v2); 				// this = this - v2;
    inline void subtract(const IVP_U_Float_Point *v2); 				// this = this - v2;
    inline void subtract(const IVP_U_Point *v1,const IVP_U_Point *v2); 		// this = v1 - v2;
    inline void subtract(const IVP_U_Float_Point *v1, const IVP_U_Float_Point *v2); // this = v1 - v2;
    inline void subtract(const IVP_U_Float_Point *v1, const IVP_U_Point *v2); // this = v1 - v2;
    inline void subtract(const IVP_U_Point *v1, const IVP_U_Float_Point *v2); // this = v1 - v2;
    inline void inline_subtract_and_mult(const IVP_U_Point *v1,const IVP_U_Point *v2, IVP_DOUBLE factor); // this = (v1-v2) * factor
    inline void inline_subtract_and_mult(const IVP_U_Float_Point *v1,const IVP_U_Float_Point *v2, IVP_DOUBLE factor); // this = (v1-v2) * factor
    inline void set_negative(const IVP_U_Point *p_source);			// this = p_source * -1.0

    inline void mult(IVP_DOUBLE factor);						// this *= factor
    inline void set_multiple(const IVP_U_Point *p,IVP_DOUBLE f);		// this = p * f
    inline void set_multiple(const IVP_U_Float_Point *p,IVP_DOUBLE f);	// this = p * f
    inline void set_multiple(const IVP_U_Quat *q_source,IVP_DOUBLE f);		// this = q_source.xyz * f
    inline void set_pairwise_mult(const IVP_U_Point *v1, const IVP_U_Point *v2);    // this->k[i] = v1->k[i] * v2->k[i]
    inline void set_pairwise_mult(const IVP_U_Point *v1, const IVP_U_Float_Point *v2);    // this->k[i] = v1->k[i] * v2->k[i]
    

    inline void inline_set_vert_to_area_defined_by_three_points(const IVP_U_Point *tp0,const IVP_U_Point *tp1,const IVP_U_Point *tp2);
    inline void inline_set_vert_to_area_defined_by_three_points(const IVP_U_Float_Point *tp0,const IVP_U_Float_Point *tp1,const IVP_U_Point *tp2);
    inline void inline_set_vert_to_area_defined_by_three_points(const IVP_U_Float_Point *tp0,const IVP_U_Float_Point *tp1,const IVP_U_Float_Point *tp2);

    void set_interpolate(const IVP_U_Point *p0,const IVP_U_Point *p1, IVP_DOUBLE s);	/* linear interpolation between p0 and p1:
											 * this = p1 * s + p0 * (1.0f-s) */
    void set_interpolate(const IVP_U_Float_Point *p0,const IVP_U_Float_Point *p1, IVP_DOUBLE s);	/* linear interpolation between p0 and p1:
												 * this = p1 * s + p0 * (1.0f-s) */
#endif
    
    /***** The following function change 'this' ****/ 
    // INTERN_START
    IVP_BOOL is_parallel(const IVP_U_Point *v2, IVP_DOUBLE eps) const; // checks length of crossproduct against eps
    
    void calc_an_orthogonal(const IVP_U_Point *ip);
    
    void solve_quadratic_equation_fast(const IVP_U_Point *a);	/* Solve a->k[0]*x*x + a->k[1]*x + a->k[2] = 0,
								 * this->k[0] = term under squareroot, this->k[1] = smaller value
								 * this->k[2] = greater value,
								 * if this->k[0] < 0: no result.
								 * Uses fast squareroot */
    void solve_quadratic_equation_accurate(const IVP_U_Point *a);	/* Solve a->k[0]*x*x + a->k[1]*x + a->k[2] = 0,
									 * this->k[0] = term under squareroot, this->k[1] = smaller value
									 * this->k[2] = greater value,
									 * if this->k[0] < 0: no result. */

    // INTERN_END


    void set_orthogonal_part(const IVP_U_Point *vector,const IVP_U_Point *normal_v); //project vector on surface given by normal_v
    void set_orthogonal_part(const IVP_U_Point *vector,const IVP_U_Float_Point *normal_v); //project vector on surface given by normal_v
    IVP_RETURN_TYPE set_crossing(IVP_U_Hesse *h0, IVP_U_Hesse *h1, IVP_U_Hesse *h2); // set this to the crossing of three areas

    void rotate(IVP_COORDINATE_INDEX axis, IVP_FLOAT angle);   // rotate the point around origin
    
    void line_min(const IVP_U_Point *p); // sets this to linewise min
    void line_max(const IVP_U_Point *p); // sets this to linewise max
    
    void print(const char *comment = 0);

    IVP_U_Point(){;};
    inline IVP_U_Point(const IVP_U_Float_Point &p);
    IVP_U_Point(IVP_DOUBLE x, IVP_DOUBLE y,IVP_DOUBLE z){ k[0] = x; k[1] = y; k[2] = z;};

	inline void byte_swap() { IVP_ASSERT( 0 && "No byte swap for doubles yet"); CORE; }

} IVP_ALIGN_16;



class IVP_Inline_Math {
public:
    static inline IVP_RETURN_TYPE invert_2x2_matrix(const IVP_DOUBLE a1_in, const IVP_DOUBLE b1_in,
						    const IVP_DOUBLE a2_in, const IVP_DOUBLE b2_in,
						    IVP_DOUBLE *i_a1_out, IVP_DOUBLE *i_b1_out,
						    IVP_DOUBLE *i_a2_out, IVP_DOUBLE *i_b2_out);
    static inline IVP_FLOAT approx5_sin(IVP_FLOAT angle);  // fifth order approximation
    static inline IVP_FLOAT approx5_cos(IVP_FLOAT angle);  // fifth order approximation of cos

    
    static inline IVP_FLOAT save_acosf(IVP_FLOAT x);	// slow and save acos

    static inline IVP_FLOAT fast_asin(IVP_FLOAT x);          // for documentation see ivu_linear_macros.hxx
    static inline IVP_FLOAT upper_limit_asin(IVP_FLOAT angle);   // for documentation see ivu_linear_macros.hxx

    static inline IVP_FLOAT fast_approx_asin(IVP_FLOAT angle);   //   "
    static inline IVP_FLOAT fast_anywhere_asin(IVP_FLOAT angle); //   "

    static IVP_FLOAT isqrt_float(IVP_FLOAT quad); //   "
    static IVP_DOUBLE isqrt_double(IVP_DOUBLE quad); //   "

    static inline int int_div_2(int a); //workaround for compiler bug in CodeWarrior1.6

#if defined(IVP_NO_DOUBLE) && !defined(SUN)
    static IVP_DOUBLE fabsd(IVP_DOUBLE f){ return fabsf(f); };
#ifdef PSXII
    static IVP_DOUBLE ivp_sqrtf(IVP_DOUBLE x){ 
		__asm__ __volatile__ (" \
		.set noreorder \
			sqrt.s	%0, %0 \
		.set reorder \
		" : "+f" (x) :); \
		return x; \
	}
#else	    
    static IVP_DOUBLE ivp_sqrtf(IVP_DOUBLE f){ return sqrtf(f); };
#endif
    static IVP_DOUBLE sqrtd(IVP_DOUBLE f){ return ivp_sqrtf(f); };
    static IVP_DOUBLE ivp_sinf(IVP_DOUBLE f) { return sinf(f); };
    static IVP_DOUBLE ivp_cosf(IVP_DOUBLE f) { return cosf(f); };
    static IVP_DOUBLE sind(IVP_DOUBLE f) { return sinf(f); };
    static IVP_DOUBLE cosd(IVP_DOUBLE f) { return cosf(f); };
    static IVP_DOUBLE acosd(IVP_DOUBLE f){ return acosf(f); };
    static IVP_DOUBLE asind(IVP_DOUBLE f){ return asinf(f); };
    static IVP_DOUBLE atand(IVP_DOUBLE f){ return atanf(f); };
    static IVP_DOUBLE ivp_expf(IVP_DOUBLE f) { return expf(f); };
    static IVP_DOUBLE atan2d(IVP_DOUBLE f1, IVP_DOUBLE f2){ return atan2f(f1,f2); };
#else
    static IVP_DOUBLE fabsd(IVP_DOUBLE f){ return fabs(f); };
    static IVP_DOUBLE ivp_sqrtf(IVP_DOUBLE f){ return sqrt(f); };
    static IVP_DOUBLE sqrtd(IVP_DOUBLE f){ return sqrt(f); };
    static IVP_DOUBLE ivp_sinf(IVP_DOUBLE f) { return sin(f); };
    static IVP_DOUBLE ivp_cosf(IVP_DOUBLE f) { return cos(f); };
    static IVP_DOUBLE sind(IVP_DOUBLE f) { return sin(f); };
    static IVP_DOUBLE cosd(IVP_DOUBLE f) { return cos(f); };
    static IVP_DOUBLE acosd(IVP_DOUBLE f){ return acos(f); };
    static IVP_DOUBLE asind(IVP_DOUBLE f){ return asin(f); };
    static IVP_DOUBLE atand(IVP_DOUBLE f){ return atan(f); };
    static IVP_DOUBLE ivp_expf(IVP_DOUBLE f) { return exp(f); };
    static IVP_DOUBLE atan2d(IVP_DOUBLE f1, IVP_DOUBLE f2){ return atan2(f1,f2); };
#endif
};


/********************************************************************************
 *	Name:	       	IVP_U_Hesse
 *	Description:	A convenient way to define a plane,
 *			and e.g. for calculating the distance from a point to that
 *			plane.
 ********************************************************************************/
class IVP_U_Hesse: public IVP_U_Point {
public:
#if !defined(IVP_VECTOR_UNIT_DOUBLE)
    IVP_DOUBLE hesse_val;
#endif
    void calc_hesse(const IVP_U_Point *p0,const IVP_U_Point *p1,const IVP_U_Point *p2);
    void calc_hesse(const IVP_U_Float_Point *p0, const IVP_U_Float_Point *p1, const IVP_U_Float_Point *p2);
    void calc_hesse_val(const IVP_U_Point *p0);
    void proj_on_plane(const IVP_U_Point *p, IVP_U_Point *result) const;	// sets result to a point on the plane and nearest to p
    void mult_hesse(IVP_DOUBLE factor);						// scale the hesse
    void normize();								// normize hesse (->super.calc_quad_length() == 1.0f)
#if !defined(IVP_NO_DOUBLE)    
    inline IVP_DOUBLE get_dist(const IVP_U_Point *p) const;				// get the distance between a point and the plane
#endif
    inline IVP_DOUBLE get_dist(const IVP_U_Float_Point *p) const;			// get the distance between a point and the plane
    IVP_RETURN_TYPE calc_intersect_with(const IVP_U_Straight *straight,		// calc the intersection between a straight and the plane
					IVP_U_Point *point_out) const; 

	void byte_swap() { ivp_byte_swap4( (uint&) hesse_val ); IVP_U_Point::byte_swap(); }
    
};


/********************************************************************************
 *	Name:	       	IVP_U_Float_Hesse
 *	Description:	A convenient way to define a plane,
 *			and e.g. for calculating the distance from a point to that
 *			plane. (Float version)
 ********************************************************************************/
class IVP_U_Float_Hesse: public IVP_U_Float_Point {
public:
#if !defined(IVP_VECTOR_UNIT_FLOAT)
    IVP_FLOAT hesse_val;
#endif
    void set4(const IVP_U_Float_Hesse *h){ this->set(h); this->hesse_val = h->hesse_val;};
    void calc_hesse(const IVP_U_Float_Point *p0,const IVP_U_Float_Point *p1,const IVP_U_Float_Point *p2);
    void calc_hesse_val(const IVP_U_Float_Point *p0);
    void proj_on_plane(const IVP_U_Float_Point *p, IVP_U_Float_Point *result) const;	// sets result to a point on the plane and nearest to p
    void mult_hesse(IVP_DOUBLE factor);						// scale the hesse
    void normize();								// normize hesse (->super.calc_quad_length() == 1.0f)
    // get the distance between a point and the plane
    inline IVP_DOUBLE get_dist(const IVP_U_Float_Point *p) const;				// get the distance between a point and the plane

    IVP_U_Float_Hesse(){;};
    IVP_U_Float_Hesse(IVP_DOUBLE xi, IVP_DOUBLE yi, IVP_DOUBLE zi, IVP_DOUBLE val) { k[0]=(IVP_FLOAT)xi; k[1]=(IVP_FLOAT)yi; k[2]=(IVP_FLOAT)zi; hesse_val=val; }

	void byte_swap() { ivp_byte_swap4( (uint&) hesse_val ); IVP_U_Float_Point::byte_swap(); }
};

typedef IVP_U_Float_Hesse IVP_U_Float_Point4;

/********************************************************************************
 *	Name:	    	IVP_U_Matrix3
 *	Description:	A rotation matrix in a three dimensional space
 *	Attention:	syntax is different to syntax of IVP_U_Point.
 *			All transformations using inverted matrices assume that
 *			matrices are orthonormized !!!.
 *			To invert any matrix, use real_invert
 ********************************************************************************/
class IVP_U_Matrix3 {
protected:
    IVP_U_Point rows[3];    // use get_elem and set_elem to access elements
public:
    inline IVP_DOUBLE get_elem(int row, int col) const { return rows[row].k[col];};
    inline void set_elem(int row, int col, IVP_DOUBLE val){ rows[row].k[col] = val; };
    
    inline void inline_mmult3(const IVP_U_Matrix3 *mb, IVP_U_Matrix3 *m_out )const; 
    inline void inline_mimult3(const IVP_U_Matrix3 *mb, IVP_U_Matrix3 *m_out )const;
    
    void mmult3(const IVP_U_Matrix3 *mb, IVP_U_Matrix3 *m_out )const; 	// m_out = this * mb, note: note: in place operation allowed
    void mimult3(const IVP_U_Matrix3 *mb, IVP_U_Matrix3 *m_out )const; 	// m_out = (this^-1) * mb, note: in place operation allowed
    void mi2mult3(const IVP_U_Matrix3 *mb, IVP_U_Matrix3 *m_out )const; // m_out = this * (mb^-1), note: in place operation allowed

#if !defined(IVP_NO_DOUBLE)
    inline void inline_vmult3(const IVP_U_Point *p_in, IVP_U_Point * p_out ) const;  	           // p_out = this * p_in
    	   void vmult3 (const IVP_U_Point *p_in, IVP_U_Point * p_out ) const;  	                // p_out = this * p_in
#endif

    inline void inline_vmult3 (const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out ) const;   // p_out = this * p_in
           void vmult3 (const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out ) const; 	// p_out = this * p_in 

	   

#if !defined(IVP_NO_DOUBLE)
    inline void inline_vimult3(const IVP_U_Point *p_in, IVP_U_Point * p_out ) const; 	                // p_out = (this^-1) * p_in 
    	   void vimult3(const IVP_U_Point *p_in, IVP_U_Point * p_out ) const; 	                // p_out = (this^-1) * p_in 
#endif    

    inline void inline_vimult3( const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out ) const;   // assembler version
           void vimult3(const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out ) const; 	// p_out = (this^-1) * p_in


    void orthogonize();             // all vectors are in a right hand system, keeps z vector
    IVP_RETURN_TYPE normize();      // all column vectors get length 1.0
    IVP_RETURN_TYPE orthonormize(); // both of above
    
    IVP_DOUBLE quad_rot_distance_to(const IVP_U_Matrix3 *m2); // sum of quad diff of mm
    
    /********************************************************************************
     *	Name:	       	calc_eigen_vector
     *	Description:	returns the degree of freedom of the eigenvector
     ********************************************************************************/
    int calc_eigen_vector(IVP_DOUBLE eigen_value, IVP_U_Point *eigen_vector_out);
    
    void init_columns3(const IVP_U_Point *v0, const IVP_U_Point *v1, const IVP_U_Point *v2);    // sets this columns to v0,v1,v2, 
    void init_rows3(const IVP_U_Point *v0, const IVP_U_Point *v1, const IVP_U_Point *v2);	// sets this rows    to v0,v1,v2

    
    /********************************************************************************
     *	Name:	       	init_normized3
     *	Description:	creates a normized matrix
     *			the user can predefine one given row/col
     ********************************************************************************/
    void init_normized3_col(const IVP_U_Point *v, IVP_COORDINATE_INDEX coordinate_index);      // init orthornomized matrix given any axis
    void init_normized3_col(const IVP_U_Point *vb, IVP_COORDINATE_INDEX coordinate_index, const IVP_U_Point *vc);      // init orthornomized matrix given any two axes
    void init_normized3_row(const IVP_U_Point *v, IVP_COORDINATE_INDEX coordinate_index);      // init orthornomized matrix given any axis
    void init_normized3_row(const IVP_U_Point *vb, IVP_COORDINATE_INDEX coordinate_index, const IVP_U_Point *vc);      // init orthornomized matrix given any two axes

    void init_rotated3(IVP_COORDINATE_INDEX axis, IVP_FLOAT angle);  // init this matrix as a rotation around the given axis
    
    inline void get_row(IVP_COORDINATE_INDEX row, IVP_U_Point *row_out) const;
    inline void get_row(IVP_COORDINATE_INDEX row, IVP_U_Float_Point *row_out) const;
    inline void get_col(IVP_COORDINATE_INDEX column, IVP_U_Point *col_out) const;
    inline void get_col(IVP_COORDINATE_INDEX column, IVP_U_Float_Point *col_out) const;

    inline void set_row(IVP_COORDINATE_INDEX row, const IVP_U_Point *);
    inline void set_row(IVP_COORDINATE_INDEX row, const IVP_U_Float_Point *);
    inline void set_col(IVP_COORDINATE_INDEX column, const IVP_U_Point *);
    inline void set_col(IVP_COORDINATE_INDEX column, const IVP_U_Float_Point *);

    IVP_DOUBLE get_determinante() const;
    IVP_RETURN_TYPE real_invert(IVP_DOUBLE epsilon = P_DOUBLE_EPS); 			// Using determinant algorithm

    void set_transpose3(const IVP_U_Matrix3 *m);
    void transpose3();
    void init3();	// sets this to the 'I' matrix

    // INTERN_START
    IVP_RETURN_TYPE get_angles(IVP_FLOAT *alpha, IVP_FLOAT *beta, IVP_FLOAT *gamma);  	// get the euler angles (real slow implementation)
    // INTERN_END

	void byte_swap() { rows[0].byte_swap(); rows[1].byte_swap(); rows[2].byte_swap(); }

};

/********************************************************************************
 *	Name:	       	IVP_U_Matrix
 *	Description:	3D transformation matrix: rotation (mm) and shift (vv)
 ********************************************************************************/
class IVP_U_Matrix: public IVP_U_Matrix3
{
public:    
    IVP_U_Point vv;	// Translations Vector, is added in vmult after rotation.

    IVP_U_Point *get_position() { return &vv; };				// get the position of an object matrix
    const IVP_U_Point *get_position() const { return &vv; };				// get the position of an object matrix

    inline void inline_mmult4(const IVP_U_Matrix *mb, IVP_U_Matrix *m_out ) const; // matrix multiplication
    inline void inline_mimult4(const IVP_U_Matrix *mb, IVP_U_Matrix *m_out ) const; // this is transposed !!
    void mmult4(const IVP_U_Matrix *mb, IVP_U_Matrix *m_out ) const;	//m_out = this*mb
    void mimult4(const IVP_U_Matrix *mb, IVP_U_Matrix *m_out ) const; 	//m_out = (this^-1) * mb
    
    void mi2mult4(const IVP_U_Matrix *mb, IVP_U_Matrix *m_out )const; //m_out = this * (mb^-1)
    void rotate(IVP_COORDINATE_INDEX axis, IVP_FLOAT angle, IVP_U_Matrix *m_out);  // m_out = this * I.init_rotated()
    void rotate_invers(IVP_COORDINATE_INDEX axis, IVP_FLOAT angle, IVP_U_Matrix *m_out);
    
#if !defined(IVP_NO_DOUBLE)
    inline void inline_vmult4 ( const IVP_U_Point *p_in, IVP_U_Point * p_out) const;	// p_out = this * p_in
    inline void inline_vmult4 ( const IVP_U_Float_Point *p_in, IVP_U_Point * p_out) const;	// p_out = this * p_in
    void vmult4 ( const IVP_U_Point *p_in, IVP_U_Point * p_out) const;			// p_out = this * p_in
    void vmult4 ( const IVP_U_Float_Point *p_in, IVP_U_Point * p_out) const;			// p_out = this * p_in
#endif
    inline void inline_vmult4 ( const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out) const;	// p_out = this * p_in (e.g. object space to core space)
    void vmult4 ( const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out) const;			// p_out = this * p_in


#if !defined(IVP_NO_DOUBLE)
    inline void inline_vimult4( const IVP_U_Point *p_in, IVP_U_Point * p_out ) const;			// p_out = (this^-1) * p_in
    inline void inline_vimult4( const IVP_U_Point *p_in, IVP_U_Float_Point * p_out ) const;	// p_out = (this^-1) * p_in
    void vimult4( const IVP_U_Point *p_in, IVP_U_Float_Point * p_out ) const;			// p_out = (this^-1) * p_in			   
    void vimult4( const IVP_U_Point *p_in, IVP_U_Point * p_out ) const;			// p_out = (this^-1) * p_in			   
#endif
    inline void inline_vimult4( const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out ) const;	// p_out = (this^-1) * p_in    
    void vimult4( const IVP_U_Float_Point *p_in, IVP_U_Float_Point * p_out ) const;			// p_out = (this^-1) * p_in			   

    void transpose();
    void set_transpose(const IVP_U_Matrix *in);
	inline void get_4x4_column_major( IVP_FLOAT *out ) const;

    IVP_RETURN_TYPE real_invert(IVP_DOUBLE epsilon = P_DOUBLE_EPS); 			// Using determinant algorithm
    IVP_RETURN_TYPE real_invert(const IVP_U_Matrix *m, IVP_DOUBLE epsilon = P_DOUBLE_EPS); // Overload

    // INTERN_START
         /*************************************************************
          * Shifts an object matrix in world space.
          *************************************************************/
    void shift_ws( const IVP_U_Point *v_in );
    
         /*************************************************************
          * Shifts an object matrix in it's object space.
          *************************************************************/
    void shift_os( const IVP_U_Point *v_in );
    //INTERN_END
  
    
    void init();	// sets this to 'I'

    void init_rot_multiple(const  IVP_U_Point *angles, IVP_DOUBLE factor = 1.0f);	// sets this rotated (using euler angles) and position to 0,0,0

    void init_columns4(const IVP_U_Point *v0, const IVP_U_Point *v1, const IVP_U_Point *v2,const IVP_U_Point *shift);
    void init_rows4(const IVP_U_Point *v0, const IVP_U_Point *v1, const IVP_U_Point *v2,const IVP_U_Point *shift);
    void interpolate(const IVP_U_Matrix *m1, const IVP_U_Matrix *m2,    IVP_DOUBLE i_factor);                // this = m1 * (1.0f-factor) + m2 * factor

    IVP_DOUBLE quad_distance_to(const IVP_U_Matrix *m2);
    
    void set_matrix(const IVP_U_Matrix *mat) { *this=*mat; }
    
    void print(const char *headline = 0);
    IVP_ERROR_STRING write_to_file(FILE *fp,const char *key = 0);
    IVP_ERROR_STRING read_from_file(FILE *fp);

	void byte_swap() { vv.byte_swap(); IVP_U_Matrix3::byte_swap(); }
};



/********************************************************************************
 *	Name:	       	IVP_U_Quat
 *	Description:	Efficient treatment of rotation matrizes.	
 ********************************************************************************/
class IVP_U_Quat {
public:
    IVP_DOUBLE x,y,z,w;
    
    void set(IVP_DOUBLE rot_x, IVP_DOUBLE rot_y, IVP_DOUBLE rot_z);  // set rotation, Note: rot_ < IVP_PI_2
    void set_fast_multiple(const  IVP_U_Point *angles, IVP_DOUBLE factor); // sets x = sin( angles.k[0] * factor * 0.5f), y = ..   , w= 1 - sqrt(xx+yy+zz), lets the engine crash for angles > PI
    void set_fast_multiple_with_clip(const  IVP_U_Float_Point *angles, IVP_DOUBLE factor); // same as set_fast_multiple, but do not crash
    void set_very_fast_multiple(const  IVP_U_Float_Point *angles, IVP_DOUBLE factor);
    inline void init();
  void set_matrix(IVP_U_Matrix3 *mat3_out)const ;
    void set_matrix(IVP_DOUBLE m_out[4][4])const;
    void set_quaternion(const IVP_U_Matrix3 *mat3_in);
    void set_quaternion(const IVP_DOUBLE m_in[4][4]);
    void set_interpolate_smoothly(const IVP_U_Quat * from,const IVP_U_Quat * to, IVP_DOUBLE t);
    void set_interpolate_linear(const IVP_U_Quat * from,const  IVP_U_Quat * to, IVP_DOUBLE t);
    void normize_quat();
    void fast_normize_quat();           // works only on quads with |quad| = 1.0f +- 0.1f; very fast if quat is already nearly normized
  inline void normize_correct_step(int steps);        // increase precission of normized quad
    void invert_quat();
    void get_angles(IVP_U_Float_Point *angles_out);
    void set_from_rotation_vectors(IVP_DOUBLE x1,IVP_DOUBLE y1, IVP_DOUBLE z1, IVP_DOUBLE x2, IVP_DOUBLE y2, IVP_DOUBLE z2);
    inline void inline_set_mult_quat(const IVP_U_Quat* q1, const IVP_U_Quat* q2);
    inline void inline_set_mult_quat(const IVP_U_Quat* q1, const IVP_U_Float_Quat* q2);
    void set_mult_quat(const IVP_U_Quat* q1, const IVP_U_Quat* q2);
    inline IVP_DOUBLE acos_quat(const IVP_U_Quat* q1) const; // acosinus between two quats
    void set_div_unit_quat(const IVP_U_Quat* q1, const IVP_U_Quat* q2); // res = q1/q2; set div, if both quats are unit quats
    void set_invert_mult(const IVP_U_Quat* q1,const  IVP_U_Quat* q2); // res = 1.0f/q1 * q2
    void set_invert_unit_quat(const IVP_U_Quat *q1);  // invert unit quaternion

    inline IVP_DOUBLE inline_estimate_q_diff_to(const IVP_U_Float_Quat *reference) const;	// roughly estimate the quad alpha

    IVP_U_Quat(){};  // not initialized quat
    IVP_U_Quat(const IVP_U_Point &p){ this->set_fast_multiple(&p,1.0f); }; // init by a rotation
    IVP_U_Quat( const IVP_U_Matrix3 *m) { this->set_quaternion(m); }
    // INTERN_START
    //log_difference ?nicht kapiert
    //euler_to_quat
    //get_axis_and_angle
    //set_from_axis_and_angle
    //scale
    //Sqare
    //Sqrt
    //dot_product
    //length
    //negate
    //exponent
    //log
    // INTERN_END

	inline void byte_swap() { 
	#if defined(IVP_NO_DOUBLE)
		ivp_byte_swap4( (uint&) x);
		ivp_byte_swap4( (uint&) y);
		ivp_byte_swap4( (uint&) z);
		ivp_byte_swap4( (uint&) w);
	#else
		IVP_ASSERT( 0 && "No byte swap for double yet");
	#endif
	}


} IVP_ALIGN_16;



#if defined(IVP_NO_DOUBLE)
class IVP_U_Float_Quat: public IVP_U_Quat {
    public:	

#else

class IVP_U_Float_Quat {
public:
    IVP_FLOAT x,y,z,w;
#endif
    inline void set(const IVP_U_Quat *source);

	inline void byte_swap() { 
		ivp_byte_swap4( (uint&) x);
		ivp_byte_swap4( (uint&) y);
		ivp_byte_swap4( (uint&) z);
		ivp_byte_swap4( (uint&) w);
		}
};

#ifdef IVP_WINDOWS_ALIGN16
#pragma pack(pop)
#endif

#endif
