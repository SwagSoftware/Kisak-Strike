// GLOBE (C) 1995-97, oliver & oliver

#define P_EPS_PARALLEL 0.000000001f

enum IVP_U_INTERSECT_TYPE {
    IVP_U_INTERSECT_OK = 0,
    IVP_U_INTERSECT_NO_INTERSECTION = 1,
    IVP_U_INTERSECT_IDENTIC = 2,
    IVP_U_INTERSECT_PARALLEL = 3
};


class IVP_U_Straight
{
public:
    IVP_U_Point vec;	// vec is used NORMIZED ONLY!!
    IVP_U_Point start_point;	// a point on the straight

    inline IVP_U_Straight(){ ; }; // compiler demand
    IVP_U_Straight(const IVP_U_Point *i_start_point, const IVP_U_Point *i_vec);
    void set(const IVP_U_Point *i_start_point, const IVP_U_Point *i_vec);
    void set(const IVP_U_Float_Point *i_start_point, const IVP_U_Float_Point *i_vec);
    
    void calc_orthogonal_vec_from_point(const IVP_U_Point *point, IVP_U_Point *vec_out) const;
    IVP_U_INTERSECT_TYPE calc_intersect_with(const IVP_U_Straight *straight2,
		       IVP_U_Point *p_out, IVP_DOUBLE *dist_out); // ATT: 0-3 as result!
    IVP_DOUBLE get_quad_dist_to_point(IVP_U_Point *point) const;
};

class IVP_U_Plain: public IVP_U_Hesse	// don't mix up with P_Plane!
{
public:    
    IVP_U_Point start_point; // alternative representation: 2 vectors with startpoint
    IVP_U_Point vec1;
    IVP_U_Point vec2;
    
    IVP_U_Plain(const IVP_U_Hesse *hesse);
    IVP_U_Plain(const IVP_U_Point *p1,const IVP_U_Point *p2,const IVP_U_Point *p3);

    IVP_RETURN_TYPE calc_intersect_with(const IVP_U_Hesse *plane2, IVP_U_Straight *straight_out) const;    
};


