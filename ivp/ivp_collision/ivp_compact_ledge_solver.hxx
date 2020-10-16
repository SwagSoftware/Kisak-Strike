// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifdef SUN4
#	define IVP_STATIC_INLINE inline
#	define IVP_STATIC
#else
#	define IVP_STATIC_INLINE static inline
#	define IVP_STATIC	static
#endif

#define IVP_MAGIC_INERTIA_RADIUS 100.0f  // for ill calculated inertias use this default radius

#ifndef _IVP_CACHE_OBJECT_INCLUDED
#	include <ivp_cache_object.hxx>
#endif


#ifndef _IVP_CACHE_LEDGE_POINT_INCLUDED
#	include <ivp_cache_ledge_point.hxx>
#endif

class IVP_Cache_Ball;

class IVP_Unscaled_QR_Result {
public:
  IVP_FLOAT checks[3];  // check[0] >0 -> inside this
  IVP_FLOAT scale;
  IVP_BOOL is_outside(){
    unsigned int a = *((int *)&checks[0]);
    unsigned int b = *((int *)&checks[1]);
    unsigned int c = *((int *)&checks[2]);
    unsigned int sign_bit = a | b | c;
    return IVP_BOOL(sign_bit>>31);
  }
};

class IVP_Unscaled_S_Result {
public:
   IVP_FLOAT checks[2];
  IVP_BOOL is_outside(){
    unsigned int a = *((int *)&checks[0]);
    unsigned int b = *((int *)&checks[1]);
    unsigned int sign_bit = a | b;
    return IVP_BOOL(sign_bit>>31);
  }
};

class IVP_Unscaled_KK_Result {
public:
  IVP_FLOAT checks_K[2];   // >0 means L projected onto K, K->next is on K->next side of K
  IVP_FLOAT checks_L[2];

  IVP_BOOL is_outside_K(){
    unsigned int a = *((int *)&checks_K[0]);
    unsigned int b = *((int *)&checks_K[1]);
    unsigned int sign_bit = a | b;
    return IVP_BOOL(sign_bit>>31);
  }

  IVP_BOOL is_outside_L(){
    unsigned int a = *((int *)&checks_L[0]);
    unsigned int b = *((int *)&checks_L[1]);
    unsigned int sign_bit = a | b;
    return IVP_BOOL(sign_bit>>31);
  }
};

class IVP_KK_Input {
public:
  const IVP_U_Float_Point *L_Los[2];

  IVP_U_Point K_Los[2];
  
  IVP_U_Point Kvec_Los;
  IVP_U_Point Lvec_Los;
  
  const IVP_Compact_Edge *K;
  const IVP_Compact_Edge *L;
  
  IVP_Cache_Ledge_Point *cache_K;
  IVP_Cache_Ledge_Point *cache_L;

  IVP_U_Point cross_KL_Los;  // crossproduct of K vec and L vec
  
  IVP_KK_Input( const IVP_Compact_Edge *K, const IVP_Compact_Edge *L, IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L);
  IVP_DOUBLE calc_quad_distance_edge_edge();
};

class IVP_Compact_Ledge_Solver {
    int dummy;		// there are no zero length classes in some C++ compilers
public:
#ifdef DEBUG
    IVP_STATIC IVP_BOOL check_ledge(const IVP_Compact_Ledge *c_ledge); // for debugging
#endif
    /********************************************************************************
     *	Name:	       	calc s_vals
     *	Description:	find the point on an edge with the shortest distance to a given other point
     *			and return the position of that point on the edge as a relative
     *			value between 0 and 1:   0 means projected point is identical to first point of edge
     *						1 means projected point is identical to second point od the edge
     ********************************************************************************/
    IVP_STATIC_INLINE IVP_DOUBLE calc_dot_product_vec_dot_connection(const IVP_U_Point *vector, const IVP_U_Float_Point *s1, const IVP_U_Float_Point *e1);
    IVP_STATIC IVP_DOUBLE calc_s_val(const IVP_Compact_Edge *edge, const IVP_U_Point *p_world, IVP_Cache_Ledge_Point *m_cache_edge); // calcs intersect pos
    IVP_STATIC void calc_unscaled_s_val_K_space(const IVP_Compact_Ledge *c_ledge, const IVP_Compact_Edge *edge, const IVP_U_Point *P_Kos, IVP_Unscaled_S_Result *result);

    
    /********************************************************************************
     *	Name:	       	calc qr_vals
     *	Description:	find the point on an area with the shortest distance to a given other point
     *			and return the position of that point on the edge as a relative
     *			values between 0 and 1:
     * q, r refer to 
     *    q=0, r=0 -> projected_point (pp) ==  this->next
     *    q=1, r=0 -> pp == this
     *    q=0, r=1 -> pp == this->prev
     *	  q+r=1    -> pp == [this,this->prev]
     ********************************************************************************/
    IVP_STATIC void calc_qr_vals(const IVP_Compact_Edge *e_tri,const  IVP_Compact_Edge *p,
				 IVP_DOUBLE *out_q, IVP_DOUBLE *out_r,
				 IVP_Cache_Ledge_Point *m_cache_e_tri, IVP_Cache_Ledge_Point *m_cache_p);
    
    IVP_STATIC void calc_qr_vals(const IVP_Compact_Edge *e_tri,const  IVP_U_Point *p_world,
				 IVP_DOUBLE *out_q, IVP_DOUBLE *out_r,
				 IVP_Cache_Ledge_Point *m_cache_e_tri);

    IVP_STATIC void calc_unscaled_qr_vals_F_space(const IVP_Compact_Ledge *c_ledge, const IVP_Compact_Edge *tri, const IVP_U_Point *object_point,
						  IVP_Unscaled_QR_Result *result);
    
  
  /********************************************************************************
   *	Name:	       	calc calc_unscaled_KK_vals
   *	Description:	find the points on two edges with the shortest distance to each other
   *			and return the position of that points on the edges as a relative
   *			value between 0 and 1:   0 means projected point is identical to first point of edge
   *						 1 means projected point is identical to second point od the edge
   ********************************************************************************/
  IVP_STATIC IVP_RETURN_TYPE calc_unscaled_KK_vals(const IVP_KK_Input &in, IVP_Unscaled_KK_Result *result);
  

    /********************************************************************************
     *	Name:	       	hesse vectors of an area
     ********************************************************************************/
    IVP_STATIC void calc_hesse_normized_AT(const IVP_Compact_Edge *edge, IVP_Cache_Ledge_Point *cc, IVP_U_Hesse *hesse_out_ws);
    IVP_STATIC void calc_hesse_object(const IVP_Compact_Edge *edge, const IVP_Compact_Ledge *ledge, IVP_U_Hesse *hesse_out_os);
  

    IVP_STATIC void calc_hesse_vec_object_not_normized(const IVP_Compact_Edge *edge, const IVP_Compact_Ledge *ledge, IVP_U_Point *out_vec);
    IVP_STATIC void calc_hesse_vec_object_not_normized(const IVP_Compact_Edge *edge, const IVP_Compact_Ledge *ledge, IVP_U_Float_Point *out_vec);



  
    /********************************************************************************
     *	Name:	       	minimize_on_other_side
     *	Description:    Pierce through convex object and
     * 			traverse triangles until valid q/r values (or valid term.len?).
     *			Returned edge represents the triangle found.
     ********************************************************************************/
    IVP_STATIC const IVP_Compact_Edge *minimize_on_other_side(const IVP_Compact_Edge *edge, const IVP_U_Point *partner_this_os);

    
    /********************************************************************************
     *	Name:	       	various functions to get the coordinates of a point
     ********************************************************************************/
    IVP_STATIC  void give_world_coords_AT(const IVP_Compact_Edge *edge, IVP_Cache_Ledge_Point *clp, IVP_U_Point *p_ws_out);
    IVP_STATIC  inline  const IVP_Compact_Poly_Point *give_object_coords(const IVP_Compact_Edge *edge, IVP_Cache_Ledge_Point *clp);
    IVP_STATIC  inline  const IVP_Compact_Poly_Point *give_object_coords(const IVP_Compact_Edge *edge, const IVP_Compact_Ledge *ledge);

    IVP_STATIC   void calc_pos_other_space(const IVP_Compact_Edge *P,IVP_Cache_Ledge_Point *m_cache_P,    IVP_Cache_Ledge_Point *m_cache_other_space, IVP_U_Point *res);
    IVP_STATIC   void transform_vec_other_space(const IVP_U_Point *dir_os, IVP_Cache_Ledge_Point *m_cache_dir, IVP_Cache_Ledge_Point *m_cache_other_space, IVP_U_Point *res);
    IVP_STATIC   void transform_pos_other_space(const IVP_U_Float_Point *pos_os, IVP_Cache_Ledge_Point *m_cache_pos, IVP_Cache_Ledge_Point *m_cache_other_space, IVP_U_Point *res);
    
    /********************************************************************************
     *	Name:	       	various functions to get the distance between two primitives
     ********************************************************************************/
    IVP_STATIC IVP_DOUBLE quad_dist_edge_to_point_K_space(const IVP_Compact_Ledge *ledge_K, const IVP_Compact_Edge *K, const IVP_U_Point *object_pos);

    IVP_STATIC IVP_DOUBLE calc_qlen_PP_P_space(const IVP_Compact_Ledge *P_ledge, const IVP_Compact_Edge *P,const IVP_U_Point *P_Pos);
    IVP_STATIC IVP_DOUBLE calc_qlen_PK_K_space(const IVP_U_Point *P_in_K_space, const IVP_Compact_Ledge *K_ledge, const IVP_Compact_Edge *K );
    IVP_STATIC IVP_DOUBLE calc_qlen_KK(const IVP_Compact_Edge *K,const IVP_Compact_Edge *L, IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L);
    IVP_STATIC IVP_DOUBLE calc_qlen_PF_F_space(const IVP_Compact_Ledge *tri_ledge, const IVP_Compact_Triangle *tri,const IVP_U_Point *P_Fos);



    
    /********************************************************************************
     *	Names:	       	Convenience Routines
     ********************************************************************************/
    IVP_STATIC void get_all_ledges(const class IVP_Compact_Ledgetree_Node *node, IVP_U_BigVector <IVP_Compact_Ledge> *ledges_out);
    IVP_STATIC void get_all_ledges(const class IVP_Compact_Surface* surface, IVP_U_BigVector <IVP_Compact_Ledge>* ledges_out);
#ifdef HAVOK_MOPP
    IVP_STATIC void get_all_ledges(const class IVP_Compact_Mopp* mopp, IVP_U_BigVector <IVP_Compact_Ledge>* ledges_out);
#endif // HAVOK_MOPP
    
    /********************************************************************************
     *	Name:	       	Bounding box and radius calculation 
     ********************************************************************************/
private:
    
public:
    IVP_STATIC void calc_bounding_box( const IVP_Compact_Ledge *c_ledge_in,
				       IVP_U_Point *min_extents_out, IVP_U_Point *max_extents_out);

    IVP_STATIC void calc_radius_to_given_center( const IVP_Compact_Surface *c_surface_in, const IVP_U_Point *center_in,
						 IVP_DOUBLE *radius_out ,IVP_DOUBLE *radius_dev_out=0);
#ifdef HAVOK_MOPP
    IVP_STATIC void calc_radius_to_given_center( const class IVP_Compact_Mopp* c_mopp_in, const IVP_U_Point *center_in,
						 IVP_DOUBLE *radius_out ,IVP_DOUBLE *radius_dev_out=0);
#endif // HAVOK_MOPP
    IVP_STATIC void calc_radius_to_given_center( const IVP_Compact_Ledge *c_ledge, const IVP_U_Point *center_in,	// note does not init radius_out and radius_dev_out
						 IVP_DOUBLE *radius_out ,IVP_DOUBLE *radius_dev_out=0);

    IVP_STATIC void calc_geom_center_and_radius( const IVP_Compact_Ledge *c_ledge_in,
						 IVP_U_Point *geom_center_out, IVP_DOUBLE *geom_radius_out);
    IVP_STATIC void calc_mass_center_and_radius( const IVP_Compact_Ledge *c_ledge_in,
						 IVP_U_Point *mass_center_out, IVP_DOUBLE *mass_radius_out);

};

extern IVP_Compact_Ledge_Solver IVP_CLS;

const IVP_Compact_Poly_Point *IVP_Compact_Ledge_Solver::give_object_coords(const IVP_Compact_Edge *edge, const IVP_Compact_Ledge *ledge){
    IVP_ASSERT( ledge == edge->get_compact_ledge() );
  int point_index = edge->get_start_point_index();
  const IVP_Compact_Poly_Point *res = &ledge->get_point_array()[point_index];
  
  return res;
}


const IVP_Compact_Poly_Point *IVP_Compact_Ledge_Solver::give_object_coords(const IVP_Compact_Edge *edge, IVP_Cache_Ledge_Point *clp)
{
    IVP_ASSERT( clp->compact_ledge == edge->get_compact_ledge() );
    int point_index = edge->get_start_point_index();
    const IVP_Compact_Poly_Point *res = &clp->compact_poly_points[point_index];

	IVP_ASSERT ( (intptr_t(res) & 15) == 0);

    return res;
}




IVP_DOUBLE IVP_Compact_Ledge_Solver::calc_dot_product_vec_dot_connection(const IVP_U_Point *vector, const IVP_U_Float_Point *s0, const IVP_U_Float_Point *e0){
    IVP_DOUBLE ax = e0->k[0] - s0->k[0];
    IVP_DOUBLE ay = e0->k[1] - s0->k[1];
    IVP_DOUBLE az = e0->k[2] - s0->k[2];
    IVP_DOUBLE bx = ax * vector->k[0];
    IVP_DOUBLE by = ay * vector->k[1];
    IVP_DOUBLE bz = az * vector->k[2];
    return bx + by + bz;
}

