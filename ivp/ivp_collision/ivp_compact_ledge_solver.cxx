// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	File:	       	ivp_compact_ledge_solver.cxx	
 *	Description:	function collection
 ********************************************************************************/

#include <ivp_physics.hxx>
#include <ivu_float.hxx>
#include <ivu_hash.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>

#include <ivp_compact_ledge_solver.hxx>
#include <ivp_compact_surface.hxx>
#ifdef HAVOK_MOPP
#include <hk_mopp/ivp_compact_mopp.hxx>
#endif // HAVOK_MOPP
 
IVP_Compact_Ledge_Solver IVP_CLS;

///////////////////////////////////////////////////////////////////////////////////////


/********************************************************************************
 *	Names:	       	Bounding box and radius calculation 
 ********************************************************************************/

void IVP_Compact_Ledge_Solver::calc_bounding_box(    const IVP_Compact_Ledge *c_ledge_in,
						     IVP_U_Point *min_extents_out,
						     IVP_U_Point *max_extents_out)
{
    // calc object axis aligned bounding box
    
    // @@@ for point array might be shared, we walk through all points of all triangles!
    // @@@ this is trice as much as optimum...

    const IVP_Compact_Triangle *tri = c_ledge_in->get_first_triangle();
    IVP_U_Float_Point min_extents, max_extents; // to avoid many casts

    {	// set min max to any point
	const IVP_Compact_Edge *first_edge = tri->get_first_edge();
	const IVP_U_Float_Point *any_ledge_point = give_object_coords( first_edge, c_ledge_in);
	min_extents.set(any_ledge_point);
	max_extents.set(any_ledge_point);
    }

    int tri_cnt = c_ledge_in->get_n_triangles()-1;
    for( ; tri_cnt>=0; tri_cnt--,tri=tri->get_next_tri()){
	int edge_cnt;
	const IVP_Compact_Edge *edge;
	for( edge_cnt = 2, edge = tri->get_first_edge();
	     edge_cnt>=0;
	     edge_cnt--,edge=edge->get_next()){
	    const IVP_U_Float_Point *p = give_object_coords(edge,c_ledge_in);

	    if(p->k[0] < min_extents.k[0]){
		min_extents.k[0] = p->k[0];
	    }else if (p->k[0] > max_extents.k[0])
		max_extents.k[0] = p->k[0];
	    
	    if(p->k[1] < min_extents.k[1]){
		min_extents.k[1] = p->k[1];
	    }else if (p->k[1] > max_extents.k[1])
		max_extents.k[1] = p->k[1];

	    if(p->k[2] < min_extents.k[2]){
		min_extents.k[2] = p->k[2];
	    }else if (p->k[2] > max_extents.k[2])
		max_extents.k[2] = p->k[2];
	}
    }
    min_extents_out->set(&min_extents);
    max_extents_out->set(&max_extents);
}


void IVP_Compact_Ledge_Solver::get_all_ledges(const IVP_Compact_Ledgetree_Node *node, IVP_U_BigVector<IVP_Compact_Ledge> *ledges_out)
{
    if (node->is_terminal()) {
	const IVP_Compact_Ledge *ledge = node->get_compact_ledge();
	ledges_out->add((IVP_Compact_Ledge *)ledge );
    }
    else{
	const IVP_Compact_Ledgetree_Node *ls = node->left_son();
	const IVP_Compact_Ledgetree_Node *rs = node->right_son();
	get_all_ledges(ls,ledges_out);
	get_all_ledges(rs,ledges_out);
    }
}

/********************************************************************************
 *	Names:	       	calc_geom_center_and_radius
 *	Description:	calc the center based on the bounding box
 ********************************************************************************/

void IVP_Compact_Ledge_Solver::calc_geom_center_and_radius( const IVP_Compact_Ledge *c_ledge_in, IVP_U_Point *geom_center_out, IVP_DOUBLE *geom_radius_out)
{
    // simply calc bounding box and take center and radius from there
    
    IVP_U_Point min_extents, max_extents;
    IVP_CLS.calc_bounding_box(c_ledge_in, &min_extents, &max_extents);

    geom_center_out->set_interpolate(&min_extents, &max_extents, 0.5f);
    *geom_radius_out = 0.5f * IVP_Inline_Math::sqrtd(min_extents.quad_distance_to(&max_extents));
};


void IVP_Compact_Ledge_Solver::get_all_ledges( const IVP_Compact_Surface* surface, IVP_U_BigVector<IVP_Compact_Ledge> *all_ledges_out )
{
    get_all_ledges( surface->get_compact_ledge_tree_root(), all_ledges_out);
}



#ifdef HAVOK_MOPP
void IVP_Compact_Ledge_Solver::get_all_ledges( const IVP_Compact_Mopp* mopp, IVP_U_BigVector<IVP_Compact_Ledge>* all_ledges_out)
{
	char* ledge = (char*)mopp + mopp->offset_ledges + mopp->size_convex_hull;

	char* point0 = (char*)(((IVP_Compact_Ledge*)ledge)->get_point_array());

	do
	{
		IVP_ASSERT(((unsigned int)(ledge) & 0xf) == 0);

		all_ledges_out->add((IVP_Compact_Ledge*)ledge);

		ledge += ((IVP_Compact_Ledge*)ledge)->get_size();

//		ledge = (IVP_Compact_Ledge*)((char*)ledge + 48); // @@CB big hack !! only supports triangles !!
	} while(ledge != point0);

	return;
}
#endif // HAVOK_MOPP



void IVP_Compact_Ledge_Solver::calc_radius_to_given_center( const IVP_Compact_Ledge *ledge,
							    const IVP_U_Point *center_in,
							    IVP_DOUBLE *radius_out,
							    IVP_DOUBLE *radius_dev_out)
{    
    IVP_DOUBLE q_max = 0.0f;
    IVP_DOUBLE dev_max = 0.0f;

    const IVP_Compact_Triangle *tri;
    int t;
    for (t = ledge->get_n_triangles()-1, tri = ledge->get_first_triangle();
	 t >= 0 ;
	 t--, tri= tri->get_next_tri() ){
	
	const IVP_Compact_Edge *e = tri->get_first_edge();
	IVP_U_Point hesse_vec;
	IVP_CLS.calc_hesse_vec_object_not_normized(e, ledge, &hesse_vec);
	IVP_DOUBLE i_hv_len_squared = 1.0f / hesse_vec.quad_length();
	
	int j;
	for(j=0; j<3; j++, e++){
	    IVP_U_Point p;
	    const IVP_U_Float_Point *p_orig = IVP_CLS.give_object_coords(e, ledge);
	    p.subtract( p_orig , center_in );
	    {	// radius
		IVP_DOUBLE q = p.quad_length();
		if ( q > q_max) q_max = q;
	    }
	    {
		IVP_U_Point cp;
		cp.inline_calc_cross_product(&p, &hesse_vec);
		IVP_DOUBLE dev = cp.quad_length() * i_hv_len_squared;
		if (dev > dev_max){
		    dev_max = dev;
		}
	    }
	}
    }
	 q_max = IVP_Inline_Math::sqrtd(q_max);
    if (q_max > *radius_out) {
	*radius_out = q_max;
    }
    if (radius_dev_out){
	dev_max = IVP_Inline_Math::sqrtd(dev_max);
	if (dev_max > *radius_dev_out){
	    *radius_dev_out = dev_max;
	}
    }
}

#ifdef HAVOK_MOPP
void IVP_Compact_Ledge_Solver::calc_radius_to_given_center(const IVP_Compact_Mopp* c_mopp_in,
	const IVP_U_Point* center_in,
	IVP_DOUBLE* radius_out,
	IVP_DOUBLE* radius_dev_out)
{    
	*radius_out = 0;
	*radius_dev_out = 0;
	IVP_U_BigVector<IVP_Compact_Ledge> all_ledges(1024);
	get_all_ledges(c_mopp_in, &all_ledges );

	for (int l = all_ledges.len()-1; l>=0;l--)
	{
		const IVP_Compact_Ledge *ledge = all_ledges.element_at(l);
		calc_radius_to_given_center(ledge,center_in,radius_out,radius_dev_out);
	}
}
#endif // HAVOK_MOPP

void IVP_Compact_Ledge_Solver::calc_radius_to_given_center( const IVP_Compact_Surface *c_surface_in,
							    const IVP_U_Point *center_in,
							    IVP_DOUBLE *radius_out,
							    IVP_DOUBLE *radius_dev_out)
{    
    *radius_out = 0;
    *radius_dev_out = 0;
    IVP_U_BigVector<IVP_Compact_Ledge> all_ledges(1024);
    get_all_ledges(c_surface_in, &all_ledges );
    
    for (int l = all_ledges.len()-1; l>=0;l--){
	const IVP_Compact_Ledge *ledge = all_ledges.element_at(l);
	calc_radius_to_given_center(ledge,center_in,radius_out,radius_dev_out);
    }
}

void IVP_Compact_Ledge_Solver::calc_pos_other_space(const IVP_Compact_Edge *P,IVP_Cache_Ledge_Point *m_cache_P,
						       IVP_Cache_Ledge_Point *m_cache_other_space, IVP_U_Point *res){
    const IVP_U_Float_Point *p_os = IVP_CLS.give_object_coords( P, m_cache_P );
    IVP_U_Point p_ws;
    m_cache_P->get_object_cache()->m_world_f_object.inline_vmult4(p_os, &p_ws);
    m_cache_other_space->clp_cache_object->m_world_f_object.inline_vimult4(&p_ws, res);
}

void IVP_Compact_Ledge_Solver::transform_vec_other_space(const IVP_U_Point *dir_os, IVP_Cache_Ledge_Point *m_cache_dir,
						       IVP_Cache_Ledge_Point *m_cache_other_space, IVP_U_Point *res){
  IVP_U_Point dir_ws;
  m_cache_dir->clp_cache_object->m_world_f_object.inline_vmult3(dir_os, &dir_ws);
  m_cache_other_space->clp_cache_object->m_world_f_object.inline_vimult3(&dir_ws, res);
}

void IVP_Compact_Ledge_Solver::transform_pos_other_space(const IVP_U_Float_Point *pos_in_os, IVP_Cache_Ledge_Point *m_cache_dir,
						       IVP_Cache_Ledge_Point *m_cache_other_space, IVP_U_Point *res){
  IVP_U_Point dir_ws;
  IVP_U_Point pos_os(pos_in_os);
  m_cache_dir->clp_cache_object->m_world_f_object.inline_vmult4(&pos_os, &dir_ws);
  m_cache_other_space->clp_cache_object->m_world_f_object.inline_vimult4(&dir_ws, res);
}
#ifdef DEBUG
/********************************************************************************
 *	Names:	       	calc s_vals
 *	Description:	find the point on an edge with the shortest distance to a given other point
 *			and return the position of that point on the edge as a relative
 *			value between 0 and 1:   0 means projected point is identical to first point of edge
 *						1 means projected point is identical to second point od the edge
 ********************************************************************************/
IVP_DOUBLE IVP_Compact_Ledge_Solver::calc_s_val(const IVP_Compact_Edge *edge, const IVP_U_Point *p_world, IVP_Cache_Ledge_Point *m_cache_edge)
{
     // calcs intersect pos
     // von lot von p auf this (rel. zu this)
    IVP_U_Point tp;  give_world_coords_AT(edge, m_cache_edge, &tp);
    IVP_U_Point tp_next; give_world_coords_AT(edge->get_next(), m_cache_edge, &tp_next);
    
    IVP_U_Point vec1, vec2;
    vec1.subtract(&tp_next, &tp);
    vec2.subtract(p_world, &tp);
    IVP_DOUBLE i_qlen = 1.0f / vec1.quad_length();
    IVP_DOUBLE s = vec1.dot_product(&vec2);
    s *= i_qlen;
    return s;
}
#endif

void IVP_Compact_Ledge_Solver::calc_unscaled_s_val_K_space(const IVP_Compact_Ledge *c_ledge, const IVP_Compact_Edge *edge, const IVP_U_Point *p_object,IVP_Unscaled_S_Result *result)
{
     // calcs intersect pos
     // von lot von p auf this (rel. zu this)
 
    const IVP_U_Float_Point *p0 = give_object_coords(edge, c_ledge);
    const IVP_U_Float_Point *p1 = give_object_coords(edge->get_next(), c_ledge);
    IVP_U_Point d; d.subtract( p1, p0 );
    IVP_U_Point d0; d0.subtract( p0, p_object );
    IVP_U_Point d1; d1.subtract( p1, p_object );
    IVP_DOUBLE a = -d.dot_product( &d0 );
    IVP_DOUBLE b = d.dot_product( &d1 );
    result->checks[0] = a;
    result->checks[1] = b;
}



// calcs pos of lot von p auf
// plane spanned by -e_tri (=Q) and this->next (=R)

// q, r relate to:
//    origin = e_tri->next
//    Q = e_tri - e_tri->next (opposite to e_tri!)
//    R = e_tri->prev - e_tri->next
// for debugging only
#ifdef DEBUG
void IVP_Compact_Ledge_Solver::calc_qr_vals(const IVP_Compact_Edge *e_tri,const  IVP_U_Point *p_world,
					    IVP_DOUBLE *out_q, IVP_DOUBLE *out_r,
					    IVP_Cache_Ledge_Point *m_cache_e_tri)
{
    IVP_U_Point tp; IVP_CLS.give_world_coords_AT(e_tri, m_cache_e_tri, &tp);
    IVP_U_Point tp_next; IVP_CLS.give_world_coords_AT(e_tri->get_next(), m_cache_e_tri, &tp_next);
    IVP_U_Point tp_prev; IVP_CLS.give_world_coords_AT(e_tri->get_prev(), m_cache_e_tri, &tp_prev);
    
    IVP_U_Point R, Q, Pvec;
    Q.subtract(&tp, &tp_next); // other dir
    R.subtract(&tp_prev, &tp_next);
    
    IVP_DOUBLE QQ = Q.quad_length();
    IVP_DOUBLE RR = R.quad_length();
       
    IVP_DOUBLE QR = R.dot_product(&Q);

    IVP_DOUBLE QQRR = QQ * RR;
    IVP_DOUBLE QRQR = QR * QR;
    IVP_DOUBLE i_det = 1.0f / (QQRR - QRQR);

    Pvec.subtract( p_world, &tp_next);
    IVP_DOUBLE sq = Pvec.dot_product(&Q);
    IVP_DOUBLE sr = Pvec.dot_product(&R);
    
    *out_q = i_det * ( RR * sq - sr * QR );
    *out_r = i_det * ( QQ * sr - sq * QR );
}
#endif

#ifdef DEBUG
void IVP_Compact_Ledge_Solver::calc_qr_vals(const IVP_Compact_Edge *e_tri,const  IVP_Compact_Edge *p,
			     IVP_DOUBLE *out_q, IVP_DOUBLE *out_r,
			     IVP_Cache_Ledge_Point *m_cache_e_tri, IVP_Cache_Ledge_Point *m_cache_p)
{
    IVP_U_Point pp; IVP_CLS.give_world_coords_AT(p, m_cache_p, &pp);
    calc_qr_vals( e_tri, &pp, out_q, out_r, m_cache_e_tri);
}
#endif

/* rounding error of calc_unscaled_qr_vals_F_space:
   input:  p_object    error (P_MAX_WORLD_DOUBLE * P_DOUBLE_RES * ( 1 + x ) || inf)
           tp, tp_next, tp_prv 0 epsilon
 */

static char ivp_uqr_mod_table[] = { 0,2 * sizeof(IVP_FLOAT),1 * sizeof(IVP_FLOAT),0,2 * sizeof(IVP_FLOAT) };

void IVP_Compact_Ledge_Solver::calc_unscaled_qr_vals_F_space(const IVP_Compact_Ledge *c_ledge,
							     const IVP_Compact_Edge *tri, const IVP_U_Point *p_object,
							     IVP_Unscaled_QR_Result *result){
    // like calc_qr_vals(), all in F coord space
  const int this_edge_index = tri->get_edge_index();
  
  const IVP_Compact_Triangle *triangle = tri->get_triangle();
  
    const IVP_U_Float_Point *tp = give_object_coords( triangle->get_edge(0), c_ledge);
    const IVP_U_Float_Point *tp_next = give_object_coords(triangle->get_edge(1), c_ledge);
    const IVP_U_Float_Point *tp_prev = give_object_coords(triangle->get_edge(2), c_ledge);
    
    IVP_U_Point R, Q, Pvec;
    Q.subtract(tp, tp_next);      
    R.subtract(tp_prev, tp_next);

    IVP_DOUBLE QQ = Q.quad_length();
    IVP_DOUBLE RR = R.quad_length();
    
    Pvec.subtract( p_object, tp_next);
    
    IVP_DOUBLE QR = R.dot_product(&Q);

    IVP_DOUBLE QQRR = QQ * RR;
    IVP_DOUBLE QRQR = QR * QR;
    IVP_DOUBLE Det = (QQRR - QRQR);

    IVP_DOUBLE sq = Pvec.dot_product(&Q);
    IVP_DOUBLE sr = Pvec.dot_product(&R);
    
    IVP_DOUBLE q = ( RR * sq - sr * QR );
    IVP_DOUBLE r = ( QQ * sr - sq * QR );
    
    result->scale = Det;
    
    IVP_ASSERT ( sizeof( result->checks[0] == sizeof(IVP_FLOAT)));
    char *res = (char *) & result->checks[0];
    ((IVP_FLOAT *)( res + ivp_uqr_mod_table[this_edge_index] ))[0] = r;
    ((IVP_FLOAT *)( res + ivp_uqr_mod_table[this_edge_index+1] ))[0] = Det - q - r;
    ((IVP_FLOAT *)( res + ivp_uqr_mod_table[this_edge_index+2] ))[0] = q;
}


IVP_KK_Input::IVP_KK_Input( const IVP_Compact_Edge *K_in, const IVP_Compact_Edge *L_in, IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L){
  K = K_in;
  L = L_in;
  const IVP_U_Float_Point *K_os = IVP_CLS.give_object_coords(K, m_cache_K);
  const IVP_U_Float_Point *K_next_os = IVP_CLS.give_object_coords(K->get_next(), m_cache_K);
  IVP_U_Point Kvec_Kos;
  Kvec_Kos.subtract(K_next_os, K_os);
  IVP_CLS.transform_vec_other_space( &Kvec_Kos, m_cache_K, m_cache_L, & Kvec_Los);
  cache_K = m_cache_K;
  cache_L = m_cache_L;

  IVP_CLS.calc_pos_other_space( K, m_cache_K, m_cache_L, & K_Los[0]);
  IVP_CLS.calc_pos_other_space( K->get_next(), m_cache_K, m_cache_L, & K_Los[1]);
  
  const IVP_U_Float_Point *L_os = IVP_CLS.give_object_coords(L, m_cache_L);
  const IVP_U_Float_Point *L_next_os = IVP_CLS.give_object_coords(L->get_next(), m_cache_L);
  L_Los[0] = L_os;
  L_Los[1] = L_next_os;
  Lvec_Los.subtract(L_next_os, L_os);

  cross_KL_Los.calc_cross_product(&Kvec_Los, &Lvec_Los);
}

IVP_DOUBLE IVP_KK_Input::calc_quad_distance_edge_edge(){
  IVP_DOUBLE quad_dist = cross_KL_Los.quad_length();
  if(quad_dist > (P_DOUBLE_RES * P_DOUBLE_RES)){ // not parallel
    IVP_DOUBLE a = cross_KL_Los.dot_product(&K_Los[0]);
    IVP_DOUBLE b = cross_KL_Los.dot_product(L_Los[0]);
    return (a-b)* (a-b) / quad_dist;
  }
  return IVP_CLS.calc_qlen_PK_K_space( &K_Los[0], L->get_compact_ledge(), L);
}

IVP_RETURN_TYPE IVP_Compact_Ledge_Solver::calc_unscaled_KK_vals(const IVP_KK_Input &in, IVP_Unscaled_KK_Result *result){
  // returns IVP_FAULT when edges are parallel, in this case sk and sl are estimated
  // calcs common point of two straights K and L (as Tri_Edges)
  // and returns both intersect proportions
  const IVP_U_Point &norm = in.cross_KL_Los;
  IVP_DOUBLE quad_dist = norm.quad_length();

  if(quad_dist > (P_DOUBLE_RES * P_DOUBLE_RES * P_MAX_OBJECT_SIZE * P_MAX_OBJECT_SIZE)){ // not parallel
    IVP_U_Point K_area;	// area vertical to K and norm
    IVP_U_Point L_area;	// area vertical to L and norm
    K_area.calc_cross_product(&in.Kvec_Los, &norm);
    L_area.calc_cross_product(&in.Lvec_Los, &norm);

    IVP_DOUBLE a,b,x,ab;
    a = K_area.dot_product( in.L_Los[0] );
    b = K_area.dot_product( in.L_Los[1] );
    x = K_area.dot_product( &in.K_Los[0] );
    ab = a - b;
    result->checks_L[0] = (a-x) * ab;
    result->checks_L[1] = (x-b) * ab;

    a = L_area.dot_product( &in.K_Los[0] );
    b = L_area.dot_product( &in.K_Los[1] );
    x = L_area.dot_product( in.L_Los[0] );
    ab = a - b;
    result->checks_K[0] = (a-x) * ab;
    result->checks_K[1] = (x-b) * ab;

    return IVP_OK;
  }
    
  IVP_IF(1){
    printf("calc_unscaled_KK_vals: parallel edges\n");
  }
  // ok edges are nearly parallel, do very carefull distance checking !!!

  // check different positions at K
  IVP_DOUBLE min_qdist = P_DOUBLE_MAX;
# define IVP_KK_CHECK_0 0.001f
# define IVP_KK_CHECK_1 0.000001f
  IVP_FLOAT check_pos[] = { -1.0f,0.5f,2.0f,0.0f,1.0f,
			    0-IVP_KK_CHECK_0, 0+IVP_KK_CHECK_0, 1-IVP_KK_CHECK_0, 1+IVP_KK_CHECK_0,
			    0-IVP_KK_CHECK_1, 0+IVP_KK_CHECK_1, 1-IVP_KK_CHECK_1, 1+IVP_KK_CHECK_1
			    } ;

  // check K positions
  {
    const IVP_Compact_Ledge *ledge_L = in.cache_L->get_compact_ledge();
    for (int i=0;i<11;i++){
      IVP_FLOAT pos = check_pos[i];
      IVP_U_Point pK_Los; pK_Los.set_interpolate( &in.K_Los[0], & in.K_Los[1], pos);
      IVP_DOUBLE qdist = quad_dist_edge_to_point_K_space( ledge_L, in.L, &pK_Los);
      if (qdist < min_qdist){
	min_qdist = qdist;
	result->checks_K[0] = pos;
	result->checks_K[1] = 1.0f - pos;
	// check s_val;
	IVP_Unscaled_S_Result sr;
	calc_unscaled_s_val_K_space(ledge_L,in.L, &pK_Los, &sr );
	result->checks_L[0] = sr.checks[0];
	result->checks_L[1] = sr.checks[1];
      }
    }
  }
  {
    // check L positions
    IVP_U_Point L_Kos[2];
    transform_pos_other_space( in.L_Los[0], in.cache_L, in.cache_K, &L_Kos[0]);
    transform_pos_other_space( in.L_Los[1], in.cache_L, in.cache_K, &L_Kos[1]);
  
    const IVP_Compact_Ledge *ledge_K = in.cache_K->get_compact_ledge();
    for (int i=0;i<9;i++){
      IVP_FLOAT pos = check_pos[i];
      IVP_U_Point pL_Kos; pL_Kos.set_interpolate( &L_Kos[0], & L_Kos[1], pos);
      IVP_DOUBLE qdist = quad_dist_edge_to_point_K_space( ledge_K, in.K, &pL_Kos);
      if (qdist < min_qdist){
	min_qdist = qdist;
	result->checks_L[0] = pos;
	result->checks_L[1] = 1.0f - pos;
	// check s_val;
	IVP_Unscaled_S_Result sr;
	calc_unscaled_s_val_K_space(ledge_K,in.K, &pL_Kos, &sr );
	result->checks_K[0] = sr.checks[0];
	result->checks_K[1] = sr.checks[1];
      }
    }
  }
  return IVP_FAULT;
}

#ifdef DEBUG
// debugging only
void IVP_Compact_Ledge_Solver::calc_hesse_normized_AT(const IVP_Compact_Edge *edge,IVP_Cache_Ledge_Point *clp, IVP_U_Hesse *hesse_out_ws)
{
    // normized     
    IVP_U_Point p0; IVP_CLS.give_world_coords_AT(edge, clp, &p0);
    IVP_U_Point p1; IVP_CLS.give_world_coords_AT(edge->get_next(), clp, &p1);
    IVP_U_Point p2; IVP_CLS.give_world_coords_AT(edge->get_prev(), clp, &p2);
    hesse_out_ws->calc_hesse(&p0, &p2, &p1);
    hesse_out_ws->normize();
}
#endif

void IVP_Compact_Ledge_Solver::calc_hesse_object(const IVP_Compact_Edge *edge, const IVP_Compact_Ledge *ledge, IVP_U_Hesse *hesse_out_os){
    // normized in object coords
    const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(edge, ledge);
    const IVP_U_Float_Point *p1 = IVP_CLS.give_object_coords(edge->get_next(), ledge);
    const IVP_U_Float_Point *p2 = IVP_CLS.give_object_coords(edge->get_prev(), ledge);
    hesse_out_os->calc_hesse(p0, p2, p1);
    hesse_out_os->normize();
}

void IVP_Compact_Ledge_Solver::calc_hesse_vec_object_not_normized(const IVP_Compact_Edge *edge, const IVP_Compact_Ledge *ledge, IVP_U_Point *out_vec){
    const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(edge, ledge);
    const IVP_U_Float_Point *p1 = IVP_CLS.give_object_coords(edge->get_next(), ledge);
    const IVP_U_Float_Point *p2 = IVP_CLS.give_object_coords(edge->get_prev(), ledge);
    out_vec->inline_set_vert_to_area_defined_by_three_points(p0, p2, p1);
}

void IVP_Compact_Ledge_Solver::calc_hesse_vec_object_not_normized(const IVP_Compact_Edge *edge, const IVP_Compact_Ledge *ledge, IVP_U_Float_Point *out_vec){
    const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(edge, ledge);
    const IVP_U_Float_Point *p1 = IVP_CLS.give_object_coords(edge->get_next(), ledge);
    const IVP_U_Float_Point *p2 = IVP_CLS.give_object_coords(edge->get_prev(), ledge);
    out_vec->inline_set_vert_to_area_defined_by_three_points(p0, p2, p1);
}

IVP_DOUBLE IVP_Compact_Ledge_Solver::quad_dist_edge_to_point_K_space(const IVP_Compact_Ledge *ledge_K, const IVP_Compact_Edge *K, const IVP_U_Point *object_pos){
    const IVP_U_Float_Point *p0 = IVP_CLS.give_object_coords(K, ledge_K);
    const IVP_U_Float_Point *p1 = IVP_CLS.give_object_coords(K->get_next(), ledge_K);
    IVP_U_Point vec_world;
    vec_world.inline_set_vert_to_area_defined_by_three_points(p0, p1, object_pos );
    
    IVP_DOUBLE iqlen = 1.0f / p0->quad_distance_to(p1);
    return vec_world.quad_length() * iqlen;    
}

IVP_DOUBLE IVP_Compact_Ledge_Solver::calc_qlen_PF_F_space(const IVP_Compact_Ledge *ledge, const IVP_Compact_Triangle *tri,const  IVP_U_Point *object_pos)
{
  // former IVP_Triangle::quad_distance_to_core_pos()
    
  const IVP_Compact_Edge *e = tri->get_first_edge();
  class IVP_Unscaled_QR_Result qr;
  IVP_CLS.calc_unscaled_qr_vals_F_space(ledge, e, object_pos, &qr );
  
  // inside triangle
  if (! qr.is_outside()){
    IVP_U_Point normal;
    const IVP_U_Float_Point *start_point = give_object_coords(e,ledge);
    IVP_CLS.calc_hesse_vec_object_not_normized(e, ledge, &normal);
    IVP_DOUBLE inv_qlen = 1.0f/normal.quad_length();
    IVP_DOUBLE res = normal.dot_product(object_pos) - normal.dot_product(start_point);
    return res*res * inv_qlen;
  }
  IVP_DOUBLE qres = P_DOUBLE_MAX;
  for (int i=2;i>=0;i--){
    e = tri->get_edge(i);
    IVP_Unscaled_S_Result sr;
    IVP_CLS.calc_unscaled_s_val_K_space(ledge, e, object_pos, &sr);
    if (sr.checks[0]<0){
	const IVP_U_Float_Point *e_object = give_object_coords(e,ledge);
	IVP_DOUBLE qdist = object_pos->quad_distance_to(e_object);
	if (qdist < qres) qres = qdist;
      continue;
    }
    if (sr.checks[1] < 0) continue; // no need to check this case as this point wil be touched with the next edge
    IVP_DOUBLE qdist = IVP_CLS.quad_dist_edge_to_point_K_space(ledge, e, object_pos);
    if (qdist < qres) qres = qdist;
  }
  return qres;
};


IVP_DOUBLE IVP_Compact_Ledge_Solver::calc_qlen_PP_P_space(const IVP_Compact_Ledge *P_ledge, const IVP_Compact_Edge *P,const IVP_U_Point *P_Pos){
    const IVP_U_Float_Point *P_os = give_object_coords(P, P_ledge);
    return P_Pos->quad_distance_to(P_os);
}
  

IVP_DOUBLE  IVP_Compact_Ledge_Solver::calc_qlen_PK_K_space(const IVP_U_Point *P_in_K_space, const IVP_Compact_Ledge *K_ledge, const IVP_Compact_Edge *K ){
#if 0
    //TL: Playstation2 assert for alignment
    unsigned int adress=(unsigned int)P_in_K_space;
	unsigned int aligned_a=(adress & 0xfffffff0 );
	IVP_IF( aligned_a != adress ) {
		printf("erroradress %lx\n",adress);
		IVP_ASSERT(1==0);
	}
#endif
	
	IVP_Unscaled_S_Result sr;
    calc_unscaled_s_val_K_space(K_ledge, K,P_in_K_space, &sr);

    if ( !sr.is_outside()){
	return quad_dist_edge_to_point_K_space(K_ledge, K, P_in_K_space);	
    }

    // outside edge: take point instead
    if(sr.checks[0]<0.0f){
	return calc_qlen_PP_P_space( K_ledge, K, P_in_K_space);
    }
    return calc_qlen_PP_P_space( K_ledge, K->get_next(), P_in_K_space);
}

void IVP_Compact_Ledge_Solver::give_world_coords_AT(const IVP_Compact_Edge *edge, IVP_Cache_Ledge_Point *clp, IVP_U_Point *p_ws_out){
    IVP_ASSERT( clp->compact_ledge == edge->get_compact_ledge() );
    const IVP_U_Float_Point *p_os = give_object_coords(edge, clp );
    clp->get_object_cache()->transform_position_to_world_coords( p_os, p_ws_out);
}


IVP_DOUBLE IVP_Compact_Ledge_Solver::calc_qlen_KK(const IVP_Compact_Edge *K,const IVP_Compact_Edge *L, IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L)
{
    IVP_KK_Input kkin( K, L, m_cache_K, m_cache_L);
    IVP_Unscaled_KK_Result kkr;
    IVP_CLS.calc_unscaled_KK_vals(kkin, &kkr);
    
    if(!kkr.is_outside_L()){
	if ( kkr.checks_K[0] <= 0.0f){
	    return calc_qlen_PK_K_space( &kkin.K_Los[0], m_cache_L->get_compact_ledge(), L);
	}
	if ( kkr.checks_K[1] <= 0.0f){
	    return calc_qlen_PK_K_space( &kkin.K_Los[1], m_cache_L->get_compact_ledge(), L);
	}
	return kkin.calc_quad_distance_edge_edge();
    }
    if( !kkr.is_outside_K()){
	if ( kkr.checks_L[0] <= 0.0f){
	    IVP_U_Point L_Kos;
	    IVP_CLS.calc_pos_other_space( L, m_cache_L, m_cache_K, &L_Kos);
	    return calc_qlen_PK_K_space( &L_Kos, m_cache_K->get_compact_ledge(), K);
	}
	if ( kkr.checks_L[1] <= 0.0f){
	    IVP_U_Point L_Kos;
	    IVP_CLS.calc_pos_other_space( L->get_next(), m_cache_L, m_cache_K, &L_Kos);
	    return calc_qlen_PK_K_space( &L_Kos, m_cache_K->get_compact_ledge(), K);
	}
	CORE;
    }
    
    // we really are outside both edges.
    // now we have to figure out the minimal point point dist (in a secure way)

    const IVP_Compact_Edge *pK[2];
    const IVP_Compact_Edge *pL[2];
    pK[0] = K;
    pK[1] = K->get_next();
    pL[0] = L;
    pL[1] = L->get_next();
    
    IVP_DOUBLE min_len = P_DOUBLE_MAX;
    const IVP_Compact_Edge *min_Kp = 0;
    int i;
    for(i=0; i<=1; i++){
	IVP_U_Point L_Kos;
	IVP_CLS.calc_pos_other_space( pL[i], m_cache_L, m_cache_K, &L_Kos);
	IVP_DOUBLE h_len = calc_qlen_PK_K_space( &L_Kos, m_cache_K->get_compact_ledge(), K);
	if(h_len < min_len){ 
	    min_len = h_len;
	    min_Kp = pL[i];
	}
	IVP_DOUBLE h_len_2 = calc_qlen_PK_K_space( &kkin.K_Los[i], m_cache_L->get_compact_ledge(), L);
	if(h_len_2 < min_len){ 
	    min_len = h_len_2;
	    min_Kp = pK[i];
	}
    }
    IVP_ASSERT( min_Kp );
    return min_len;
}


#ifdef DEBUG
IVP_BOOL IVP_Compact_Ledge_Solver::check_ledge(const IVP_Compact_Ledge *cl)
{
    // For debugging only
    // Checks for consistency

	IVP_ASSERT ( (intptr_t(cl) & 15) == 0);
	const IVP_Compact_Poly_Point *ppppp=cl->get_point_array();
	IVP_ASSERT ( ((intptr_t)ppppp & 15 ) == 0);

    // all triangles 
    const IVP_Compact_Triangle *tri = cl->get_first_triangle();
    for(int i=0; i<cl->get_n_triangles(); i++){
	IVP_ASSERT(i == tri->get_tri_index());

		IVP_ASSERT ( (intptr_t(tri) & 15) == 0);

	// all edges
	for(int j=0; j<3; j++){
	    const IVP_Compact_Edge *edge = tri->get_edge(j);
	    const IVP_Compact_Edge *oppo = edge->get_opposite();
	    IVP_ASSERT(edge == oppo->get_opposite());
	    IVP_ASSERT(edge->get_start_point_index() == oppo->get_next()->get_start_point_index());

		//IVP_ASSERT ( (int(edge) & 15) == 0);

	    // convexity
	    IVP_U_Hesse hesse;
	    IVP_U_Point p0, p1, p2;
	    p0.set(edge->get_start_point(cl));
	    p1.set(edge->get_next()->get_start_point(cl));
	    p2.set(edge->get_prev()->get_start_point(cl));

	    hesse.calc_hesse(&p0, &p1, &p2);
	    IVP_U_Point pp;
	    pp.set(oppo->get_prev()->get_start_point(cl));
	    IVP_DOUBLE test_dist=hesse.get_dist(&pp);
	    IVP_ASSERT( test_dist >= - P_FLOAT_RES * 10000);

	    // check for a round trip
	    int max_c = 0;
	    for ( const IVP_Compact_Edge *e = edge->get_opposite(); ; e = e->get_next()->get_opposite()){
		if (max_c++ > 1000) CORE;
		if (e->get_opposite() == edge) break; // loop finished
	    }
	}

	tri = tri->get_next_tri();
    }
    return IVP_TRUE;
}
#endif

