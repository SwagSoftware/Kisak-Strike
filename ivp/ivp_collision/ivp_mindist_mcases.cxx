
#include <ivp_physics.hxx>

#include <ivu_float.hxx>

#include <ivp_mindist_intern.hxx>
#include <ivp_mindist_minimize.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>
#include <ivp_compact_ledge_solver.hxx>

// IVP_EXPORT_PRIVATE

/***************************** PF ******************************************/
/***************************** PF ******************************************/
/***************************** PF ******************************************/
/***************************** PF ******************************************/
/***************************** PF ******************************************/
/***************************** PF ******************************************/

// Vertex to Surface

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_PF(const IVP_Compact_Edge *P,
							const IVP_Compact_Edge *F,
							IVP_Cache_Ledge_Point *m_cache_P,
							IVP_Cache_Ledge_Point *m_cache_F)
{
    // Case: Vertex / Surface
    
    // F (the surface) is defined by Q, R (and S)
    // where (Q=F, R=F->next, S=F->prev)
    
    // Checks situation (is P above E?) and takes the appropriate branch.
    
    IVP_ASSERT(m_cache_P->tmp.synapse == mindist->get_sorted_synapse(0));    
#ifdef DEBUG_CHECK_LEN
    check_len_PF(P,F, m_cache_P, m_cache_F);
#endif
    IVP_U_Point p_Fos;
    IVP_CLS.calc_pos_other_space( P, m_cache_P, m_cache_F, &p_Fos);
    
    IVP_Unscaled_QR_Result qr;
    const IVP_Compact_Ledge *c_ledge = m_cache_F->get_compact_ledge();
    IVP_CLS.calc_unscaled_qr_vals_F_space( c_ledge, F, &p_Fos, &qr);

    if ( ! qr.is_outside() ){
	return p_minimize_Leave_PF(P, &p_Fos, F, m_cache_P, m_cache_F);
    }

    // search s_values to find best edge
    const IVP_Compact_Edge *e;
    int j;
    
 // find out the best edge to walk to
    IVP_DOUBLE found_minimal_dist = P_DOUBLE_MAX;
    const IVP_Compact_Edge *min_edge = 0;
    for (e=F,j=0; j<3; e=e->get_next(),j++){
	// if (checks[j] > 0 ) continue; /* inside triangle */
	IVP_DOUBLE dist = IVP_CLS.calc_qlen_PK_K_space( &p_Fos, m_cache_F->get_compact_ledge(), e);
	if(dist < found_minimal_dist){
	    found_minimal_dist = dist;
	    min_edge = e;
	}
    }
    IVP_ASSERT(min_edge);
    return p_minimize_PK( P, min_edge, m_cache_P, m_cache_F);   // s val in range
}


IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_BF(IVP_Cache_Ball *m_cache_B, const IVP_Compact_Edge *F,
				    IVP_Cache_Ledge_Point *m_cache_F)
{
    // Case: Ball - Surface
    
    // F (the surface) is defined by Q, R (and S)
    // where (Q=F, R=F->next, S=F->prev)
    
    // if status indicates friction mode, point is always kept the same
    


    IVP_U_Point p_Fos;
    IVP_U_Point *center_B = m_cache_B->cache_object->m_world_f_object.get_position();
    m_cache_F->clp_cache_object->transform_position_to_object_coords(center_B, &p_Fos);
    
    IVP_Unscaled_QR_Result qr;
    const IVP_Compact_Ledge *c_ledge = m_cache_F->get_compact_ledge();
    IVP_CLS.calc_unscaled_qr_vals_F_space( c_ledge, F, &p_Fos, &qr);

    if ( ! qr.is_outside() ){
	const IVP_U_Float_Point *F_O_os = IVP_CLS.give_object_coords(F,m_cache_F);
    
	IVP_U_Point wHesse_vecF_os;
	IVP_CLS.calc_hesse_vec_object_not_normized(F, m_cache_F->get_compact_ledge(), &wHesse_vecF_os); // should be normized: see below
	wHesse_vecF_os.normize();
    IVP_U_Point contact_plane;
	m_cache_F->get_object_cache()->transform_vector_to_world_coords( &wHesse_vecF_os, &contact_plane);
	mindist->contact_plane.set(&contact_plane);

	IVP_DOUBLE dist_plus_radius = p_Fos.dot_product( &wHesse_vecF_os ) -  wHesse_vecF_os.dot_product(F_O_os);
	m_cache_F->tmp.synapse->update_synapse(F, IVP_ST_TRIANGLE);
	if (dist_plus_radius < 0.0f){	// check for backside
	    m_cache_F->tmp.synapse->update_synapse(F, IVP_ST_BACKSIDE);
	    this->pos_opposite_BacksideOs.set(&p_Fos);
	    return IVP_MRC_BACKSIDE;
	}
	mindist->len_numerator = dist_plus_radius - mindist->sum_extra_radius;

#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
	IVP_U_Float_Point diff_center;
	diff_center.subtract( &m_cache_B->cache_object->core_pos, &m_cache_F->get_object_cache()->core_pos );
	mindist->contact_dot_diff_center = diff_center.dot_product(&mindist->contact_plane);
#endif
	return IVP_MRC_OK;
    }

    // search s_values to find best edge
    const IVP_Compact_Edge *e;
    int j;
    
    // find out the best edge to walk to
    IVP_DOUBLE found_minimal_dist = P_DOUBLE_MAX;
    const IVP_Compact_Edge *min_edge = 0;
    for (e=F,j=0; j<3; e=e->get_next(),j++){
	IVP_DOUBLE dist = IVP_CLS.calc_qlen_PK_K_space(&p_Fos, m_cache_F->get_compact_ledge(), e);
	if(dist < found_minimal_dist){
	    found_minimal_dist = dist;
	    min_edge = e;
	}
    }
    IVP_ASSERT(min_edge);
    return p_minimize_BK( m_cache_B, min_edge, m_cache_F);
}




IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_Leave_PF(const IVP_Compact_Edge *P, const IVP_U_Point *P_Fos,
							      const IVP_Compact_Edge *F,
							      IVP_Cache_Ledge_Point *m_cache_P,
							      IVP_Cache_Ledge_Point *m_cache_F)
{
    // just searching a neighbour point of P, which is nearer to F than P,
    // or, if there is no such point within triangle F, look for nearer edge

    if(--P_Finish_Counter < 0){
	if( check_loop_hash(IVP_ST_POINT, P, IVP_ST_TRIANGLE, F) ){
	    return IVP_MRC_ENDLESS_LOOP;
	}
    }

    
    IVP_U_Point hesse_vec_Pos;
 
    IVP_DOUBLE back_side_check_distance = 0.0f; //-0.5f * ivp_mindist_settings.coll_dist;	// when should we treat surface as backside
    {
	IVP_U_Point wHesse_vecF_Fos;
	IVP_CLS.calc_hesse_vec_object_not_normized(F, m_cache_F->get_compact_ledge(), &wHesse_vecF_Fos); // should be normized: see below
	wHesse_vecF_Fos.fast_normize();
	IVP_U_Point hesse_vec_ws;
	m_cache_F->get_object_cache()->transform_vector_to_world_coords( &wHesse_vecF_Fos, &hesse_vec_ws);
	m_cache_P->get_object_cache()->transform_vector_to_object_coords( &hesse_vec_ws, &hesse_vec_Pos);
    
	// if backside than reverse hesse
	const IVP_U_Float_Point *F_O_Fos = IVP_CLS.give_object_coords(F,m_cache_F);
	mindist->len_numerator = wHesse_vecF_Fos.dot_product( P_Fos ) -  wHesse_vecF_Fos.dot_product(F_O_Fos);
	mindist->contact_plane.set( &hesse_vec_ws);
	
	if (mindist->len_numerator < back_side_check_distance){
	    hesse_vec_Pos.mult(-1.0f);
	}

	mindist->len_numerator -= mindist->sum_extra_radius;

#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
	IVP_U_Float_Point diff_center;
	diff_center.subtract( &m_cache_P->get_object_cache()->core_pos, &m_cache_F->get_object_cache()->core_pos );
	mindist->contact_dot_diff_center = diff_center.dot_product(&mindist->contact_plane);
#endif
    }

#ifdef DEBUG_CHECK_LEN
    check_len_PF(P,F, m_cache_P, m_cache_F);	// check wether world_pos_hash is afflicted
#endif
	
    IVP_DOUBLE min_d_pos = 0.0f;
    const IVP_Compact_Edge *min_edge = NULL;
    {
	// loop through all neighbour points of point and search a nearer one
	// backsides are treated correctly (excluded)
	const IVP_Compact_Poly_Point *we = IVP_CLS.give_object_coords(P,m_cache_P);

	const IVP_Compact_Edge *Pm = P->get_prev();
	const IVP_Compact_Edge *e = Pm->get_opposite()->get_prev();

	for( ; 1; e=e->get_opposite()->get_prev()){ 
	    const IVP_Compact_Poly_Point *wenext = IVP_CLS.give_object_coords(e,m_cache_P);
	    IVP_U_Point vec_Pos;
	    
	    vec_Pos.subtract(wenext, we);
	    IVP_DOUBLE dist = hesse_vec_Pos.dot_product(&vec_Pos);
	    if (dist < 0){
		IVP_DOUBLE e_qlen = vec_Pos.quad_length();
		dist *= IVP_Inline_Math::isqrt_float(e_qlen);
		if(dist  < min_d_pos ){
		    min_edge = e;
		    min_d_pos = dist;
		}
	    }
	    if(e==Pm) break;
	}
    }
    if(min_edge == NULL){ // no nearer point found
	// END: vertex - surface
      
	m_cache_P->tmp.synapse->update_synapse(P, IVP_ST_POINT);
	m_cache_F->tmp.synapse->update_synapse(F, IVP_ST_TRIANGLE);

	if (mindist->len_numerator + mindist->sum_extra_radius <  back_side_check_distance){	// check for backside
	  this->pos_opposite_BacksideOs.set(P_Fos);
	  m_cache_F->tmp.synapse->update_synapse(F, IVP_ST_BACKSIDE);
	  return IVP_MRC_BACKSIDE;
	}
	return IVP_MRC_OK;
    }

    // Is nearest/best P within triangle ?
    // if not, find direction to go (KK case). 
    // remark, this also works if we are already behind the area !!!!
    
    const IVP_Compact_Edge *P2 = min_edge;
    
    // calc direction to go
    IVP_Unscaled_QR_Result qr;
    IVP_U_Point P2_Fos;
    IVP_CLS.calc_pos_other_space(P2, m_cache_P, m_cache_F, &P2_Fos);
    IVP_CLS.calc_unscaled_qr_vals_F_space(F->get_compact_ledge(), F, &P2_Fos, &qr);

    // when new point is located within area -> new PF case with same F
    if ( ! qr.is_outside() ){
	return p_minimize_Leave_PF(P2,&P2_Fos, F, m_cache_P, m_cache_F);
    }


    // now that we are outside, let's search for exactly one edge e of F with these restrictions:
    // 		1. new P is outside e
    //		2. shortest distance between e and P_NP is on e ( 0 < s_val < 1);
    
    int violated_q_rs = 0;
    { // count number of violated q+rs
	for (int j = 0;j<3;j++){
	    if (qr.checks[j] >= 0) continue;
	    violated_q_rs ++;
	}
    }
    // first try to fast find a KK case, as only one qr val is violated
    if (violated_q_rs == 1){
	int j;
	const IVP_Compact_Edge *e;
	for (e=F,j=0; j<3; e=e->get_next(),j++){
	    if (qr.checks[j] >= 0 ) continue;  // inside triangle
	    // check all sides of old area for crossing of new edge and area side
	    return p_minimize_KK(min_edge,e, m_cache_P, m_cache_F);
	}
    }
    IVP_ASSERT( violated_q_rs == 2);
    // now real checking of len
    const IVP_Compact_Edge *best_edge = NULL;
    IVP_DOUBLE best_len = P_DOUBLE_MAX;
    {
	int j;
	const IVP_Compact_Edge *e;
	for (e=F,j=0; j<3; e=e->get_next(),j++){
	    if (qr.checks[j] > 0 ) continue;  // on correct side of triangle, no check needed
	    IVP_DOUBLE len = IVP_CLS.calc_qlen_KK(min_edge, e, m_cache_P, m_cache_F);
	    if (len < best_len){
		best_len = len;
		best_edge = e;
	    }
	}
    }
    return p_minimize_KK(min_edge,best_edge, m_cache_P, m_cache_F);
} // p_minimize_Leave_PF


// OKOKOK


/***************************** KK ******************************************/
/***************************** KK ******************************************/
/***************************** KK ******************************************/
/***************************** KK ******************************************/
/***************************** KK ******************************************/


// Case: Edge - Edge

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_KK(const IVP_Compact_Edge *K, const IVP_Compact_Edge *L,
							IVP_Cache_Ledge_Point *m_cache_K,
							IVP_Cache_Ledge_Point *m_cache_L)
{
    // Calculate next distance and next iteration situation.
    // Use sk und sl are scaling values:
    // in the edge's space: ]0,1[ means inside,
    // pk+ is pk1, if sk>=1, pk- is the other point


    IVP_KK_Input kkin( K, L, m_cache_K, m_cache_L);
    IVP_Unscaled_KK_Result kkr;
    IVP_CLS.calc_unscaled_KK_vals(kkin, &kkr);

    IVP_ASSERT(m_cache_K->tmp.synapse == mindist->get_sorted_synapse(0));
#ifdef DEBUG_CHECK_LEN
    check_len_KK(K,L,m_cache_K, m_cache_L);
#endif
    
    if( !kkr.is_outside_L()) {     
	if ( kkr.checks_K[0] < 0.0f){
	    return p_minimize_PK(K, L, m_cache_K, m_cache_L);
	}
	if ( kkr.checks_K[1] < 0.0f){
	    return p_minimize_PK(K->get_next(), L,  m_cache_K, m_cache_L);
	}
	return p_minimize_Leave_KK(K, L, kkin, &kkr, m_cache_K, m_cache_L);
    }
    
    if( !kkr.is_outside_K()) {
	sort_synapses(m_cache_L->tmp.synapse, m_cache_K->tmp.synapse);

	if(kkr.checks_L[0] < 0.0f){		// sl is already checked for interval [0 .. 1], now check side only
	    return p_minimize_PK(L, K, m_cache_L, m_cache_K);
	}else{
	    return p_minimize_PK(L->get_next(), K, m_cache_L, m_cache_K);
	}
    }

    // Now minimal dist is outside of both edges, meaning that the lines have the nearest distance outside the edges
    // before switching to PP case we have to check whether a PK case
    // should be called

    const IVP_Compact_Edge *pkp, *plp, *plm, *pkm;
    // search point of K which is nearer to L then K->next: == sort points
    if(kkr.checks_K[0]< kkr.checks_K[1]){
	pkp = K;
	pkm = K->get_next();
    }else{
	pkp = K->get_next();
	pkm = K;
    }
    if(kkr.checks_L[0] < kkr.checks_L[1]){
	plp = L;
	plm = L->get_next();
    }else{
	plp = L->get_next();
	plm = L;
    }

    IVP_U_Point pkp_Los;
    IVP_CLS.calc_pos_other_space( pkp, m_cache_K, m_cache_L, &pkp_Los);
    IVP_Unscaled_S_Result sr_kL;
    IVP_CLS.calc_unscaled_s_val_K_space(m_cache_L->get_compact_ledge(), L, & pkp_Los, &sr_kL);

    if(!sr_kL.is_outside()){
	return p_minimize_PK(pkp, L, m_cache_K, m_cache_L);
    }

    IVP_U_Point plp_Kos;
    IVP_CLS.calc_pos_other_space( plp, m_cache_L, m_cache_K, &plp_Kos);
    IVP_Unscaled_S_Result sr_lK;
    IVP_CLS.calc_unscaled_s_val_K_space(m_cache_K->get_compact_ledge(), K, & plp_Kos, &sr_lK);


    if(!sr_lK.is_outside()){
	sort_synapses(m_cache_L->tmp.synapse,m_cache_K->tmp.synapse);
	return p_minimize_PK(plp, K, m_cache_L, m_cache_K);
    }
    
    if( sr_kL.checks[0] * kkr.checks_L[0] < 0.0f){
	return p_minimize_PP(pkp, plm, m_cache_K, m_cache_L);
    }
    
    if( sr_lK.checks[0] * kkr.checks_K[0] < 0.0f){
	sort_synapses(m_cache_L->tmp.synapse,m_cache_K->tmp.synapse);
	return p_minimize_PP(plp, pkm, m_cache_L, m_cache_K);
    }
    
    return p_minimize_PP(pkp, plp, m_cache_K, m_cache_L);
}

class IVP_Leave_KK_Case {
public:
  const IVP_Compact_Edge *F;
  const IVP_Compact_Edge *P;
  const IVP_U_Point *P_Fos;
  IVP_Cache_Ledge_Point *m_cache_P;
  IVP_Cache_Ledge_Point *m_cache_F;
  const IVP_U_Point *H_Fos;
  IVP_U_Point hesse_Fos;
};

// prerequisites:
// synapses are set correctly
// world_coords of points of KK are already set
IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_Leave_KK(const IVP_Compact_Edge *K, const IVP_Compact_Edge *L,
							      const IVP_KK_Input &kkin, const IVP_Unscaled_KK_Result *kkr,
							      IVP_Cache_Ledge_Point *m_cache_K,
							      IVP_Cache_Ledge_Point *m_cache_L)
{
    if(--P_Finish_Counter < 0){
	if( check_loop_hash(IVP_ST_EDGE, K, IVP_ST_EDGE, L) ){
	    // we had this situation before: end
	    return IVP_MRC_ENDLESS_LOOP;
	}
    }
    
    // create plane to determine on which side of K the start point of L can be found
    IVP_U_Point H_Kos;
    IVP_U_Point H_Los;
    IVP_U_Point H_ws;

    int side;// OSR
    {
	IVP_U_Point diff_l_k;
	diff_l_k.subtract(kkin.L_Los[0], &kkin.K_Los[0]);

	IVP_FLOAT val = diff_l_k.dot_product(&kkin.cross_KL_Los);
	IVP_DOUBLE HH = kkin.cross_KL_Los.quad_length();

	side = (*( unsigned int *)&val)>>31;

	IVP_DOUBLE iHQ = IVP_Fast_Math::isqrt(HH,3);

	
	H_Los.set(&kkin.cross_KL_Los);

	m_cache_L->get_object_cache()->transform_vector_to_world_coords( & H_Los, & H_ws);
	m_cache_K->get_object_cache()->transform_vector_to_object_coords( & H_ws, & H_Kos);

	mindist->len_numerator = IVP_Inline_Math::fabsd(val * iHQ);	// no direction of KK
	mindist->len_numerator -= mindist->sum_extra_radius;
	mindist->contact_plane.set_multiple ( &H_ws,( side - 0.5f ) * 2.0f * iHQ);

#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
	IVP_U_Float_Point diff_center;
	diff_center.subtract( &m_cache_K->get_object_cache()->core_pos, &m_cache_L->get_object_cache()->core_pos );
	mindist->contact_dot_diff_center = diff_center.dot_product(&mindist->contact_plane);
#endif	
    }
    

    IVP_BOOL          reverse_side_check[4];	// IVP_TRUE when point sees side from wrong direction
    IVP_Leave_KK_Case cases[4];
    IVP_U_Point L_Kos[2];
    IVP_CLS.transform_pos_other_space( kkin.L_Los[0], m_cache_L, m_cache_K, &L_Kos[0]);
    IVP_CLS.transform_pos_other_space( kkin.L_Los[1], m_cache_L, m_cache_K, &L_Kos[1]);
    {
      cases[0].F          = K->get_opposite();
      cases[0].P          = kkin.L;
      cases[0].P_Fos      = &L_Kos[0];
      cases[0].m_cache_F  = m_cache_K;
      cases[0].m_cache_P  = m_cache_L;
      cases[0].H_Fos      = &H_Kos;

      cases[1].F          = K;
      cases[1].P          = kkin.L->get_opposite();
      cases[1].P_Fos      = &L_Kos[1];
      cases[1].m_cache_F  = m_cache_K;
      cases[1].m_cache_P  = m_cache_L;
      cases[1].H_Fos      = &H_Kos;

      cases[2].F          = L->get_opposite();
      cases[2].P          = kkin.K;
      cases[2].P_Fos      = &kkin.K_Los[0];
      cases[2].m_cache_F  = m_cache_L;
      cases[2].m_cache_P  = m_cache_K;
      cases[2].H_Fos      = &H_Los;
      
      cases[3].F          = L;
      cases[3].P          = kkin.K->get_opposite();
      cases[3].P_Fos      = &kkin.K_Los[1];
      cases[3].m_cache_F  = m_cache_L;
      cases[3].m_cache_P  = m_cache_K;
      cases[3].H_Fos      = &H_Los;
    }
    {
      IVP_Leave_KK_Case *c = &cases[0];
	for(int i=0; i<=3; i++){
	      IVP_CLS.calc_hesse_vec_object_not_normized( c->F, c->F->get_compact_ledge(), &c->hesse_Fos);
	      IVP_FLOAT side_check_dist = c->H_Fos->dot_product(&c->hesse_Fos);
	      int reverse = (*(unsigned int *)&side_check_dist)>>31;
	      reverse ^=  (i>>1) ^ side;
	      reverse_side_check[i] = (IVP_BOOL)reverse;
	      c++;
	}
    }
    
    IVP_DOUBLE min_grad_pos = -P_DOUBLE_RES * 4; // max_double
    const IVP_Compact_Edge *min_edge = NULL;
    const IVP_Compact_Edge *min_plane = NULL;
    IVP_Cache_Ledge_Point *min_edge_m_cache = NULL;
    IVP_Cache_Ledge_Point *min_plane_m_cache = NULL;
    IVP_Unscaled_QR_Result min_qr;
    {

    // check all neigbouring areas !!
      IVP_Leave_KK_Case *cF = &cases[0];
    for(int i=0; i<=3; i++,cF++){
	IVP_Leave_KK_Case *ce = &cases[(i^side) ^ (int)reverse_side_check[i]];
	IVP_DOUBLE delta_h2;
        IVP_U_Point e_dir_Fos;
	{
	    IVP_Leave_KK_Case *ce2 = &cases[i^side];
	    IVP_Leave_KK_Case *ce2_next = &cases[1^(i^side)];	    
	    
	    e_dir_Fos.subtract( ce2->P_Fos, ce2_next->P_Fos );
	    delta_h2 = e_dir_Fos.dot_product(&cF->hesse_Fos);
	}
	
	if (delta_h2 >= 0.0f) continue;

	// calc gradient as real sin(alpha)
	IVP_DOUBLE inv_hesse_real_length = IVP_Inline_Math::isqrt_float(cF->hesse_Fos.quad_length());
	IVP_DOUBLE grad = delta_h2 * IVP_Inline_Math::isqrt_float(e_dir_Fos.quad_length());
	grad *= inv_hesse_real_length;
	if(grad < min_grad_pos){
	    // check side of triangle

	  IVP_Unscaled_QR_Result qr;
	  IVP_CLS.calc_unscaled_qr_vals_F_space( cF->m_cache_F->get_compact_ledge(), cF->F, ce->P_Fos, &qr);
	  
	    if (qr.checks[0] > 0.0f){
		// we now remember plane with best gradient
		min_grad_pos = grad;
		min_edge = ce->P;
		min_edge_m_cache = ce->m_cache_P;
		min_plane = cF->F;
		min_plane_m_cache = cF->m_cache_F;
		min_qr = qr;
	    }else{
		IVP_IF(1) {
		    printf("negative r value in KK: %g\n",qr.checks[0]);
		}
	    }
	}
    }
    }
    if(min_edge == 0){
	m_cache_K->tmp.synapse->update_synapse(K, IVP_ST_EDGE);
	m_cache_L->tmp.synapse->update_synapse(L, IVP_ST_EDGE);

	// End KL	// sk and sl are already set by master
	if (reverse_side_check[0] + reverse_side_check[1] == IVP_TRUE + IVP_TRUE){
	    m_cache_K->tmp.synapse->update_synapse(K, IVP_ST_BACKSIDE);
	    m_cache_L->tmp.synapse->update_synapse(L, IVP_ST_TRIANGLE);
	    IVP_DOUBLE sl = kkr->checks_L[0] / ( kkr->checks_L[0] + kkr->checks_L[1] );
	    this->pos_opposite_BacksideOs.set_interpolate(&L_Kos[0], &L_Kos[1], sl);
	    return IVP_MRC_BACKSIDE;
	}
	if (reverse_side_check[2] + reverse_side_check[3] == IVP_TRUE + IVP_TRUE){
	    m_cache_K->tmp.synapse->update_synapse(K, IVP_ST_TRIANGLE);
	    IVP_DOUBLE sk = kkr->checks_K[0] / ( kkr->checks_K[0] + kkr->checks_K[1] );
	    this->pos_opposite_BacksideOs.set_interpolate( &kkin.K_Los[0], &kkin.K_Los[1], sk );
	    m_cache_L->tmp.synapse->update_synapse(L, IVP_ST_BACKSIDE);
	    return IVP_MRC_BACKSIDE;
	}
	return IVP_MRC_OK;
    }
    
    /** Now we found a good plane,
	  check for a KK case of neighbouring edges */
    sort_synapses(min_edge_m_cache->tmp.synapse,min_plane_m_cache->tmp.synapse);

    {
	if (!min_qr.is_outside()){// nearer point is inside new area->PF
	    IVP_U_Point edge_Fos;
	    IVP_CLS.calc_pos_other_space( min_edge, min_edge_m_cache, min_plane_m_cache,&edge_Fos);
	    return p_minimize_Leave_PF(min_edge,&edge_Fos,  min_plane, min_edge_m_cache, min_plane_m_cache);
	}

	if (min_qr.checks[2] >= 0.0f){	// other side is inside
	    return p_minimize_KK(min_edge, min_plane->get_next(), min_edge_m_cache, min_plane_m_cache);	// no ideas about backsides !!!
	}

	if ( min_qr.checks[1] >= 0.0f){
	    return p_minimize_KK(min_edge, min_plane->get_prev(), min_edge_m_cache, min_plane_m_cache);	// no ideas about backsides !!!
	}
    }
    // now we are outside, find new KK s vales of other edges of new area
    // which edge to take? calc and compare dists...
    {
	IVP_DOUBLE dist_next = IVP_CLS.calc_qlen_KK(min_edge, min_plane->get_next(),min_edge_m_cache, min_plane_m_cache);
	IVP_DOUBLE dist_prev = IVP_CLS.calc_qlen_KK(min_edge, min_plane->get_prev()->get_opposite(),min_edge_m_cache, min_plane_m_cache);

	// @@@ slow solution: compare all 3 dists...
#ifdef IVP_MINDIST_BEHAVIOUR_DEBUG
	IVP_DOUBLE dist_plane = calc_qlen_KK(min_edge, min_plane,min_edge_m_cache, min_plane_m_cache);
	if( (dist_plane < dist_next) && (dist_plane < dist_prev) ){
	    CORE;
	    // How could this have happened?
	    // No improvement seems possible, though grad says so...
	    // End KL	// sk and sl are already set by master
	    m_cache_K->tmp.synapse->update_synapse(K, IVP_ST_EDGE);
	    m_cache_L->tmp.synapse->update_synapse(L, IVP_ST_EDGE);
	    return IVP_MRC_OK;
	
	}
#endif
	if(dist_next > dist_prev){
	    return p_minimize_KK(min_edge, min_plane->get_prev(), min_edge_m_cache, min_plane_m_cache);	// no ideas about backsides !!!
	}else{				       
	    return p_minimize_KK(min_edge, min_plane->get_next(), min_edge_m_cache, min_plane_m_cache);	// no ideas about backsides !!!
	}
    }
} // p_minimize_Leave_KK

/***************************** BP ******************************************/
/***************************** BP ******************************************/
/***************************** BP ******************************************/

// Case: Ball - Vertex

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_BP(IVP_Cache_Ball *m_cache_ball, const IVP_Compact_Edge *P, IVP_Cache_Ledge_Point *m_cache_P){

    if(--P_Finish_Counter < 0){
	if( check_loop_hash(IVP_ST_BALL, NULL, IVP_ST_POINT, P) ){
	    // we had this situation before: end
	    return IVP_MRC_ENDLESS_LOOP;
	}
    }
    

    const IVP_U_Point *center_b_ws;
    IVP_U_Point center_ball_Pmos;
    
    IVP_U_Point p_ws; IVP_CLS.give_world_coords_AT(P, m_cache_P, &p_ws);
    center_b_ws = m_cache_ball->cache_object->m_world_f_object.get_position();
    m_cache_P->get_object_cache()->transform_position_to_object_coords(center_b_ws, &center_ball_Pmos);

    IVP_U_Point contact_plane;
    contact_plane.subtract( center_b_ws, &p_ws );
    IVP_DOUBLE dist_plus_radius =  contact_plane.real_length_plus_normize();
    mindist->contact_plane.set(&contact_plane);
    
    mindist->len_numerator = dist_plus_radius - mindist->sum_extra_radius;
    
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
    IVP_U_Float_Point diff_center;
    diff_center.subtract( &m_cache_ball->cache_object->core_pos, &m_cache_P->get_object_cache()->core_pos );
    mindist->contact_dot_diff_center = diff_center.dot_product(&mindist->contact_plane);
#endif    
    IVP_DOUBLE s_grad_max = 0; 
    
    const IVP_Compact_Edge *Kmax = NULL;

    // each vertex P is position, all neighbouring edges of Pm checked
    {
      const IVP_Compact_Edge *Pm = P; // alias
	const IVP_U_Float_Point *Pm_Pmos = IVP_CLS.give_object_coords(Pm, m_cache_P);

	IVP_U_Point Pm_P_Pmos;
	Pm_P_Pmos.subtract( &center_ball_Pmos, Pm_Pmos);
	
	IVP_DOUBLE dist = Pm_P_Pmos.dot_product(Pm_Pmos);

	// each edge
	Pm = Pm->get_prev();
	const IVP_Compact_Edge *e = Pm->get_opposite()->get_prev();


	for( ; 1; e=e->get_opposite()->get_prev()){ 
	    const IVP_U_Float_Point *tp_next_os = IVP_CLS.give_object_coords(e,m_cache_P);
	    IVP_DOUBLE grad = Pm_P_Pmos.dot_product(tp_next_os) - dist;
	    if (grad > 0){
		IVP_DOUBLE i_len = IVP_Inline_Math::isqrt_float( tp_next_os->quad_distance_to(Pm_Pmos));
		grad *= i_len;
		if(grad > s_grad_max){	// search for greatest step to walk
			// now we found a new smax _value, check for reverse areas
			// check whether new point on edge still could see me 
			s_grad_max = grad;
			Kmax = e;
		}
	    }
	    if(e==Pm) break;
	}
    }

    if(Kmax == NULL){
	// END AB
      end_BP:
	m_cache_P->tmp.synapse->update_synapse(P, IVP_ST_POINT);
	return IVP_MRC_OK;
    }
    IVP_Unscaled_S_Result sr;
    IVP_CLS.calc_unscaled_s_val_K_space(m_cache_P->get_compact_ledge(), Kmax, &center_ball_Pmos, &sr); //
    if (sr.checks[1] <= 0){
      IVP_IF(1){
	printf("BP epsilon problem\n");
      }
      goto end_BP;
    }

    // let's walk, check PP case first
    if(sr.checks[0] < 0 ){
	return p_minimize_BP(m_cache_ball, Kmax, m_cache_P);
    }
    return p_minimize_BK(m_cache_ball, Kmax, m_cache_P);
}


/***************************** PP ******************************************/
/***************************** PP ******************************************/
/***************************** PP ******************************************/


IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_PP(const IVP_Compact_Edge *A, const IVP_Compact_Edge *B,
				    IVP_Cache_Ledge_Point *m_cache_A,
				    IVP_Cache_Ledge_Point *m_cache_B)
{
    // Checks for a better edge, i.e. an edge that points in the right direction
    // (maybe the other vertex of the edge is the next vertex to consider)

    if(--P_Finish_Counter < 0){
	if( check_loop_hash(IVP_ST_POINT, A, IVP_ST_POINT, B) ){
	    // had this situation before: end
	    return IVP_MRC_ENDLESS_LOOP;
	}
    }
    
    IVP_ASSERT(m_cache_A->tmp.synapse == mindist->get_sorted_synapse(0));
#ifdef DEBUG_CHECK_LEN
    check_len_PP(A, B, m_cache_A, m_cache_B);
#endif

    

    const IVP_Compact_Edge *Point[2];
    Point[0] = A; Point[1] = B;

    IVP_U_Point points_ws[2];
    IVP_CLS.give_world_coords_AT(A, m_cache_A, &points_ws[0]);
    IVP_CLS.give_world_coords_AT(B, m_cache_B, &points_ws[1]);
    
    IVP_U_Point point_other_os[2];
    m_cache_B->get_object_cache()->transform_position_to_object_coords(&points_ws[0], &point_other_os[0]);
    m_cache_A->get_object_cache()->transform_position_to_object_coords(&points_ws[1], &point_other_os[1]);

    {
	IVP_DOUBLE qlen = points_ws[0].quad_distance_to(&points_ws[1]);
	if (qlen > P_DOUBLE_RES){
		IVP_DOUBLE inv_len = IVP_Fast_Math::isqrt(qlen,3);
		mindist->len_numerator = qlen * inv_len - mindist->sum_extra_radius;
		mindist->contact_plane.inline_subtract_and_mult( &points_ws[0], &points_ws[1], inv_len);
	}else{
	    return IVP_MRC_ENDLESS_LOOP;
	}

#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
	IVP_U_Float_Point diff_center;
	diff_center.subtract( &m_cache_A->get_object_cache()->core_pos, &m_cache_B->get_object_cache()->core_pos );
	mindist->contact_dot_diff_center = diff_center.dot_product(&mindist->contact_plane);
#endif
    }    
    
    IVP_Cache_Ledge_Point *m_cache[2];
    m_cache[0] = m_cache_A; m_cache[1] = m_cache_B;
    IVP_DOUBLE s_grad_max = 0.0f;

    const IVP_U_Point *P_Pmos_max = NULL;
    const IVP_Compact_Edge *Kmax = NULL;
    const IVP_Compact_Edge *Pmax = NULL;
    IVP_Cache_Ledge_Point *m_cache_K_max = NULL;
    IVP_Cache_Ledge_Point *m_cache_P_max = NULL;
    

    // for each point, all neighbouring edges of Pm (=opposite point) are checked
    for(int ip=1; ip>=0; ip--){
      //IVP_Cache_Ledge_Point *m_cache_P = m_cache[ip];
      IVP_Cache_Ledge_Point *m_cache_Pm = m_cache[1-ip];

	const IVP_U_Point *P_Pmos = &point_other_os[ip];
	const IVP_Compact_Edge *Pm = Point[1-ip];
	const IVP_U_Float_Point *Pm_Pmos = IVP_CLS.give_object_coords(Pm, m_cache_Pm);
	IVP_U_Point Pm_P_Pmos;
	Pm_P_Pmos.subtract( P_Pmos, Pm_Pmos);
		
	// each edge
	IVP_DOUBLE dist = Pm_P_Pmos.dot_product(Pm_Pmos);

	Pm = Pm->get_prev();

	const IVP_Compact_Edge *e = Pm->get_opposite()->get_prev();

	for( ; 1; e = e->get_opposite()->get_prev()){ 
	    const IVP_U_Float_Point *tp_next_os = IVP_CLS.give_object_coords(e,m_cache_Pm);
	    IVP_DOUBLE grad = Pm_P_Pmos.dot_product(tp_next_os) - dist;
	    if (grad > 0) {
		IVP_DOUBLE i_len = IVP_Inline_Math::isqrt_float( tp_next_os->quad_distance_to(Pm_Pmos));
		grad *= i_len;
		if(grad > s_grad_max){	// search for greatest step to walk
			// now we found a new smax _value, check for espilon
		  P_Pmos_max = P_Pmos;
		    s_grad_max = grad;
		    Kmax = e;
		    Pmax = Point[ip];
		    m_cache_K_max = m_cache_Pm;
		    m_cache_P_max = m_cache[ip];
		}
	    }
	    if(e==Pm) break;
	}
    }

    if(Kmax == NULL){
	// END AB
    end_PP:
	m_cache_A->tmp.synapse->update_synapse(A, IVP_ST_POINT);
	m_cache_B->tmp.synapse->update_synapse(B, IVP_ST_POINT);
	
	return IVP_MRC_OK;
    }

    IVP_Unscaled_S_Result sr;
    IVP_CLS.calc_unscaled_s_val_K_space(m_cache_K_max->get_compact_ledge(), Kmax, P_Pmos_max, &sr); //
    if (sr.checks[1] <= 0){
      IVP_IF(1){
	printf("PP epsilon problem\n");
      }
      goto end_PP;
    }
    
    // let's walk, check PP case first
    if(sr.checks[0] < 0.0f){
	sort_synapses(m_cache_K_max->tmp.synapse,m_cache_P_max->tmp.synapse);
	return p_minimize_PP(Kmax, Pmax, m_cache_K_max, m_cache_P_max);
    }

    sort_synapses(m_cache_P_max->tmp.synapse,m_cache_K_max->tmp.synapse);
    return p_minimize_Leave_PK(Pmax, Kmax, m_cache_P_max, m_cache_K_max);
}

/***************************** BK ******************************************/
/***************************** BK ******************************************/
/***************************** BK ******************************************/
// Do not use this class yet.
//template<class T_Base, class T_Offset, int N> class IVP_Matrix_Ortho3;

// Case: Ball - Edge

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_BK(IVP_Cache_Ball *m_cache_B, const IVP_Compact_Edge *K,
							IVP_Cache_Ledge_Point *m_cache_K     )
{
    IVP_U_Point P_Kos;
    {
      IVP_U_Point *wP = m_cache_B->cache_object->m_world_f_object.get_position();
      m_cache_K->get_object_cache()->transform_position_to_object_coords( wP, &P_Kos);
    }
    
    IVP_Unscaled_S_Result sr;
    IVP_CLS.calc_unscaled_s_val_K_space( m_cache_K->get_compact_ledge(), K, &P_Kos, &sr);    

    if(sr.checks[0] < 0.0f){
	return p_minimize_BP(m_cache_B, K, m_cache_K);
    }
    
    if(sr.checks[1] < 0.0f){
	return p_minimize_BP(m_cache_B, K->get_next(), m_cache_K);
    }
    return p_minimize_Leave_BK(m_cache_B, K, m_cache_K);
}


IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_Leave_BK(IVP_Cache_Ball *m_cache_ball, const IVP_Compact_Edge *K,
							      IVP_Cache_Ledge_Point *m_cache_K)
{
    if(--P_Finish_Counter < 0){
	if( check_loop_hash(IVP_ST_BALL, NULL, IVP_ST_EDGE, K) ){
	    return IVP_MRC_ENDLESS_LOOP;
	}
    }

    // ********** first step, check for PF cases
    IVP_U_Point p_ks; // point p in K space
    {
      IVP_U_Point *p_ws = m_cache_ball->cache_object->m_world_f_object.get_position();
      m_cache_K->get_object_cache()->transform_position_to_object_coords( p_ws, &p_ks);
    }
    
    // get direction of edge K
    const IVP_U_Float_Point *k_start_os    = IVP_CLS.give_object_coords(K, m_cache_K);		// a point on an edge
    const IVP_U_Float_Point *k_next_os     = IVP_CLS.give_object_coords(K->get_next(), m_cache_K);	// next point
    const IVP_U_Float_Point *k_tri_os      = IVP_CLS.give_object_coords(K->get_prev(), m_cache_K);	// prev point == other point of this triange
    const IVP_U_Float_Point *k_oppo_tri_os = IVP_CLS.give_object_coords(K->get_opposite()->get_prev(), m_cache_K);	// point of opposite triangle
    
    IVP_U_Point vec_K_ks;    	vec_K_ks.subtract( k_next_os, k_start_os);	// not normized direction yet
    
    IVP_U_Point vec_K_P;	vec_K_P.subtract( &p_ks, k_start_os);

    IVP_U_Point vec_K_tri;   	vec_K_tri.subtract( k_tri_os, k_start_os);
    IVP_U_Point vec_K_oppo_tri; vec_K_oppo_tri.subtract( k_oppo_tri_os, k_start_os);
    
    IVP_U_Point hesse_tri_os;      hesse_tri_os.calc_cross_product( &vec_K_ks, &vec_K_tri);		// vertical to triangle
    IVP_U_Point hesse_oppo_tri_os; hesse_oppo_tri_os.calc_cross_product( &vec_K_oppo_tri, &vec_K_ks); // v. to oppo tri

    IVP_DOUBLE vert[2];	// dist between point and triangle, oppo tri
    vert[0] = vec_K_P.dot_product( &hesse_tri_os);
    vert[1] = vec_K_P.dot_product( &hesse_oppo_tri_os);
    
     // new verion using qr
    IVP_Unscaled_QR_Result qr0,qr1;
    IVP_CLS.calc_unscaled_qr_vals_F_space(m_cache_K->get_compact_ledge(), K, &p_ks, &qr0);
    IVP_CLS.calc_unscaled_qr_vals_F_space(m_cache_K->get_compact_ledge(), K->get_opposite(), &p_ks, &qr1);

	
    if ( qr0.checks[0] > 0.0f){	// check projected distance
	if (qr1.checks[0] > 0.0f){	// both planes better,
	    if (vert[1] > 0){		// plane visible, if true than it has priority
		return p_minimize_BF(m_cache_ball, K->get_opposite(), m_cache_K);
	    }
	}
	return p_minimize_BF(m_cache_ball, K, m_cache_K);
    }
    
    if (qr1.checks[0] > 0.0f){
	return p_minimize_BF(m_cache_ball, K->get_opposite(), m_cache_K);
    }
    
    // check for backside !!
    if ( vert[0] < -P_DOUBLE_EPS && vert[1] < -P_DOUBLE_EPS){	// both from behind
      this->pos_opposite_BacksideOs.set(&p_ks);
      m_cache_K->tmp.synapse->update_synapse(K, IVP_ST_BACKSIDE);
      return IVP_MRC_BACKSIDE;
    }

    IVP_U_Point Lot2; // unnormized Lot
    Lot2.calc_cross_product( &vec_K_ks, &vec_K_P);

    IVP_DOUBLE iqK_len = 1.0f / vec_K_ks.quad_length();
    IVP_DOUBLE qdist = Lot2.quad_length() * iqK_len;
    IVP_DOUBLE inv_len = IVP_Inline_Math::isqrt_double(qdist);
    mindist->len_numerator = qdist * inv_len - mindist->sum_extra_radius;

    Lot2.calc_cross_product( &vec_K_ks, &Lot2 ); // does change the length !!!

    m_cache_K->get_object_cache()->transform_vector_to_world_coords( &Lot2, &Lot2);

    mindist->contact_plane.set_multiple( &Lot2,  - inv_len * iqK_len);

#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
    IVP_U_Float_Point diff_center;
    diff_center.subtract( &m_cache_ball->cache_object->core_pos, &m_cache_K->get_object_cache()->core_pos );
    mindist->contact_dot_diff_center = diff_center.dot_product(&mindist->contact_plane);
#endif
    
    
    m_cache_K->tmp.synapse->update_synapse(K, IVP_ST_EDGE);
    
    return IVP_MRC_OK;
}


/***************************** PK ******************************************/
/***************************** PK ******************************************/
/***************************** PK ******************************************/

// Case: Vertex - Edge

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_PK(const IVP_Compact_Edge *P, const IVP_Compact_Edge *K,
				      IVP_Cache_Ledge_Point *m_cache_P,
				      IVP_Cache_Ledge_Point *m_cache_K)
{
    // Checks wether we can switch to the case PP (vertex - vertex)
    // If not: take appropriate branch
    IVP_ASSERT( m_cache_P->tmp.synapse == mindist->get_sorted_synapse(0));
#ifdef DEBUG_CHECK_LEN
    check_len_PK(P, K, m_cache_P, m_cache_K);
#endif
    IVP_U_Point P_Kos;
    IVP_CLS.calc_pos_other_space(P, m_cache_P, m_cache_K,&P_Kos );

    IVP_Unscaled_S_Result sr;
    IVP_CLS.calc_unscaled_s_val_K_space( m_cache_K->get_compact_ledge(), K, &P_Kos, &sr);

    if(sr.checks[0] < 0.0f){
	return p_minimize_PP(P, K, m_cache_P, m_cache_K);
    }
    if(sr.checks[1] < 0.0f){
	return p_minimize_PP(P, K->get_next(), m_cache_P, m_cache_K);
    }
    return p_minimize_Leave_PK(P, K, m_cache_P, m_cache_K);
}

IVP_MRC_TYPE IVP_Mindist_Minimize_Solver::p_minimize_Leave_PK(const IVP_Compact_Edge *P,const IVP_Compact_Edge *K,
					    IVP_Cache_Ledge_Point *m_cache_P,
					    IVP_Cache_Ledge_Point *m_cache_K)
{
    if(--P_Finish_Counter < 0){
	if( check_loop_hash(IVP_ST_POINT, P, IVP_ST_EDGE, K) ){
	    // had this situation before: end
	    return IVP_MRC_ENDLESS_LOOP;
	}
    }
    
#ifdef DEBUG_CHECK_LEN
    check_len_PK(P, K, m_cache_P, m_cache_K);
#endif
    // ********** first step, check for PF cases
    IVP_U_Point p_ks; // point p in K space
    IVP_CLS.calc_pos_other_space( P, m_cache_P, m_cache_K,&p_ks);
 
    // get direction of edge K
    const IVP_U_Float_Point *k_start_os    = IVP_CLS.give_object_coords(K, m_cache_K);		        // a point on an edge
    const IVP_U_Float_Point *k_next_os     = IVP_CLS.give_object_coords(K->get_next(), m_cache_K);	// next point
    const IVP_U_Float_Point *k_tri_os      = IVP_CLS.give_object_coords(K->get_prev(), m_cache_K);	// prev point == other point of this triange
    const IVP_U_Float_Point *k_oppo_tri_os = IVP_CLS.give_object_coords(K->get_opposite()->get_prev(), m_cache_K);	// point of opposite triangle
    

    IVP_U_Point vec_K_ks;    	vec_K_ks.subtract( k_next_os, k_start_os);	// unnormized direction
    
    IVP_U_Point vec_K_P;	vec_K_P.subtract( &p_ks, k_start_os);

    IVP_U_Point vec_K_tri;   	vec_K_tri.subtract( k_tri_os, k_start_os);
    IVP_U_Point vec_K_oppo_tri; vec_K_oppo_tri.subtract( k_oppo_tri_os, k_start_os);

    {
	//IVP_DOUBLE K_ks_len = vec_K_ks.real_length();
	//IVP_DOUBLE i_K_ks_len = 1.0f / K_ks_len;
	//vec_K_ks.mult(i_K_ks_len); // normize K_ks
    }
    //vec_K_ks.fast_normize();
    
    IVP_U_Point hesse_tri_os;      hesse_tri_os.calc_cross_product( &vec_K_ks, &vec_K_tri);		// vertical to triangle
    IVP_U_Point hesse_oppo_tri_os; hesse_oppo_tri_os.calc_cross_product( &vec_K_oppo_tri, &vec_K_ks); // v. to oppo tri
    
    IVP_DOUBLE vert[2];	// dist between point and triangle, oppo tri
    vert[0] = vec_K_P.dot_product( &hesse_tri_os);
    vert[1] = vec_K_P.dot_product( &hesse_oppo_tri_os);

     // new verion using qr
    IVP_Unscaled_QR_Result qr0,qr1;
    IVP_CLS.calc_unscaled_qr_vals_F_space(m_cache_K->get_compact_ledge(), K, &p_ks, &qr0);
    IVP_CLS.calc_unscaled_qr_vals_F_space(m_cache_K->get_compact_ledge(), K->get_opposite(), &p_ks, &qr1);
    
    if ( qr0.checks[0] > 0.0f){	// check projected distance
	if (qr1.checks[0] > 0.0f){	// both planes better,
	    if (vert[1] > 0){		// plane visible, if true than it has priority
		return p_minimize_PF(P, K->get_opposite(), m_cache_P, m_cache_K);
	    }
	}
	return p_minimize_PF(P, K, m_cache_P, m_cache_K);
    }
    
    if (qr1.checks[0] > 0.0f){
	return p_minimize_PF(P, K->get_opposite(), m_cache_P, m_cache_K);
    }
    
    // check for backside !!
    if ( vert[0] < 0 && vert[1] < 0){	// both from behind
	m_cache_P->tmp.synapse->update_synapse(P, IVP_ST_POINT);
	m_cache_K->tmp.synapse->update_synapse(K, IVP_ST_BACKSIDE);
	this->pos_opposite_BacksideOs.set(&p_ks);
	return IVP_MRC_BACKSIDE;
    }


    // ********* next step, find KK cases

    // each edge: find minimal angle between P and Lot( P, K)


    IVP_U_Point Lot2; // unnormized Lot
    IVP_DOUBLE qlen;
    {	// Lot _ws
	
	Lot2.calc_cross_product( &vec_K_ks, &vec_K_P);

	IVP_DOUBLE iqK_len = 1.0f / vec_K_ks.quad_length();
	qlen = Lot2.quad_length() * iqK_len;
	if (qlen > P_DOUBLE_EPS){
	    IVP_DOUBLE inv_len = IVP_Inline_Math::isqrt_double(qlen);
	    mindist->len_numerator = qlen * inv_len - mindist->sum_extra_radius;
	    Lot2.calc_cross_product( &vec_K_ks, &Lot2 ); // does not change the length !!!
	    m_cache_K->get_object_cache()->transform_vector_to_world_coords( &Lot2, &Lot2);
	    mindist->contact_plane.set_multiple( &Lot2, - inv_len * iqK_len);
	}else{
	    mindist->len_numerator = - mindist->sum_extra_radius;
	    IVP_U_Point orth; orth.calc_an_orthogonal( &vec_K_ks );
	    orth.normize();
	    mindist->contact_plane.set(&orth);
	}
	//IVP_ASSERT( mindist->contact_plane.real_length() < 1.001f);

#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
	IVP_U_Float_Point diff_center;
	diff_center.subtract( &m_cache_P->get_object_cache()->core_pos, &m_cache_K->get_object_cache()->core_pos );
	mindist->contact_dot_diff_center = diff_center.dot_product(&mindist->contact_plane);
#endif
    }

    // Lot _point_space
    m_cache_P->get_object_cache()->transform_vector_to_object_coords( &Lot2, &Lot2);

    const IVP_Compact_Edge *NKmax = NULL;    
    IVP_DOUBLE max_s_val = P_DOUBLE_RES * qlen;

    const IVP_U_Float_Point *p_os = IVP_CLS.give_object_coords(P, m_cache_P);

    const IVP_Compact_Edge *Pm = P->get_prev();
    const IVP_Compact_Edge *e = Pm->get_opposite()->get_prev();

    for( ; 1; e=e->get_opposite()->get_prev()){ 
	IVP_U_Point e_vec_Pos;
	e_vec_Pos.subtract( IVP_CLS.give_object_coords(e, m_cache_P), p_os);
	IVP_DOUBLE angle = e_vec_Pos.dot_product(&Lot2);
	if (angle > 0){
	    angle *= IVP_Inline_Math::isqrt_float( e_vec_Pos.quad_length());
	    if(angle > max_s_val){
		max_s_val = angle;
		NKmax = e;
	    }
	}
	if(e==Pm) break;
    }

    while(NKmax != NULL){
	if (max_s_val < P_RES_EPS){	// maybe an epsilon problem
	  IVP_KK_Input kkin( K, NKmax, m_cache_K, m_cache_P);
	  IVP_Unscaled_KK_Result kkr;
	  IVP_RETURN_TYPE check = IVP_CLS.calc_unscaled_KK_vals(kkin, &kkr);
	  if (check == IVP_FAULT || kkr.checks_L[0] < 0.0f){
	    IVP_IF(0) {
	      printf("PK_KK epsilon problem\n");
	    }
	    break;
	  }
	}
	return p_minimize_KK(NKmax, K, m_cache_P, m_cache_K);
    }

    // END PK
    m_cache_P->tmp.synapse->update_synapse(P, IVP_ST_POINT);
    m_cache_K->tmp.synapse->update_synapse(K, IVP_ST_EDGE);

    return IVP_MRC_OK;
} //p_minimize_Leave_PK


