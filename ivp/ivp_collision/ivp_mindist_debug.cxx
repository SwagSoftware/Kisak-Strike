// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>
#include <ivu_geometry.hxx>

#include <ivp_debug.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_mindist_minimize.hxx>


#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>
#include <ivp_compact_ledge_solver.hxx>


/************************************************************/
/************************************************************/
/************************************************************/
/************************************************************/
/************************************************************/

// NOT YET FULLY TRANSFORMED TO COMPACT LEDGE STYLE !!! -OG

/************************************************************/
/************************************************************/
/************************************************************/
/************************************************************/
/************************************************************/


IVP_BOOL ivp_check_debug_mindist( IVP_Mindist *md){
    if (! IVP_DEBUG_OBJECT0 ) return IVP_FALSE;
    if (! IVP_DEBUG_OBJECT1 ) return IVP_FALSE;
    const char *search0 = IVP_DEBUG_OBJECT0;
    const char *search1 = IVP_DEBUG_OBJECT1;

    const char *name0 = md->get_synapse(0)->get_object()->get_name();
    const char *name1 = md->get_synapse(1)->get_object()->get_name();
    if (	!P_String::string_cmp(name0, search0, IVP_FALSE) && !P_String::string_cmp(name1, search1, IVP_FALSE) ||
		!P_String::string_cmp(name0, search1, IVP_FALSE) && !P_String::string_cmp(name1, search0, IVP_FALSE)){
	const IVP_Time &time = md->get_synapse(0)->l_obj->get_environment()->get_current_time();
	if ( time - IVP_DEBUG_TIME >= 0.0f){
	    printf("%10s %10s %8f-%X: ", name0, name1, time.get_time(), (intptr_t)md & 0xffff );
	    return IVP_TRUE;
	}
    }
    return IVP_FALSE;
}


int p_sqrt( IVP_DOUBLE val) // for debugger
{
    printf("%g\n", IVP_Inline_Math::sqrtd(val));
    return 0;
}


#ifdef DEBUG
// copy-paste from p_minimize_FF for proove_polypoly()
IVP_DOUBLE IVP_Mindist_Minimize_Solver::p_optimize_FF( const IVP_Compact_Edge *A, const IVP_Compact_Edge *B,
			  IVP_Cache_Ledge_Point *m_cache_A,  IVP_Cache_Ledge_Point *m_cache_B,
			  IVP_DOUBLE min_qdist){
    // Case: Surface - Surface
    
    // check all 9 point point combinations first:
    {
	const IVP_Compact_Edge *pA, *pB;
	int a, b;
	for (pA=A, a=0; a<3; pA=pA->get_next(),a++){
	    for (pB=B,b=0; b<3; pB=pB->get_next(), b++){
		IVP_U_Point pos_B_ws; IVP_CLS.give_world_coords_AT(pB, m_cache_B, &pos_B_ws);
		IVP_U_Point pos_A_ws; IVP_CLS.give_world_coords_AT(pA, m_cache_A, &pos_A_ws);

		IVP_DOUBLE qdist = pos_A_ws.quad_distance_to(&pos_B_ws);
		if (qdist < min_qdist){
		    min_qdist = qdist;
		}
	    }
	}
    }
    // check all 6 point area combinations
    IVP_Cache_Ledge_Point *cc_A[2]; cc_A[0] = m_cache_A; cc_A[1] = m_cache_B;
    IVP_Cache_Ledge_Point *cc_B[2]; cc_B[0] = m_cache_B; cc_B[1] = m_cache_A;
    const IVP_Compact_Edge *tabA[2]; tabA[0] = A; tabA[1] = B;
    const IVP_Compact_Edge *tabB[2]; tabB[0] = B; tabB[1] = A;
    {
	for (int i=0;i<2;i++){
	    const IVP_Compact_Edge *pA;
	    const IVP_Compact_Edge *pB= tabB[i];
	    int a;
	    IVP_U_Hesse hesse_B;
	    IVP_CLS.calc_hesse_object(pB, cc_B[i]->get_compact_ledge(), &hesse_B);

	    for ( pA=tabA[i],a=0; a<3; pA=pA->get_next(),a++){
	      IVP_U_Point pA_Bos;
	      IVP_CLS.calc_pos_other_space( pA, cc_A[i], cc_B[i], &pA_Bos );
	      IVP_Unscaled_QR_Result qr;
	      IVP_CLS.calc_unscaled_qr_vals_F_space(cc_B[i]->get_compact_ledge(), pB, &pA_Bos, &qr);
	      if ( !qr.is_outside()){
		IVP_DOUBLE qdist = hesse_B.get_dist( &pA_Bos );
		qdist *= qdist;
		if ( qdist * (1+P_DOUBLE_RES) < min_qdist){
		  min_qdist = qdist;
		}
	      }
	    }
	}
    }

    // check all 18 point edge combinations
    {
	int i;
	for (i=0; i<2; i++){
	    const IVP_Compact_Edge *pA, *pB;
	    int a, b;
	    for (pB=tabB[i],b=0; b<3; pB=pB->get_next(),b++){
	      IVP_U_Point pB_Aos;
	      IVP_CLS.calc_pos_other_space( pB, cc_B[i], cc_A[i], &pB_Aos );
	      
	      for (pA=tabA[i],a=0; a<3; pA=pA->get_next(),a++){
		IVP_Unscaled_S_Result sr;
		IVP_CLS.calc_unscaled_s_val_K_space(  cc_A[i]->get_compact_ledge(), pA, &pB_Aos, &sr);
		if (sr.is_outside()) continue;
		IVP_DOUBLE qdist = IVP_CLS.quad_dist_edge_to_point_K_space(cc_A[i]->get_compact_ledge(), pA, &pB_Aos );
		if ( qdist * (1+P_DOUBLE_RES) < min_qdist){
		  min_qdist = qdist;
		}
	      }
	    }
	}
    }
    {
	// check all 9 edge edge combinations
	const IVP_Compact_Edge *pA, *pB;
	int a, b;
	for (pA=A,a=0; a<3; pA=pA->get_next(),a++){
	    for (pB=B,b=0; b<3; pB=pB->get_next(),b++){
	      IVP_KK_Input kkin( pA, pB, m_cache_A, m_cache_B);
	      IVP_DOUBLE quad_dist = kkin.cross_KL_Los.quad_length();
	      if(quad_dist <= (P_DOUBLE_RES * P_DOUBLE_RES * P_MAX_OBJECT_SIZE * P_MAX_OBJECT_SIZE)){ // not parallel
		  continue;
	      }
	      IVP_Unscaled_KK_Result kkr;
	      IVP_CLS.calc_unscaled_KK_vals(kkin, &kkr);
	      if (kkr.is_outside_K()) continue;
	      if (kkr.is_outside_L()) continue;
	      
	      IVP_DOUBLE qdist = kkin.calc_quad_distance_edge_edge();
		if (qdist * (1+P_DOUBLE_RES) < min_qdist){
		    min_qdist = qdist;
		}
	    }
	}
    }
    return min_qdist;
}
#endif


#ifdef DEBUG
IVP_RETURN_TYPE IVP_Mindist_Minimize_Solver::proove_polypoly()
{
    // does a REAL SLOW (!!!) brute force recalculation and check.

    // requires collision free situation
    IVP_Synapse_Real *syn0, *syn1;
    
    syn0 = mindist->get_synapse(0);
    syn1 = mindist->get_synapse(1);

    // get caches
    IVP_Polygon *poly_0 = syn0->get_object()->to_poly();
    IVP_Polygon *poly_1 = syn1->get_object()->to_poly();
    
    
    const IVP_Compact_Edge *e0 = syn0->edge;
    const IVP_Compact_Edge *e1 = syn1->edge;
    
    const IVP_Compact_Ledge *ledge0 = e0->get_compact_ledge();
    const IVP_Compact_Ledge *ledge1 = e1->get_compact_ledge();

    IVP_Cache_Ledge_Point m_cache_0; m_cache_0.init_cache_ledge_point(poly_0,ledge0);
    IVP_Cache_Ledge_Point m_cache_1; m_cache_1.init_cache_ledge_point(poly_1,ledge1);
    
    m_cache_0.tmp.synapse = syn0;	// remember order of synapses
    m_cache_1.tmp.synapse = syn1;
    
    IVP_DOUBLE min_qdist = P_DOUBLE_MAX;

    const IVP_Compact_Triangle *best_t0;
    const IVP_Compact_Triangle *best_t1;
   
    // each area of the first object
    for (int l0 = ledge0->get_n_triangles()-1; l0>=0;l0--){
	for (int l1 = ledge1->get_n_triangles()-1; l1>=0;l1--){
	    const IVP_Compact_Triangle *t0 = &ledge0->get_first_triangle()[l0];
	    const IVP_Compact_Triangle *t1 = &ledge1->get_first_triangle()[l1];
	    
	    IVP_DOUBLE qd = p_optimize_FF(t0->get_first_edge(), t1->get_first_edge(), &m_cache_0, &m_cache_1, min_qdist);
	    if(qd < min_qdist){
		// found better situation, synapses are already updated
		best_t0 = t0;
		best_t1 = t1;
		min_qdist = qd;
	    }
	}	
    }
    m_cache_0.remove_reference();
    m_cache_1.remove_reference();
    
    // compare brute force result with original mindist state
	IVP_DOUBLE dist_0 = sqrt(min_qdist);
	IVP_DOUBLE dist_1 = mindist->get_length();
	IVP_DOUBLE diff = IVP_Inline_Math::fabsd(dist_0 - dist_1) + P_DOUBLE_EPS;
	
	IVP_DOUBLE perc = diff * 100.0f / dist_1;
        // printf("\nBrute Force Recalculation of Mindist shows differences:\n");
	// printf("Distances: md %f, bf %f, diff: %f percent: %f\n", dist_0, dist_1, diff, perc);

	// check if difference is neglectible
	if(perc < 50.0f){
	    return IVP_OK;
	}
	
	// somewhat greater differences ...
	printf("\nBrute Force Recalculation of Mindist shows differences:\n");
	printf("Dsts: md %f, bf %f, diff: %f percent: %f\n", dist_0, dist_1, diff, perc);
	printf("Coords of synapses (original 0/1, brute force 0/1):\n");

	//	IVP_DOUBLE qd0 = p_optimize_FF(best_t0->get_first_edge(), best_t1->get_first_edge(), &m_cache_0, &m_cache_1, min_qdist + P_DOUBLE_RES);
	//IVP_DOUBLE qd1 = p_optimize_FF(best_t0->get_first_edge(), best_t1->get_first_edge(), &m_cache_0, &m_cache_1, min_qdist + P_DOUBLE_RES);

	CORE;
	return IVP_FAULT;
}
#endif 

#ifdef DEBUG
IVP_RETURN_TYPE IVP_Mindist_Minimize_Solver::proove_ballpoly()
{
    // does a REAL SLOW (!!!) brute force recalculation and check.

    // requires collision free situation
    IVP_Synapse_Real  *syn1;
    IVP_Synapse_Real *syn0;
    
    syn0 = mindist->get_synapse(0);
    syn1 = mindist->get_synapse(1);

    // get caches
    IVP_Real_Object *poly_0 = syn0->get_object();
    IVP_Polygon *poly_1 = syn1->get_object()->to_poly();
    
    
    const IVP_Compact_Edge *e1 = syn1->edge;
    const IVP_Compact_Ledge *ledge1 = e1->get_compact_ledge();

    IVP_Cache_Object *m_cache_0 = poly_0->get_cache_object();
    IVP_Cache_Object *m_cache_1 = poly_1->get_cache_object();
       
    IVP_DOUBLE min_qdist = P_DOUBLE_MAX;

    const IVP_Compact_Triangle *best_t1;
    IVP_U_Point *ball_ws;
    IVP_U_Point ball_Fos;
    ball_ws = m_cache_0->m_world_f_object.get_position();
    m_cache_1->transform_position_to_object_coords( ball_ws, &ball_Fos);
   
    for (int l1 = ledge1->get_n_triangles()-1; l1>=0;l1--){
	const IVP_Compact_Triangle *t1 = &ledge1->get_first_triangle()[l1];

	IVP_DOUBLE qd = IVP_CLS.calc_qlen_PF_F_space(ledge1, t1, &ball_Fos);
	if(qd < min_qdist){
	    // found better situation, synapses are already updated
	    best_t1 = t1;
	    min_qdist = qd;
	}
    }
    m_cache_0->remove_reference();
    m_cache_1->remove_reference();
    
    // compare brute force result with original mindist state
    IVP_DOUBLE dist_0 = sqrt(min_qdist) - mindist->sum_extra_radius;
    IVP_DOUBLE dist_1 = mindist->get_length();
    IVP_DOUBLE diff = IVP_Inline_Math::fabsd(dist_0 - dist_1) + P_DOUBLE_EPS;
    
    IVP_DOUBLE perc = diff * 100.0f / dist_1;
    // check if difference is neglectible
    if(perc < 50.0f){
	return IVP_OK;
    }
    
    // somewhat greater differences ...
    printf("\nBrute Force Recalculation of Mindist shows differences:\n");
    printf("Dsts: md %f, bf %f, diff: %f percent: %f\n", dist_0, dist_1, diff, perc);
    printf("Coords of synapses (original 0/1, brute force 0/1):\n");
    
    //IVP_DOUBLE qd0 = IVP_CLS.calc_qlen_PF_F_space(ledge1, best_t1, &ball_Fos);
    //IVP_DOUBLE qd1 = IVP_CLS.calc_qlen_PF_F_space(ledge1, best_t1, &ball_Fos);
    
    CORE;
    return IVP_FAULT;
}
#endif

#ifdef DEBUG
int situation(IVP_Cache_Ledge_Point *tri_cache, IVP_Cache_Ledge_Point *e_cache, IVP_Compact_Edge *triangle_e0, IVP_Compact_Edge *edge)
{
    // may be called from debugger (with print)
    
    // transforms triangle to flat (paper!)
    // also transforms edge
    // prints coords and labels

    // get world coords


    IVP_U_Point t_e_0, t_e_1, t_e_2;
    IVP_CLS.give_world_coords_AT(triangle_e0,tri_cache,&t_e_0);
    IVP_CLS.give_world_coords_AT(triangle_e0->get_next(),tri_cache,&t_e_1);
    IVP_CLS.give_world_coords_AT(triangle_e0->get_prev(),tri_cache,&t_e_2);


    IVP_U_Point e_0, e_1;
    IVP_CLS.give_world_coords_AT(edge,e_cache,&e_0);
    IVP_CLS.give_world_coords_AT(edge->get_next(),e_cache,&e_1);

    /*** search matrix flat on triangle with x-axis = triangle_e0 ***/

    // vector 0 (x) points in direction of triangle_e0
    IVP_U_Point vec_0;
    vec_0.subtract(&t_e_1, &t_e_0);
    vec_0.normize();

    // vector 2 (z) is orthogonal to triangle, i.e. emerging out of the paper
    IVP_U_Hesse vec_2; // world, normized
    IVP_CLS.calc_hesse_normized_AT( triangle_e0, tri_cache, &vec_2);
    vec_2.mult(-1.0f);
    
    // vector 1 (y) orthogonal to it
    IVP_U_Point vec_1;
    vec_1.calc_cross_product(&vec_0, &vec_2);
    vec_1.normize();

    // t_e_0 as origin
    IVP_U_Point trans;
    trans.set(&t_e_0);
    IVP_U_Matrix m_world_from_flat;
    m_world_from_flat.init_columns4(&vec_0, &vec_1, &vec_2, &trans);
    // m_flat_from_world.orthonormize();
    
    /*** transform whole situation ***/

    // transform triangle
    IVP_U_Point tf_0, tf_1, tf_2;
    m_world_from_flat.vimult4(&t_e_0, &tf_0);
    m_world_from_flat.vimult4(&t_e_1, &tf_1);
    m_world_from_flat.vimult4(&t_e_2, &tf_2);

    // transform edge
    IVP_U_Point ef_0, ef_1;
    m_world_from_flat.vimult4(&e_0, &ef_0);
    m_world_from_flat.vimult4(&e_1, &ef_1);

    // calc pierce position (if any)
    ivp_u_bool pierces = IVP_FALSE;
    ivp_u_bool pierce_inside = IVP_FALSE;
    IVP_U_Point pierce_point_f;

    IVP_U_Straight edge_straight;
    IVP_U_Point edge_vec;
    edge_vec.subtract(&e_1, &e_0);
    edge_straight.set(&e_0, &edge_vec);
    IVP_U_Hesse tri_hesse;
    IVP_CLS.calc_hesse_normized_AT( triangle_e0, tri_cache, &tri_hesse);

    IVP_U_Point pierce_point_w;
    IVP_DOUBLE q =0, r = 0;
    if( tri_hesse.calc_intersect_with(&edge_straight, &pierce_point_w) == IVP_OK ){
	pierces = IVP_TRUE;
	
	m_world_from_flat.vimult4(&pierce_point_w, &pierce_point_f);
	IVP_CLS.calc_qr_vals(triangle_e0,&pierce_point_w, &q, &r, tri_cache);
	if ( q >= 0 && r >= 0 && r+q<=1 ){
	    pierce_inside = IVP_TRUE;
	}
    }

    IVP_DOUBLE q_edge_start, q_edge_end, r_edge_start, r_edge_end;
    IVP_CLS.calc_qr_vals(triangle_e0, &e_0, &q_edge_start, &r_edge_start, tri_cache);
    IVP_CLS.calc_qr_vals(triangle_e0, &e_1, &q_edge_end, &r_edge_end, tri_cache);


    IVP_KK_Input kkin( triangle_e0, edge, tri_cache, e_cache);
    IVP_Unscaled_KK_Result kkr;
    IVP_CLS.calc_unscaled_KK_vals(kkin, &kkr);
    
    /** print everything ***/
    printf("triangle edge situation:");
    printf(" (coords so that triangle appears flat):\n\n");
    printf("Triangle: %f %f %f\n", tf_0.k[0], tf_0.k[1], tf_0.k[2]);
    printf("          %f %f %f\n", tf_1.k[0], tf_1.k[1], tf_1.k[2]);
    printf("          %f %f %f\n\n", tf_2.k[0], tf_2.k[1], tf_2.k[2]);
    printf("Edge:     %f %f %f\n", ef_0.k[0], ef_0.k[1], ef_0.k[2]);
    printf("          %f %f %f\n\n", ef_1.k[0], ef_1.k[1], ef_1.k[2]);
    
    if(pierces){
	printf("pierces %s triangle:  %f %f    q=%f r=%f \n",
	       (pierce_inside)?"inside":"outside",
	       pierce_point_f.k[0],
	       pierce_point_f.k[1],
	       q,
	       r);
    } else {
	printf("Edge is parallel.\n");
    }
    printf("\n");
    printf("edge startpoint q=%f r=%f\n", q_edge_start, r_edge_start);
    printf("                s=%f\n", IVP_CLS.calc_s_val(triangle_e0,&e_0, tri_cache));
    printf("     endpoint   q=%f r=%f\n", q_edge_end, r_edge_end);
    printf("                s=%f\n\n", IVP_CLS.calc_s_val(triangle_e0,&e_1, tri_cache));

    printf("tri_edge(K) to edge(L): slK=%f skL=%f\n",
	   kkr.checks_K[0] / (kkr.checks_K[0] + kkr.checks_K[1]) ,
	   kkr.checks_L[0] / (kkr.checks_L[0] + kkr.checks_L[1])
	   );
    return 0; // for debugger
}
#endif




///////////////////////////////////
///////////////////////////////////


#if 0

IVP_DOUBLE IVP_Mindist::calc_qlen_PF(IVP_Compact_Edge *P, IVP_Compact_Edge *F, IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_F)
{
    IVP_U_Point *wP;
    wP = P->give_world_coords_AT(m_cache_P);

    IVP_DOUBLE q, r;
    F->calc_qr_vals(P, &q, &r, m_cache_F, m_cache_P);

    if ( q >= 0.0f && r >= 0.0f && r+q<=1.0f ){
	// inside triangle
	IVP_U_Hesse hesse_world;
	F->calc_hesse_AT(m_cache_F, &hesse_world);
	IVP_DOUBLE now_len = hesse_world.get_dist( wP );
	return now_len*now_len; // we want quad dists	
    }
    
    // outside triangle: take best edge instead
    IVP_DOUBLE checks[3];
    checks[0] = r;// go to direction if checks[j] < 0
    checks[1] = q;
    checks[2] = 1.0f - q - r;
    
    IVP_Compact_Edge *e;
    int j;
    IVP_DOUBLE minlen = P_DOUBLE_MAX;
    IVP_Compact_Edge *min_edge = 0;
    for (e=F,j=0;j<3;e=e->get_next(),j++){
	IVP_DOUBLE len = calc_qlen_PK( P, e, m_cache_P, m_cache_F);
	if(len < minlen){
	    min_edge = e;
	    minlen = len;
	}
    }
    IVP_ASSERT(min_edge); // what happened !?

    return minlen;
}




//////////////////////////////////
//////////////////////////////////

#ifdef DEBUG_CHECK_LEN
IVP_RETURN_TYPE IVP_Mindist::validate_termination_len(IVP_DOUBLE now_len)
{    
    IVP_DOUBLE t_len = IVP_Mindist::termination_len;

    // for debugger
    IVP_DOUBLE real_now_len = sqrt( now_len );
    IVP_DOUBLE real_term_len = sqrt( t_len );
    diff = real_now_len - real_term_len;
    if(now_len - IVP_MIN_TERMINATION_QDLEN > t_len){ // @@@ test for '=='
	CORE;
    }
    if(now_len - IVP_MIN_TERMINATION_QDLEN_EPS > t_len){ // @@@ test for '=='
	printf("Termination Len Violation: Epsilon Problem?\n");
	return IVP_FAULT;
    }
    if(now_len < ivp_mindist_settings.real_coll_dist * ivp_mindist_settings.real_coll_dist){
	CORE;
	return IVP_FAULT;
    }

    IVP_Mindist::termination_len = now_len;
    return IVP_OK;
}

#if 0
IVP_RETURN_TYPE IVP_Mindist::check_len_PP(IVP_Compact_Ledge *A, IVP_Compact_Edge *B, IVP_Cache_Ledge_Point *m_cache_A, IVP_Cache_Ledge_Point *m_cache_B)
{
    // termination function test
    IVP_DOUBLE now_len = calc_qlen_PP(A, B, m_cache_A, m_cache_B);
    return validate_termination_len(now_len);
}
#endif

IVP_RETURN_TYPE IVP_Mindist::check_len_PF(IVP_Compact_Edge *P, IVP_Compact_Edge *F, IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_F)
{
    // termination function test
    IVP_DOUBLE now_len = calc_qlen_PF(P, F, m_cache_P, m_cache_F);

    return validate_termination_len(now_len);
}

IVP_RETURN_TYPE IVP_Mindist::check_len_PK(IVP_Compact_Edge *P, IVP_Compact_Edge *K, IVP_Cache_Ledge_Point *m_cache_P, IVP_Cache_Ledge_Point *m_cache_K)
{
    // termination function test
    IVP_DOUBLE now_len = calc_qlen_PK(P, K, m_cache_P, m_cache_K);
    return validate_termination_len(now_len);
}

IVP_RETURN_TYPE IVP_Mindist::check_len_KK(IVP_Compact_Edge *K, IVP_Compact_Edge *L, IVP_Cache_Ledge_Point *m_cache_K, IVP_Cache_Ledge_Point *m_cache_L)
{
    // termination function test
    IVP_DOUBLE now_len = calc_qlen_KK(K, L, m_cache_K, m_cache_L);

    return validate_termination_len(now_len);
}

IVP_RETURN_TYPE IVP_Mindist::check_len_FF(IVP_Compact_Edge *F, IVP_Compact_Edge *G, IVP_Cache_Ledge_Point *m_cache_F, IVP_Cache_Ledge_Point *m_cache_G)
{
    // termination function test
  
    IVP_DOUBLE now_len = IVP_CLS.p_optimize_FF(F, G, m_cache_F, m_cache_G, P_DOUBLE_MAX);

    return validate_termination_len(now_len);
}
#endif

#endif






