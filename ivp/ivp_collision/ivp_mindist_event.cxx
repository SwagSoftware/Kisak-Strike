// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


// IVP_EXPORT_PRIVATE

#include <ivp_physics.hxx>
#include <ivp_debug_manager.hxx>

#include <ivp_mindist_intern.hxx>
#include <ivp_mindist_event.hxx>

#include <ivu_hash.hxx>


#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>
#include <ivp_compact_ledge_solver.hxx>

#include <ivp_3d_solver.hxx>

#define P_MAX(a,b)(((a)>(b))?(a):(b))
#define P_MIN(a,b)(((a)<(b))?(a):(b))



class IVP_3D_Solver_PF_COLL: public IVP_3D_Solver {
public:
    // object A
    IVP_U_Point point_object;

    IVP_U_Point hesse_of_area_object;
    IVP_U_Point point_of_area_object;
    // object B    
    
    IVP_DOUBLE get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c);
};

IVP_DOUBLE IVP_3D_Solver_PF_COLL::get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c){
    IVP_3D_Solver_PF_COLL *t = this;
    IVP_U_Point point_world;
    A_w_f_c->vmult4(&t->point_object,&point_world);

    IVP_U_Point hesse_of_area_world, point_of_area_world;
    B_w_f_c->inline_vmult4(&t->point_of_area_object, &point_of_area_world);
    B_w_f_c->inline_vmult3(&t->hesse_of_area_object, &hesse_of_area_world);

    IVP_U_Point point_minus_area_world;
    point_minus_area_world.subtract(&point_world, &point_of_area_world);
    IVP_DOUBLE res = point_minus_area_world.dot_product(&hesse_of_area_world);
    return res;
}

class IVP_3D_Solver_VEC_PARALLEL_AREA: public IVP_3D_Solver {
public:
    // object A
    IVP_U_Point vec_object;

    // object B
    IVP_U_Point hesse_of_area_object;
    IVP_DOUBLE get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c);
};

IVP_DOUBLE IVP_3D_Solver_VEC_PARALLEL_AREA::get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c){
    IVP_3D_Solver_VEC_PARALLEL_AREA *t = this;
    IVP_U_Point vec_world;
    A_w_f_c->vmult3(&t->vec_object,&vec_world);

    IVP_U_Point hesse_of_area_world;
    B_w_f_c->vmult3(&t->hesse_of_area_object, &hesse_of_area_world);

    IVP_DOUBLE res = vec_world.dot_product(&hesse_of_area_world);
    return res;
}


class IVP_3D_Solver_DISTANCE_OF_TWO_POINTS: public IVP_3D_Solver {
public:
    // object A
    IVP_U_Point A_object;
    IVP_U_Point B_object;
    IVP_U_Point normized_direction_world_at_t0;

    IVP_DOUBLE get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c);
};

IVP_DOUBLE IVP_3D_Solver_DISTANCE_OF_TWO_POINTS::get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c){
    IVP_3D_Solver_DISTANCE_OF_TWO_POINTS *t = this;
    
    IVP_U_Point A_world;
    IVP_U_Point B_world;
    A_w_f_c->vmult4(&t->A_object,&A_world);
    B_w_f_c->vmult4(&t->B_object,&B_world);
    IVP_U_Point dir_world;
    dir_world.subtract(&B_world, &A_world);
    IVP_DOUBLE res = dir_world.quad_length();

    const IVP_DOUBLE linear_factor = 1.2f;
    // check linear
    IVP_DOUBLE linear_dist = dir_world.dot_product(&normized_direction_world_at_t0) * linear_factor;

    if ( linear_dist * IVP_Inline_Math::fabsd(linear_dist) < res  ){
	return linear_dist;
    }
    
    return IVP_Inline_Math::sqrtd(res);	// bad, but allows good estimates for first deviation
}

class IVP_3D_Solver_S_VALS: public IVP_3D_Solver {
public:
    // object A is Point
    IVP_U_Point P_object; 
    // object B is edge
    IVP_U_Point K_object;
    IVP_U_Point K_vec_object;	// normized K_vec
    IVP_DOUBLE get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c);
};

IVP_DOUBLE IVP_3D_Solver_S_VALS::get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c){
    IVP_3D_Solver_S_VALS *t = this;
    
    IVP_U_Point P_world;
    IVP_U_Point K_world;
    IVP_U_Point K_vec_world;
    A_w_f_c->vmult4(&t->P_object,&P_world);
    B_w_f_c->vmult4(&t->K_object,&K_world);
    B_w_f_c->vmult3(&t->K_vec_object,&K_vec_world);

    IVP_U_Point PK;
    PK.subtract(&K_world,&P_world);
    IVP_DOUBLE res = PK.dot_product(&K_vec_world);
    return res;
}

class IVP_3D_Solver_PK_KK: public IVP_3D_Solver {
public:
    // object A is Point
    IVP_U_Point P_object;
    IVP_U_Point P_ne_vec_object;		// neighbour edge of P
    // object B is edge
    IVP_U_Point K_object;
    IVP_U_Point K_vec_object;	// normized K_vec
    IVP_DOUBLE get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c);
};


// when does a PK case lead to KK case
// idea: build the Lot from P to K
// and check angle between Lot and neighbouring edges of P
// Note: to minimize errors, the lot is constructed using calc_hesse twice instead of calc_s_val

IVP_DOUBLE IVP_3D_Solver_PK_KK::get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c){
    IVP_3D_Solver_PK_KK *t = this;
    
    IVP_U_Point P_world;
    IVP_U_Point P_ne_vec_world;
    IVP_U_Point K_world;
    IVP_U_Point K_vec_world;
    A_w_f_c->vmult4(&t->P_object,&P_world);
    A_w_f_c->vmult3(&t->P_ne_vec_object,&P_ne_vec_world);
    B_w_f_c->vmult4(&t->K_object,&K_world);
    B_w_f_c->vmult3(&t->K_vec_object,&K_vec_world);

    IVP_U_Point KP;
    KP.subtract(&P_world,&K_world);

    IVP_U_Point Lot;
    Lot.inline_calc_cross_product(&K_vec_world, &KP);
    Lot.inline_calc_cross_product_and_normize(&Lot, &K_vec_world);
    IVP_DOUBLE res = Lot.dot_product(&P_ne_vec_world);
    
    return res;
}

/* check the distance between point and edge
 * as this distance never gets negative, check the distance
 * to a virtual plane too ( minus plane_offset )
 * and return the minimum of both distances */
class IVP_3D_Solver_PK_COLL: public IVP_3D_Solver {
public:
    IVP_DOUBLE plane_offset;		// negative offset for plane check
    // object A is Point
    IVP_U_Float_Point P_object; 
    // object B is edge
    IVP_U_Float_Point K_object;
    IVP_U_Point K_vec_object;	// normized K_vec
    IVP_U_Point K_Lot_object;	// a normized vertical direction
    IVP_DOUBLE get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c);
};

IVP_DOUBLE IVP_3D_Solver_PK_COLL::get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c){
    IVP_3D_Solver_PK_COLL *t = this;
    
    IVP_U_Point P_world;
    IVP_U_Point K_world;
    IVP_U_Point K_vec_world;	// normized direction
    IVP_U_Point K_Lot_world;
    A_w_f_c->vmult4(&t->P_object,&P_world);
    B_w_f_c->vmult4(&t->K_object,&K_world);
    B_w_f_c->vmult3(&t->K_vec_object,&K_vec_world);
    B_w_f_c->vmult3(&t->K_Lot_object,&K_Lot_world);
    
    IVP_U_Point PK;
    PK.subtract(&K_world,&P_world);
    IVP_U_Point H;
    H.calc_cross_product( &PK, &K_vec_world);
    IVP_DOUBLE res = H.real_length();
    // check distance to plane two
    IVP_DOUBLE res2 = H.dot_product(&K_Lot_world) + plane_offset;
    if (res2 < res) return res2;
    return res;
}

class IVP_3D_Solver_PF_NPF: public IVP_3D_Solver {
public:
    // object A
    IVP_U_Point vec_object;

    // object B
    IVP_U_Point hesse_of_area_object;
    IVP_DOUBLE get_value(IVP_U_Matrix *A_w_f_c, IVP_U_Matrix *B_w_f_c);

};

IVP_DOUBLE IVP_3D_Solver_PF_NPF::get_value(IVP_U_Matrix *A_w_f_c,IVP_U_Matrix *B_w_f_c){
    IVP_U_Point hesse_world;
    IVP_U_Point hesse_other_object;		// fast normized

    B_w_f_c->vmult3(&hesse_of_area_object, &hesse_world);
    A_w_f_c->vimult3(&hesse_world,&hesse_other_object);
    
    IVP_DOUBLE res = hesse_other_object.dot_product(&vec_object);
    return res;
}

void IVP_Mindist_Event_Solver::calc_next_event_PF(const IVP_Compact_Edge *P,const  IVP_Compact_Edge *F,
						    IVP_Cache_Ledge_Point *m_cache_P,
						    IVP_Cache_Ledge_Point *m_cache_F)
{
    // returns the delta time till the next event
    // if real_event_occurred is false, no event until max_delta_time occurred 
    
    /** detect collision time, first **/

    IVP_U_Matrix_Cache cache_P(m_cache_P->clp_cache_object);
    IVP_U_Matrix_Cache cache_F(m_cache_F->clp_cache_object);

    IVP_DOUBLE max_rot_speed = cache_P.core->abs_omega + cache_F.core->abs_omega;
    event_time_out =  t_max;	// flag, set after next psi -> no event within this PSI

    IVP_3D_Solver_PF_COLL pf_coll_solver;

    {
	pf_coll_solver.point_object.set(IVP_CLS.give_object_coords(P,m_cache_P));
	pf_coll_solver.point_of_area_object.set( IVP_CLS.give_object_coords(F,m_cache_F));
	
	// pf_coll_solver.hesse_of_area_object = F->get_triangle()->tmp.gen.hesse;
	IVP_CLS.calc_hesse_vec_object_not_normized(F, m_cache_F->get_compact_ledge(), &pf_coll_solver.hesse_of_area_object);
	pf_coll_solver.hesse_of_area_object.fast_normize();
	
	pf_coll_solver.set_max_deviation(this->max_coll_speed);
	
	// when does point P collide with surface F (no borders!) ?
	IVP_FLOAT radius = mindist->sum_extra_radius;
	IVP_DOUBLE sec_dist = mindist->get_coll_dist() + radius;	// security distance
	IVP_DOUBLE real_coll = ivp_mindist_settings.real_coll_dist + 0.5f * radius;
	IVP_DOUBLE now_len = mindist->len_numerator+radius;	
	IVP_BOOL result = pf_coll_solver.find_first_t_for_value_coll( sec_dist,real_coll,   t_now, event_time_out,
								    &cache_P, &cache_F, &now_len, &event_time_out );
	if (result){
	    event_type_out = IVP_COLL_PF_COLL;
	}
    }


    /** find next sector change **/
    IVP_3D_Solver_PF_NPF pf_npf_solver;
    
    
    // when are neighbor points of P nearer to F than P ?
    
    pf_npf_solver.hesse_of_area_object = pf_coll_solver.hesse_of_area_object;
    IVP_DOUBLE sec_dist;             // gradient, which indicates a possible new collision
    {
	IVP_DOUBLE real_dist = P_MIN( mindist->get_coll_dist(), mindist->len_numerator);
	real_dist += mindist->sum_extra_radius * 0.1f; // lazy sector change for extra radius
	sec_dist = -ivp_mindist_settings.mindist_change_force_dist *  m_cache_F->get_core()->inv_object_diameter;
	sec_dist *= real_dist;
	sec_dist /= mindist->get_coll_dist();
    }
    IVP_U_Point wHesseF;
    IVP_U_Point hesse_in_P_object;
    {
	m_cache_F->get_object_cache()->m_world_f_object.vmult3( &pf_npf_solver.hesse_of_area_object , &wHesseF);
	m_cache_P->get_object_cache()->m_world_f_object.vimult3(&wHesseF, &hesse_in_P_object);
    }
    
    pf_npf_solver.set_max_deviation(max_rot_speed + P_DOUBLE_EPS);
    IVP_DOUBLE max_grad_change = pf_npf_solver.max_deviation * (event_time_out - t_now);
    
    const IVP_U_Float_Point *p_object = IVP_CLS.give_object_coords(P,m_cache_P);
    for(const IVP_Compact_Edge *e = P->get_prev()->get_opposite(); 1; e=e->get_prev()->get_opposite()){
	const IVP_U_Float_Point *p_next_object = IVP_CLS.give_object_coords(e->get_next(),m_cache_P);
	IVP_U_Point vec_object;
	vec_object.subtract( p_next_object, p_object);
	IVP_DOUBLE grad = hesse_in_P_object.dot_product(&vec_object);
	IVP_DOUBLE qlen = vec_object.quad_length();
	IVP_DOUBLE ilen = IVP_Inline_Math::isqrt_float(qlen);
	grad *= ilen;	// search greatest gradient
	if ( grad < max_grad_change ){	// fast check
	    pf_npf_solver.vec_object.set_multiple(&vec_object ,ilen);
	    IVP_BOOL found_flag;
	    found_flag = pf_npf_solver.find_first_t_for_value_max_dev( sec_dist,   t_now, event_time_out, 0,
								       &cache_P, &cache_F, &grad , &event_time_out );
	    if (found_flag){
		event_type_out = IVP_COLL_PF_NPF;
	    }	
	}
	if(e==P) break;
    }
}

void IVP_Mindist_Event_Solver::calc_next_event_BF(const  IVP_Compact_Edge *F,
						    IVP_Cache_Object *m_cache_B,
						    IVP_Cache_Ledge_Point *m_cache_F)
{
    // returns the delta time till the next event
    // if real_event_occurred is false, no event until max_delta_time occurred 
    
    /** detect collision time, first **/

    IVP_U_Matrix_Cache cache_P(m_cache_B);
    IVP_U_Matrix_Cache cache_F(m_cache_F->clp_cache_object);

    IVP_3D_Solver_PF_COLL pf_coll_solver;
    {
	pf_coll_solver.point_object.set_to_zero();
	pf_coll_solver.point_of_area_object.set( IVP_CLS.give_object_coords(F,m_cache_F));
	
	IVP_CLS.calc_hesse_vec_object_not_normized(F, m_cache_F->get_compact_ledge(), &pf_coll_solver.hesse_of_area_object);
	pf_coll_solver.hesse_of_area_object.fast_normize();
	pf_coll_solver.set_max_deviation(this->max_coll_speed);
	
	// when does point P collide with surface F (no borders!) ?
	IVP_FLOAT radius = mindist->sum_extra_radius;
	IVP_DOUBLE sec_dist = mindist->get_coll_dist() + radius;	// security distance
	IVP_DOUBLE real_coll = ivp_mindist_settings.real_coll_dist + 0.5f * radius;  // allow penetration
	IVP_DOUBLE now_len = mindist->len_numerator + radius;

	//@@@ optimization for static reference object (linear movement of ball)
	IVP_BOOL result = pf_coll_solver.find_first_t_for_value_coll( sec_dist,real_coll,   t_now, t_max,
									&cache_P, &cache_F, &now_len, &event_time_out );
	if (result){
	    event_type_out = IVP_COLL_PF_COLL;
	}
    }
}


class IVP_3D_Solver_KK_COLL: public IVP_3D_Solver {
public:
    // object A
    IVP_U_Point k_point_object;	// a point of K
    IVP_U_Point k_vec_object;		// k direction

    IVP_U_Point l_point_object;
    IVP_U_Point l_vec_object;		// l direction

    IVP_DOUBLE side;		// direction of H

    // object B
    IVP_DOUBLE get_value(IVP_U_Matrix *K_w_f_c,IVP_U_Matrix *L_w_f_c);
};

IVP_DOUBLE IVP_3D_Solver_KK_COLL::get_value(IVP_U_Matrix *K_w_f_c,IVP_U_Matrix *L_w_f_c){
    IVP_U_Point k_vec_world;
    IVP_U_Point l_vec_world;
    
    IVP_U_Point k_point_world;
    IVP_U_Point l_point_world;

    K_w_f_c->vmult3(&k_vec_object,&k_vec_world);
    K_w_f_c->vmult4(&k_point_object,&k_point_world);
    
    L_w_f_c->vmult3(&l_vec_object,&l_vec_world);
    L_w_f_c->vmult4(&l_point_object,&l_point_world);

    IVP_U_Point H;
    H.calc_cross_product(&k_vec_world, &l_vec_world);	// unscaled normal

    IVP_DOUBLE res = H.dot_product(&k_point_world) - H.dot_product(&l_point_world);
    IVP_DOUBLE iH_len = IVP_Inline_Math::isqrt_float(H.quad_length());			// bad, but allows good estimate for first deviation
    res *= iH_len;
    return res * side;
}

class IVP_3D_Solver_KK_PARALLEL: public IVP_3D_Solver {
public:
    // object A
    IVP_U_Point k_vec_object;		// normized k direction
    IVP_U_Point l_vec_object;		// normized l direction

    // object B
    IVP_DOUBLE get_value(IVP_U_Matrix *K_w_f_c,IVP_U_Matrix *L_w_f_c);
};

IVP_DOUBLE IVP_3D_Solver_KK_PARALLEL::get_value(IVP_U_Matrix *K_w_f_c,IVP_U_Matrix *L_w_f_c){
    IVP_U_Point k_vec_world;
    IVP_U_Point l_vec_world;
    
    K_w_f_c->vmult3(&k_vec_object,&k_vec_world);
    L_w_f_c->vmult3(&l_vec_object,&l_vec_world);

    IVP_U_Point H;
    H.calc_cross_product(&k_vec_world, &l_vec_world);	// unscaled normal
    IVP_DOUBLE res = H.quad_length();	// see comment on calc max_deviation
    return res;
}



void IVP_Mindist_Event_Solver::calc_next_event_KK(const IVP_Compact_Edge *K,
				     const IVP_Compact_Edge *L,
				     IVP_Cache_Ledge_Point *m_cache_K,
				     IVP_Cache_Ledge_Point *m_cache_L)
{
    // returns the delta time till the next event
    // if real_event_occurred is false, no event until max_delta_time occurred 
    // requires non-'parallel' K and L
    IVP_U_Matrix_Cache cache_K(m_cache_K->clp_cache_object);
    IVP_U_Matrix_Cache cache_L(m_cache_L->clp_cache_object);
    

    IVP_3D_Solver_KK_COLL solver_kk_coll;
    IVP_DOUBLE sum_abs_omega;
    {
	IVP_Core *solver_K = m_cache_K->get_core();
	IVP_Core *solver_L = m_cache_L->get_core();
	sum_abs_omega = solver_K->abs_omega + solver_L->abs_omega;
    }

    event_time_out =  t_max;
    // which direction does H have ? (we want to know for which point to check collision with which surface)    
    int side;
    
    /** detect collision time, first **/    
    {
	const IVP_U_Float_Point *k =  IVP_CLS.give_object_coords( K, m_cache_K );
	const IVP_U_Float_Point *kn = IVP_CLS.give_object_coords( K->get_next(), m_cache_K );
	const IVP_U_Float_Point *l =  IVP_CLS.give_object_coords( L, m_cache_L );
	const IVP_U_Float_Point *ln = IVP_CLS.give_object_coords( L->get_next(), m_cache_L );
      
	solver_kk_coll.k_vec_object.subtract( kn, k ); // only rough estimate for KK parallel
	solver_kk_coll.k_vec_object.fast_normize();
	solver_kk_coll.l_vec_object.subtract( ln, l );
	solver_kk_coll.l_vec_object.fast_normize();

	solver_kk_coll.k_point_object.set(k);
	solver_kk_coll.l_point_object.set(l);
	solver_kk_coll.set_max_deviation(this->max_coll_speed);	// thats not correct, but ensures no collision !!!

	// check side
	{
	    IVP_U_Point k_vec_world_now, l_vec_world_now;
	    m_cache_L->clp_cache_object->m_world_f_object.vmult3(&solver_kk_coll.l_vec_object, &l_vec_world_now);
	    m_cache_K->clp_cache_object->m_world_f_object.vmult3(&solver_kk_coll.k_vec_object, &k_vec_world_now);

	    IVP_U_Point H;
	    H.calc_cross_product(&k_vec_world_now, &l_vec_world_now);
	    IVP_DOUBLE val  = -H.dot_product(&mindist->contact_plane);
#if 0	    
	    IVP_IF(1){
		IVP_U_Point l_startp_world_now, k_startp_world_now;
		IVP_CLS.give_world_coords_AT(L, m_cache_L,&l_startp_world_now);
		IVP_CLS.give_world_coords_AT(K, m_cache_K,&k_startp_world_now);

		IVP_U_Point diff_l_k;
		diff_l_k.subtract(&l_startp_world_now, &k_startp_world_now);
		IVP_DOUBLE val2 = diff_l_k.dot_product(&H);
		IVP_ASSERT( val * val2 >= 0);
	    }
#endif


	    if(val>0.0f){   //
		side = 0; // H points from K to L
		solver_kk_coll.side = -1.0f;
	    }else{
		side = 1; // H points from L to K
		solver_kk_coll.side = 1.0f;
	    }
	}
	
	IVP_DOUBLE found =
	    solver_kk_coll.find_first_t_for_value_coll(   mindist->get_coll_dist() ,ivp_mindist_settings.real_coll_dist, 
							   t_now, event_time_out,  
							   &cache_K, &cache_L, 0, &event_time_out );
	if (found){
	    event_type_out = IVP_COLL_KK_COLL;
	}
    }

 
    
    IVP_3D_Solver_KK_PARALLEL solver_kk_parallel;
    {
	solver_kk_parallel.k_vec_object.set(&solver_kk_coll.k_vec_object);
	solver_kk_parallel.l_vec_object.set(&solver_kk_coll.l_vec_object);
	// deviation == max omega * max_len_of_H
	IVP_DOUBLE dev = sum_abs_omega + sum_abs_omega; // as kk parallel calcs the quadratic length we have to adjust dev:
	// max (f^2') = max ( 2f * f') = max ( 2* 1 * f') 
	solver_kk_parallel.set_max_deviation(dev + P_DOUBLE_EPS);
	IVP_DOUBLE found =
	    solver_kk_parallel.find_first_t_for_value_max_dev( P_DOUBLE_EPS, 
							       t_now, event_time_out, 0, 
							       &cache_K, &cache_L , 0, &event_time_out );
	if (found){
	    event_type_out = IVP_COLL_KK_PARALLEL;
	}
    }

    
    /*** find next sector change ***/
    
    // when do PF-situations occur ?


    
    const IVP_Compact_Edge *F[4];	// areas to check
    F[0] = K->get_opposite();
    F[1] = K;
    F[2] = L->get_opposite();
    F[3] = L;

    IVP_U_Point *edge_object[4];		// edges to check, see P array at p_minimize_Leave_KK
    edge_object[0] = &solver_kk_coll.l_vec_object;		// normized
    edge_object[1] = &solver_kk_coll.l_vec_object;
    edge_object[2] = &solver_kk_coll.k_vec_object;
    edge_object[3] = &solver_kk_coll.k_vec_object;

    int reversed[4];			// edge_object is meant reverse
    reversed[0] = side;
    reversed[1] = 1-side;
    reversed[2] = side;
    reversed[3] = 1-side;

    IVP_U_Matrix_Cache *cache_tab[4];
    cache_tab[0] = &cache_L;
    cache_tab[1] = &cache_L;
    cache_tab[2] = &cache_K;
    cache_tab[3] = &cache_K;

    IVP_Cache_Ledge_Point *m_cache_tab[4];
    m_cache_tab[0] = m_cache_L;
    m_cache_tab[1] = m_cache_L;
    m_cache_tab[2] = m_cache_K;
    m_cache_tab[3] = m_cache_K;

    // check for new PF cases:
    // check scalarproduct of surface normal with dist_plane_normal

    IVP_3D_Solver_VEC_PARALLEL_AREA solver_KK_PF;
    // all lengths of hesse and edges are normized, so max_dev is easy 
    solver_KK_PF.set_max_deviation(sum_abs_omega + P_DOUBLE_EPS);
    
    for(int i=0; i<=3; i++){
	if ( !reversed[i]){
	    solver_KK_PF.vec_object.set_multiple( edge_object[i],-1.0f);		// cache_e
	}else{
	    solver_KK_PF.vec_object.set( edge_object[i]);
	}
	const IVP_Compact_Edge *f = F[i];
	IVP_CLS.calc_hesse_vec_object_not_normized(f, m_cache_tab[3-i]->get_compact_ledge(), &solver_KK_PF.hesse_of_area_object);
	solver_KK_PF.hesse_of_area_object.fast_normize();
	
	IVP_ASSERT(m_cache_tab[3-i]->get_compact_ledge() == f->get_compact_ledge());
	
	IVP_U_Matrix_Cache *cache_e = cache_tab[i];
	IVP_U_Matrix_Cache *cache_F = cache_tab[3-i];

	// when does point P 'collide' with plane F (no borders!) ? 
	// consider extensions of edge
       	IVP_DOUBLE sec_dist = -ivp_mindist_settings.mindist_change_force_dist * cache_e->core->inv_object_diameter;
	ivp_u_bool found = solver_KK_PF.find_first_t_for_value_max_dev( sec_dist, 
									t_now, event_time_out, 0, 
									cache_e, cache_F , 0, &event_time_out );
	if (found){
	    event_type_out = IVP_COLL_KK_PF;
	}
    }

}


/***************************** PP ******************************************/
/***************************** PP ******************************************/
/***************************** PP ******************************************/
void IVP_Mindist_Event_Solver::calc_next_event_PP(const IVP_Compact_Edge *P,
				     const IVP_Compact_Edge *P2,
				     IVP_Cache_Ledge_Point *m_cache_P,
				     IVP_Cache_Ledge_Point *m_cache_P2)
{
    // returns the time till the next event
    // if real_event_occurred is false, no event until max_delta_time occurred 
    IVP_U_Matrix_Cache cache_P(m_cache_P->clp_cache_object);
    IVP_U_Matrix_Cache cache_P2(m_cache_P2->clp_cache_object);
    

    IVP_DOUBLE sum_abs_omega;
    IVP_Core *solver_P = m_cache_P->get_core();
    IVP_Core *solver_P2 = m_cache_P2->get_core();
    sum_abs_omega = solver_P->abs_omega + solver_P2->abs_omega;

    event_time_out =  t_max;
    const IVP_U_Float_Point *points_object[2];
    const IVP_U_Float_Point *p_object = IVP_CLS.give_object_coords(P,m_cache_P);
    const IVP_U_Float_Point *p2_object = IVP_CLS.give_object_coords(P2,m_cache_P2);
    points_object[0] = p_object;
    points_object[1] = p2_object;
    // check collision of both points
    {
	IVP_3D_Solver_DISTANCE_OF_TWO_POINTS solver_pp_coll;
	solver_pp_coll.A_object.set( p_object );
	solver_pp_coll.B_object.set( p2_object);

	solver_pp_coll.normized_direction_world_at_t0.set_multiple( & mindist->contact_plane, -1);
#if 0
	IVP_IF(1){
	  IVP_U_Point check_dir;
	  const IVP_U_Point       p_world; IVP_CLS.give_world_coords_AT(P,m_cache_P,&p_world);
	  const IVP_U_Point       p2_world;IVP_CLS.give_world_coords_AT(P2,m_cache_P2,&p2_world);
	  check_dir.subtract(&p2_world, &p_world);
	  check_dir.normize();
	  IVP_DOUBLE val = -mindist->contact_plane.dot_product( &check_dir );
	  IVP_ASSERT(  IVP_Inline_Math::fabsd(val - 1.0f) < 0.0001f );
	}
#endif	
	solver_pp_coll.set_max_deviation(this->max_coll_speed);
	IVP_FLOAT radius = mindist->sum_extra_radius;
	ivp_u_bool found =
	    solver_pp_coll.find_first_t_for_value_coll( mindist->get_coll_dist() + radius,
							ivp_mindist_settings.real_coll_dist + radius * 0.5f, 
							t_now, t_max,  
							&cache_P, &cache_P2, 0, &event_time_out );
	if (found){
	    event_type_out = IVP_COLL_PP_COLL;
	}    
    }
    // check s vals for sector change
    {
	IVP_DOUBLE deviation[2];	// dev[0] when Point[0] is point and Poit[1] is edge
	IVP_DOUBLE neg_maximal_safe_s_value;	// needed to calc exact sector change value
	{ // deviation calculation 
	    // deviation for s:
	    // speed P + speed P2 + speed of projection (= abs_omega_z * dist P P2 )
	    IVP_DOUBLE max_dist_PP = mindist->get_length();
	    max_dist_PP += this->worst_case_speed * (event_time_out-t_now);
	    
	    IVP_DOUBLE speed_P = p_object->fast_real_length() * solver_P->abs_omega + solver_P->current_speed;
	    IVP_DOUBLE speed_P2 = p2_object->fast_real_length() * solver_P2->abs_omega + solver_P2->current_speed;
	    deviation[0] = speed_P + speed_P2 + max_dist_PP * solver_P2->abs_omega;
	    deviation[1] = speed_P + speed_P2 + max_dist_PP * solver_P->abs_omega;
	    
	    IVP_DOUBLE object_radius = P_MAX(m_cache_P->get_core()->upper_limit_radius, m_cache_P2->get_core()->upper_limit_radius);
	    IVP_DOUBLE quad_coll_dist = mindist->get_coll_dist();
	    quad_coll_dist *= quad_coll_dist;
	    neg_maximal_safe_s_value = - P_MIN(max_dist_PP*max_dist_PP, quad_coll_dist) * 0.5f / object_radius ;
	}

	IVP_U_Matrix_Cache *cache[2];
	cache[0] = &cache_P;
	cache[1] = &cache_P2;

	IVP_Cache_Ledge_Point *mcache[2];
	mcache[0] = m_cache_P;
	mcache[1] = m_cache_P2;
	
	const IVP_Compact_Edge *Point[2];
	Point[0] = P;
	Point[1] = P2;

	
	for (int j=0;j<2;j++){
	  //	    IVP_Compact_Edge *point = Point[j];
	    const IVP_Compact_Edge *point2 = Point[1-j];

	    const IVP_U_Float_Point *p0 = points_object[j];
	    const IVP_U_Float_Point *p1 = points_object[1-j];

	    IVP_3D_Solver_S_VALS solver_pp_pk;
	    solver_pp_pk.P_object.set(p0);
	    solver_pp_pk.K_object.set(p1);
	    solver_pp_pk.set_max_deviation(deviation[j]);

	    for(const IVP_Compact_Edge *e = point2->get_prev()->get_opposite(); 1; e=e->get_prev()->get_opposite()){
	      const IVP_U_Float_Point *epn = IVP_CLS.give_object_coords(e->get_next(), mcache[1-j]);
	      
		solver_pp_pk.K_vec_object.subtract( epn,  p1 );
		IVP_DOUBLE qlen = solver_pp_pk.K_vec_object.quad_length();
		IVP_DOUBLE inv_len = IVP_Inline_Math::isqrt_float(qlen);
		solver_pp_pk.K_vec_object.mult(inv_len);

		IVP_DOUBLE sec_dist = neg_maximal_safe_s_value  * qlen * inv_len;

		if (	solver_pp_pk.find_first_t_for_value_max_dev( sec_dist,   t_now, event_time_out, 0, 
								     cache[j], cache[1-j], 0 , &event_time_out ))
		{
		    event_type_out = IVP_COLL_PP_PK;
		}	
		if(e==point2) break;
	    }
	}
    }
}


/***************************** PP ******************************************/
/***************************** PP ******************************************/
/***************************** PP ******************************************/
void IVP_Mindist_Event_Solver::calc_next_event_BP(IVP_Ball * ball,const  IVP_Compact_Edge *P2,
						    IVP_Cache_Object *m_cache_B,
						    IVP_Cache_Ledge_Point *m_cache_P2)
{
    // returns the time till the next event
    // if real_event_occurred is false, no event until max_delta_time occurred 
    IVP_U_Matrix_Cache cache_P(m_cache_B);
    IVP_U_Matrix_Cache cache_P2(m_cache_P2->clp_cache_object);
    
    IVP_DOUBLE sum_abs_omega;
    IVP_Core *solver_P = ball->get_core();
    IVP_Core *solver_P2 = m_cache_P2->get_core();
    sum_abs_omega = solver_P->abs_omega + solver_P2->abs_omega;

    event_time_out =  t_max;

    IVP_U_Point p_object; p_object.set_to_zero();
    const IVP_U_Float_Point *p2_object = IVP_CLS.give_object_coords(P2,m_cache_P2);

    
    // check collision of both points
    {
	IVP_3D_Solver_DISTANCE_OF_TWO_POINTS solver_pp_coll;
	solver_pp_coll.A_object.set( &p_object );
	solver_pp_coll.B_object.set( p2_object);

	solver_pp_coll.normized_direction_world_at_t0.set_multiple( &mindist->contact_plane, -1);
#if 0	
	IVP_IF(1){
	  IVP_U_Point check_dir;
	  const IVP_U_Point       *p_world  =  m_cache_B->m_world_f_object.get_position();
	  const IVP_U_Point       p2_world; IVP_CLS.give_world_coords_AT(P2,m_cache_P2,&p2_world);
	  check_dir.subtract(&p2_world, p_world);
	  check_dir.normize();
	  IVP_DOUBLE val = -mindist->contact_plane.dot_product( &check_dir );
	  IVP_ASSERT(  IVP_Inline_Math::fabsd(val - 1.0f) < 0.0001f );
	}
#endif
	
	solver_pp_coll.set_max_deviation(this->max_coll_speed);
	IVP_FLOAT radius = mindist->sum_extra_radius;
	ivp_u_bool found =
	    solver_pp_coll.find_first_t_for_value_coll( mindist->get_coll_dist() + radius,
							ivp_mindist_settings.real_coll_dist + radius * 0.5f, 
							t_now, t_max,  
							&cache_P, &cache_P2, 0, &event_time_out );
	    if (found){
		event_type_out = IVP_COLL_PP_COLL;
	    }    
    }
    
    // check s vals for sector change
    {
	IVP_DOUBLE deviation;	// dev[0] when Point[0] is point and Poit[1] is edge
	IVP_DOUBLE neg_maximal_safe_s_value;	// needed to calc exact sector change value
	{ // deviation calculation 
	    // deviation for s:
	    // speed P + speed P2 + speed of projection (= abs_omega_z * dist P P2 )
	    IVP_DOUBLE max_dist_PP = mindist->get_length();
	    max_dist_PP += this->worst_case_speed * (event_time_out-t_now);
	    
	    IVP_DOUBLE speed_P = solver_P->current_speed;
	    IVP_DOUBLE speed_P2 = p2_object->fast_real_length() * solver_P2->abs_omega + solver_P2->current_speed;
	    deviation = speed_P + speed_P2 + max_dist_PP * solver_P2->abs_omega;
	    
	    IVP_DOUBLE inv_object_diameter = m_cache_P2->get_core()->inv_object_diameter;
	    IVP_DOUBLE quad_coll_dist = mindist->get_coll_dist();
	    quad_coll_dist *= quad_coll_dist;
	    neg_maximal_safe_s_value = - P_MIN(max_dist_PP*max_dist_PP, quad_coll_dist) * inv_object_diameter;
	}

	{
	    IVP_3D_Solver_S_VALS solver_pp_pk;
	    solver_pp_pk.P_object.set(&p_object);
	    solver_pp_pk.K_object.set(p2_object);
	    solver_pp_pk.set_max_deviation(deviation);

	    for(const IVP_Compact_Edge *e = P2->get_prev()->get_opposite(); 1; e=e->get_prev()->get_opposite()){
		const IVP_U_Float_Point *epn = IVP_CLS.give_object_coords(e->get_next(), m_cache_P2);
	      
		solver_pp_pk.K_vec_object.subtract( epn,  p2_object );
		IVP_DOUBLE qlen = solver_pp_pk.K_vec_object.quad_length();
		IVP_DOUBLE inv_len = IVP_Inline_Math::isqrt_float(qlen);
		solver_pp_pk.K_vec_object.mult(inv_len);
	      IVP_DOUBLE sec_dist = neg_maximal_safe_s_value * qlen * inv_len;

	      if (	solver_pp_pk.find_first_t_for_value_max_dev( sec_dist,   t_now, event_time_out, 0, 
								     &cache_P, &cache_P2, 0 , &event_time_out ))
		{
		    event_type_out = IVP_COLL_PP_PK;
		}	
		if(e==P2) break;
	    }
	}
    }
}


/***************************** BB ******************************************/
/***************************** BB ******************************************/
/***************************** BB ******************************************/
void IVP_Mindist_Event_Solver::calc_next_event_BB(IVP_Cache_Object *m_cache_A,
						    IVP_Cache_Object *m_cache_B)
{
    // returns the time till the next event
    // if real_event_occurred is false, no event until max_delta_time occurred 
    IVP_U_Matrix_Cache cache_A(m_cache_A);
    IVP_U_Matrix_Cache cache_B(m_cache_B);
    
    event_time_out =  t_max;

    const IVP_U_Point       *A_world  =  m_cache_A->m_world_f_object.get_position();
    const IVP_U_Point       *B_world  =  m_cache_B->m_world_f_object.get_position();
    
    // check collision of both points
    {
	// @@@ optimize !!!!!
	IVP_3D_Solver_DISTANCE_OF_TWO_POINTS solver_pp_coll;
	solver_pp_coll.A_object.set_to_zero();
	solver_pp_coll.B_object.set_to_zero();
	solver_pp_coll.normized_direction_world_at_t0.subtract(B_world, A_world);
	solver_pp_coll.normized_direction_world_at_t0.normize();
	
	solver_pp_coll.set_max_deviation(this->max_coll_speed);
	IVP_FLOAT radius = mindist->sum_extra_radius;
	ivp_u_bool found =
	    solver_pp_coll.find_first_t_for_value_coll( mindist->get_coll_dist() + radius,
							ivp_mindist_settings.real_coll_dist + radius * 0.5f, 
							t_now, t_max,  
							&cache_A, &cache_B, 0, &event_time_out );
	    if (found){
		event_type_out = IVP_COLL_PP_COLL;
	    }    
    }    
}



/***************************** PK ******************************************/
/***************************** PK ******************************************/
void IVP_Mindist_Event_Solver::calc_next_event_PK(const IVP_Compact_Edge *P,
				     const IVP_Compact_Edge *K,
				     IVP_Cache_Ledge_Point *m_cache_P,
				     IVP_Cache_Ledge_Point *m_cache_K)
{


    IVP_U_Matrix_Cache cache_P(m_cache_P->clp_cache_object);
    IVP_U_Matrix_Cache cache_K(m_cache_K->clp_cache_object);
    

    IVP_DOUBLE sum_abs_omega;
    IVP_Core *solver_P = m_cache_P->get_core();
    IVP_Core *solver_K = m_cache_K->get_core();
    sum_abs_omega = solver_P->abs_omega + solver_K->abs_omega;

    event_time_out =  t_max;

    const IVP_U_Float_Point *p_object = IVP_CLS.give_object_coords(P,m_cache_P);
    
    // check collision of point and edge
    {
	IVP_3D_Solver_PK_COLL solver_pk_coll;
	const IVP_U_Float_Point *k_object =  IVP_CLS.give_object_coords(K,m_cache_K);
	const IVP_U_Float_Point *kn_object =  IVP_CLS.give_object_coords(K->get_next(),m_cache_K);

	solver_pk_coll.P_object.set( p_object );
	solver_pk_coll.K_object.set(k_object);
	solver_pk_coll.K_vec_object.subtract( kn_object,  k_object);
	solver_pk_coll.K_vec_object.fast_normize();
	solver_pk_coll.set_max_deviation(this->max_coll_speed);
	IVP_FLOAT radius = mindist->sum_extra_radius;
	solver_pk_coll.plane_offset = 0.5f * (mindist->get_coll_dist() + radius);

	// find initial virtual collision plane
	{
	    IVP_U_Point P_world;
	    IVP_U_Point K_world;
	    IVP_U_Point K_vec_world;	// normized direction
	    m_cache_P->get_object_cache()->transform_position_to_world_coords( &solver_pk_coll.P_object, &P_world);
	    m_cache_K->get_object_cache()->transform_position_to_world_coords( &solver_pk_coll.K_object, &K_world);
	    m_cache_K->get_object_cache()->transform_vector_to_world_coords( &solver_pk_coll.K_vec_object, &K_vec_world);

	    IVP_U_Point PK;
	    PK.subtract(&K_world,&P_world);
	    IVP_U_Point H;
	    H.calc_cross_product( &PK, &K_vec_world);

	    m_cache_K->get_object_cache()->transform_vector_to_object_coords( &H, &solver_pk_coll.K_Lot_object);
	    solver_pk_coll.K_Lot_object.normize();
	}
	
	ivp_u_bool found =
	    solver_pk_coll.find_first_t_for_value_coll( mindist->get_coll_dist() + radius,
							ivp_mindist_settings.real_coll_dist + radius * 0.9f, 
							t_now, t_max,  
							&cache_P, &cache_K, 0, &event_time_out );
	    if (found){
		event_type_out = IVP_COLL_PK_COLL;
	    }    
    }


    /***** find next sector change *****/

    /** first: when does case 'P over F0/F1 of K' occur? **/
    
    // both areas F0/F1
    {
	IVP_3D_Solver_PF_COLL solver_pk_pf;
	solver_pk_pf.set_max_deviation(this->worst_case_speed);	// when does point P move through lot area

	solver_pk_pf.point_object.set(p_object);
	
	const IVP_Compact_Edge *FF[2];
	FF[0] = K;
	FF[1] = K->get_opposite();
    
	int i;
	for(i=0; i<=1; i++){
	    // Generation of  collision surface for PF sector  lot = (hesse) x edge 0
	    const IVP_Compact_Edge *F = FF[i]; // area to consider
	    IVP_U_Point v0;
	    const IVP_U_Float_Point *f_object = IVP_CLS.give_object_coords(F,m_cache_K);
	    const IVP_U_Float_Point *fn_object = IVP_CLS.give_object_coords(F->get_next(),m_cache_K);
	    const IVP_U_Float_Point *fp_object = IVP_CLS.give_object_coords(F->get_prev(),m_cache_K);
	    v0.subtract( fn_object, f_object );// @@@ already calculated	    
	    IVP_U_Point fhesse_vec;
	    fhesse_vec.inline_set_vert_to_area_defined_by_three_points(f_object, fp_object, fn_object);
	    solver_pk_pf.hesse_of_area_object.calc_cross_product(&v0, &fhesse_vec);
	    solver_pk_pf.hesse_of_area_object.normize();	// @@@ fast normize 
	    
	    solver_pk_pf.point_of_area_object.set(f_object);
	    
	    IVP_DOUBLE sec_dist = -ivp_mindist_settings.mindist_change_force_dist;
	    ivp_u_bool found_flag =
		solver_pk_pf.find_first_t_for_value_max_dev( sec_dist,   t_now, event_time_out, 0, 
							     &cache_P, &cache_K, 0 , &event_time_out );
	    if (found_flag){
		event_type_out = IVP_COLL_PK_PF;
	    }	
	}
    }
    
    // check s vals for sector change

    // idea: project P on K -> Q
    // calc s_values of Q on neighbouring edges of P
    // advantage: max_deviation can be calculated

    /** second: when does any neighbor edge of P lead to a KK case? **/
    IVP_DOUBLE dist_PK_now = mindist->get_length();
    IVP_DOUBLE max_dev;
    {
	// deviation for s:
	// s is sin(  angle( Lot(P,K), P_next ) )
	// s' < angle( Lot(P,K), P_next ) '
	// s' < |omega1| + |omega0| + Lot(P,K)'
	// Lot' < (|speed K| + |speed P|) / distance
	// Note: Lot' does not use |omegax|, because it is independent from rotation of objects
	
	    IVP_DOUBLE mindist_PK = dist_PK_now - this->worst_case_speed * (event_time_out-t_now);
	    if (mindist_PK < P_RES_EPS) mindist_PK = P_RES_EPS;

	    const IVP_U_Float_Point *spp= p_object;
	    IVP_DOUBLE speed_P = spp->fast_real_length() * solver_P->abs_omega + m_cache_P->get_core()->current_speed;
	    IVP_DOUBLE speed_K = m_cache_K->get_core()->max_surface_rot_speed * solver_K->abs_omega + m_cache_K->get_core()->current_speed;
	    max_dev = (speed_P + speed_K) / mindist_PK + sum_abs_omega;
    }

    {
	IVP_3D_Solver_PK_KK solver_pk_kk;	// calcs: - s_val * tmp.common.real_len
//	const IVP_U_Float_Point *p_object = IVP_CLS.give_object_coords(P,m_cache_P);
	const IVP_U_Float_Point *k_object = IVP_CLS.give_object_coords(K,m_cache_K);
	const IVP_U_Float_Point *kn_object = IVP_CLS.give_object_coords(K->get_next(),m_cache_K);

	solver_pk_kk.P_object.set(p_object);
	solver_pk_kk.K_object.set(k_object);
	solver_pk_kk.K_vec_object.subtract(kn_object,k_object);
	solver_pk_kk.K_vec_object.fast_normize();

	solver_pk_kk.set_max_deviation(max_dev);
	IVP_DOUBLE dist;
	IVP_DOUBLE coll_dist = mindist->get_coll_dist();
	if (dist_PK_now < coll_dist){
	    dist = dist_PK_now;
	}else{
	    dist = coll_dist;
	}
	// exponentially go 
	IVP_DOUBLE sec_dist = -0.3f * dist * (m_cache_K->get_core()->inv_object_diameter * 2.0f);
	
	for(const IVP_Compact_Edge *e=P->get_prev()->get_opposite(); 1; e=e->get_prev()->get_opposite()){
	  const IVP_U_Float_Point *en_object = IVP_CLS.give_object_coords(e->get_next(),m_cache_P);
	  
	    solver_pk_kk.P_ne_vec_object.subtract( en_object, p_object);
	    solver_pk_kk.P_ne_vec_object.fast_normize();

	    ivp_u_bool found_flag =
		solver_pk_kk.find_first_t_for_value_max_dev( sec_dist,   t_now, event_time_out, 0, 
							     &cache_P, &cache_K, 0 , &event_time_out );
	    if (found_flag){
		event_type_out = IVP_COLL_PK_KK;
	    }	
	    if(e==P) break;
	}
    }
}

/***************************** PK ******************************************/
/***************************** PK ******************************************/
void IVP_Mindist_Event_Solver::calc_next_event_BK(IVP_Ball * ball,const  IVP_Compact_Edge *K,
						    IVP_Cache_Object *m_cache_B,
						    IVP_Cache_Ledge_Point *m_cache_K){


    IVP_U_Matrix_Cache cache_P(m_cache_B);
    IVP_U_Matrix_Cache cache_K(m_cache_K->clp_cache_object);
    
    IVP_DOUBLE sum_abs_omega;
    IVP_Core *solver_P = ball->get_core();
    IVP_Core *solver_K = m_cache_K->get_core();
    sum_abs_omega = solver_P->abs_omega + solver_K->abs_omega;

    event_time_out =  t_max;

    IVP_U_Point p_object; p_object.set_to_zero();
    
    // check collision of point and edge
    {
	IVP_3D_Solver_PK_COLL solver_pk_coll;
	const IVP_U_Float_Point *k_object =  IVP_CLS.give_object_coords(K,m_cache_K);
	const IVP_U_Float_Point *kn_object =  IVP_CLS.give_object_coords(K->get_next(),m_cache_K);

	solver_pk_coll.P_object.set( &p_object );
	solver_pk_coll.K_object.set( k_object);
	solver_pk_coll.K_vec_object.subtract( kn_object,  k_object);
	solver_pk_coll.K_vec_object.fast_normize();
	solver_pk_coll.set_max_deviation(this->max_coll_speed);
	IVP_FLOAT radius = mindist->sum_extra_radius;
	solver_pk_coll.plane_offset = 0.5f * ( mindist->get_coll_dist() + radius);

	// find initial virtual collision plane
	{
	    IVP_U_Point K_world;
	    IVP_U_Point K_vec_world;	// normized direction
	    IVP_U_Point *P_world = m_cache_B->m_world_f_object.get_position();

	    m_cache_K->get_object_cache()->transform_position_to_world_coords( &solver_pk_coll.K_object,  &K_world);
	    m_cache_K->get_object_cache()->transform_vector_to_world_coords(   &solver_pk_coll.K_vec_object, &K_vec_world);

	    IVP_U_Point PK;
	    PK.subtract(&K_world,P_world);
	    IVP_U_Point H;
	    H.calc_cross_product( &PK, &K_vec_world);

	    m_cache_K->get_object_cache()->transform_vector_to_object_coords(&H, &solver_pk_coll.K_Lot_object);
	    solver_pk_coll.K_Lot_object.normize();
	}
	
	ivp_u_bool found =
	    solver_pk_coll.find_first_t_for_value_coll( mindist->get_coll_dist() + radius,
							ivp_mindist_settings.real_coll_dist + radius * 0.9f,
 							t_now, t_max,  
							&cache_P, &cache_K, 0, &event_time_out );
	    if (found){
		event_type_out = IVP_COLL_PK_COLL;
	    }    
    }


    /***** find next sector change *****/

    /** first: when does case 'P over F0/F1 of K' occur? **/
    
    // both areas F0/F1
    {
	IVP_3D_Solver_PF_COLL solver_pk_pf;
	IVP_ASSERT( worst_case_speed > max_coll_speed );
	solver_pk_pf.set_max_deviation(this->worst_case_speed);	// when does point P move through lot area

	solver_pk_pf.point_object.set(&p_object);
	
	const IVP_Compact_Edge *FF[2];
	FF[0] = K;
	FF[1] = K->get_opposite();
    
	int i;
	for(i=0; i<=1; i++){
	    // Generation of  collision surface for PF sector  lot = (hesse) x edge 0
	    const IVP_Compact_Edge *F = FF[i]; // area to consider
	    IVP_U_Point v0;
	    const IVP_U_Float_Point *f_object = IVP_CLS.give_object_coords(F,m_cache_K);
	    const IVP_U_Float_Point *fn_object = IVP_CLS.give_object_coords(F->get_next(),m_cache_K);
	    const IVP_U_Float_Point *fp_object = IVP_CLS.give_object_coords(F->get_prev(),m_cache_K);
	    v0.subtract( fn_object, f_object );// @@@ already calculated	    
	    IVP_U_Point fhesse_vec;
	    fhesse_vec.inline_set_vert_to_area_defined_by_three_points(f_object, fp_object, fn_object);
	    solver_pk_pf.hesse_of_area_object.calc_cross_product(&v0, &fhesse_vec);
	    solver_pk_pf.hesse_of_area_object.normize();	// @@@ fast normize 
	    
	    solver_pk_pf.point_of_area_object.set(f_object);
	    
	    IVP_DOUBLE sec_dist = -ivp_mindist_settings.mindist_change_force_dist;
	    ivp_u_bool found_flag =
		solver_pk_pf.find_first_t_for_value_max_dev( sec_dist,   t_now, event_time_out, 0, 
							     &cache_P, &cache_K, 0 , &event_time_out );
	    if (found_flag){
		event_type_out = IVP_COLL_PK_PF;
	    }	
	}
    }
    
}


void IVP_Mindist_Event_Solver::next_event_B_POLY(IVP_Mindist_Event_Solver *mim)
{    
    IVP_Synapse_Real *syn0;
    IVP_Synapse_Real *syn1;
    syn0 = mim->mindist->get_synapse(0);
    syn1 = mim->mindist->get_synapse(1);    

    const IVP_Compact_Edge *e1 = syn1->edge;
    IVP_Ball *ball = syn0->get_object()->to_ball();
    IVP_Cache_Object *m_cache_0 = ball->get_cache_object();
    IVP_Cache_Ledge_Point m_cache_1(syn1->get_object()->to_poly(),e1->get_compact_ledge());
    
    mim->event_type_out = IVP_COLL_NONE;
#ifdef DEBUG    
    m_cache_1.tmp.synapse = NULL;
#endif    

    
    switch(syn1->get_status()){  
    case IVP_ST_POINT:
	mim->calc_next_event_BP( ball, e1, m_cache_0, &m_cache_1);
	break;
    case IVP_ST_EDGE:
	mim->calc_next_event_BK( ball, e1, m_cache_0, &m_cache_1);
	break;
    case IVP_ST_TRIANGLE:
	mim->calc_next_event_BF( e1, m_cache_0, &m_cache_1);
	break;
    default:
	CORE;
    };

    m_cache_0->remove_reference();
    m_cache_1.remove_reference();

}

void IVP_Mindist_Event_Solver::next_event_BB(IVP_Mindist_Event_Solver *mim)
{    
    IVP_Synapse_Real *syn0;
    IVP_Synapse_Real *syn1;
    syn0 = mim->mindist->get_synapse(0);
    syn1 = mim->mindist->get_synapse(1);    

    IVP_Ball *ball0 = syn0->get_object()->to_ball();
    IVP_Ball *ball1 = syn1->get_object()->to_ball();
    IVP_Cache_Object *m_cache_0 = ball0->get_cache_object();
    IVP_Cache_Object *m_cache_1 = ball1->get_cache_object();
    
    mim->event_type_out = IVP_COLL_NONE;

    mim->calc_next_event_BB( m_cache_0, m_cache_1);

    m_cache_0->remove_reference();
    m_cache_1->remove_reference();
}



void IVP_Mindist_Event_Solver::next_event_default_poly_poly(IVP_Mindist_Event_Solver *mim)
{    
    IVP_Synapse_Real *syn0, *syn1;
    syn0 = mim->mindist->get_sorted_synapse(0);
    syn1 = mim->mindist->get_sorted_synapse(1);    

    const IVP_Compact_Edge *e0 = syn0->edge;
    const IVP_Compact_Edge *e1 = syn1->edge;
    
    IVP_Cache_Ledge_Point m_cache_0(syn0->get_object()->to_poly(),e0->get_compact_ledge()); // @@@ faster way? -OG
    IVP_Cache_Ledge_Point m_cache_1(syn1->get_object()->to_poly(),e1->get_compact_ledge());

#ifdef DEBUG    
    m_cache_0.tmp.synapse = NULL;	// remember order of synapses
    m_cache_1.tmp.synapse = NULL;
#endif    
    
    mim->event_type_out = IVP_COLL_NONE;

    
    switch(syn0->get_status()){
      case IVP_ST_POINT:{
	  switch(syn1->get_status()){  
	  case IVP_ST_POINT:{	
	      mim->calc_next_event_PP(e0, e1, &m_cache_0, &m_cache_1);
	      break;
	    }
	    case IVP_ST_EDGE:{
	      mim->calc_next_event_PK(e0, e1, &m_cache_0, &m_cache_1);
	      break;
	    }
	    case IVP_ST_TRIANGLE:{
	      mim->calc_next_event_PF(e0, e1, &m_cache_0, &m_cache_1);
	      break;
	    }
	    default:
		CORE;
	  }
	  break;
      };
      case IVP_ST_EDGE:{
	  switch(syn1->get_status()){  
	    case IVP_ST_EDGE:{
	      mim->calc_next_event_KK(e0, e1, &m_cache_0, &m_cache_1);
	      break;
	    }
	    default:
		CORE;
	  }
	  break;
      };
    default:
	CORE; // unknown synapse status
    }
    
    m_cache_0.remove_reference();
    m_cache_1.remove_reference();
}



void IVP_Mindist_Event_Solver::next_event_illegal(IVP_Mindist_Event_Solver *){
    // error: inconsistency between IVP_Mindist_Event_Solver and IVP_Mindist_Minimize_Solver
//lwss hack - change this to not break on non-debug builds. It goes off sometimes
#ifdef _DEBUG
    CORE;
#else
    fprintf(stderr, "next_event_illegal called!!!\n");
#endif
//lwss end
}

void (*IVP_Mindist_Event_Solver::mim_function_table[IVP_ST_MAX_LEGAL][IVP_ST_MAX_LEGAL])(IVP_Mindist_Event_Solver *mim);

void IVP_Mindist_Event_Solver::init_mim_function_table(){
    for (int i=0; i< IVP_ST_MAX_LEGAL; i++){
	for (int j=0; j< IVP_ST_MAX_LEGAL; j++){
	    mim_function_table[i][j] = next_event_illegal;
	}
    }
    mim_function_table[IVP_ST_POINT] [IVP_ST_POINT]    = next_event_default_poly_poly;
    mim_function_table[IVP_ST_POINT] [IVP_ST_EDGE]     = next_event_default_poly_poly;
    mim_function_table[IVP_ST_POINT] [IVP_ST_TRIANGLE] = next_event_default_poly_poly;
    mim_function_table[IVP_ST_POINT] [IVP_ST_BALL]     = next_event_illegal;

    mim_function_table[IVP_ST_EDGE] [IVP_ST_POINT]    = next_event_illegal;
    mim_function_table[IVP_ST_EDGE] [IVP_ST_EDGE]     = next_event_default_poly_poly;
    mim_function_table[IVP_ST_EDGE] [IVP_ST_TRIANGLE] = next_event_illegal;
    mim_function_table[IVP_ST_EDGE] [IVP_ST_BALL]     = next_event_illegal;

    mim_function_table[IVP_ST_TRIANGLE] [IVP_ST_POINT]    = next_event_illegal;
    mim_function_table[IVP_ST_TRIANGLE] [IVP_ST_EDGE]     = next_event_illegal;
    mim_function_table[IVP_ST_TRIANGLE] [IVP_ST_TRIANGLE] = next_event_illegal;
    mim_function_table[IVP_ST_TRIANGLE] [IVP_ST_BALL]     = next_event_illegal;

    mim_function_table[IVP_ST_BALL] [IVP_ST_POINT]    = next_event_B_POLY;
    mim_function_table[IVP_ST_BALL] [IVP_ST_EDGE]     = next_event_B_POLY;
    mim_function_table[IVP_ST_BALL] [IVP_ST_TRIANGLE] = next_event_B_POLY;
    mim_function_table[IVP_ST_BALL] [IVP_ST_BALL]     = next_event_BB;
}


