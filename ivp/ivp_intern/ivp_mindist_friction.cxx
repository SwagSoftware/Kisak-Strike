// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>
#include <ivp_debug_manager.hxx>

#include <ivp_material.hxx>
#include <ivp_hull_manager.hxx>
#include <ivp_mindist_intern.hxx>
#include "ivp_mindist_macros.hxx"
#include <ivp_friction.hxx>

#include <ivp_compact_ledge.hxx>

#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <ivu_memory.hxx>

// calculates:
//    diff_contact_vec
//    surf_normal
// 	contact_point_ws
//    gap_distance
//    span_friction_v[0]
//    q,r and friction_situation_went_invalid
void IVP_Contact_Point::p_calc_friction_qr_PF(const IVP_U_Point *pp,
						 const  IVP_Compact_Edge *F,
						 IVP_Cache_Ledge_Point *m_cache_F,
						 IVP_Impact_Solver_Long_Term *info,
						 IVP_U_Float_Point *diff_contact_vec)
{
    // Punkt/Flaeche
    // F ist aufgespannt durch Q, R (und S)
    // (Q=F, R=F->next, S=F->prev)
  IVP_USE(diff_contact_vec);
    info->contact_point_ws.set(pp);

    IVP_U_Point pp_Fos;
    m_cache_F->get_object_cache()->transform_position_to_object_coords(pp, &pp_Fos);

#if !defined(  IVP_USE_S_VALS_FOR_PRETENSION )
    if ( cp_status == IVP_CPBS_NEEDS_RECHECK) { // do checking
      this->cp_status = IVP_CPBS_NORMAL;
      IVP_Unscaled_QR_Result qr;
      IVP_CLS.calc_unscaled_qr_vals_F_space(m_cache_F->get_compact_ledge(), F, &pp_Fos, &qr);
      if (qr.is_outside()){
	  info->friction_is_broken = IVP_TRUE;
      }
    }
    { // calc surface normal
	IVP_U_Point wHesse_vecF_Fos;
	IVP_CLS.calc_hesse_vec_object_not_normized(F, m_cache_F->get_compact_ledge(), &wHesse_vecF_Fos);
	wHesse_vecF_Fos.mult(inv_triangle_det);
	//wHesse_vecF_Fos.fast_normize();
      
      IVP_U_Point hesse_vec_ws;
      m_cache_F->get_object_cache()->transform_vector_to_world_coords( &wHesse_vecF_Fos, &hesse_vec_ws);
    
      const IVP_U_Float_Point *F_O_Fos = IVP_CLS.give_object_coords(F,m_cache_F);
      const IVP_U_Float_Point *F_1_Fos = IVP_CLS.give_object_coords(F->get_next(),m_cache_F);
      last_gap_len = wHesse_vecF_Fos.dot_product( &pp_Fos ) -  wHesse_vecF_Fos.dot_product(F_O_Fos);
      info->surf_normal.set_multiple( &hesse_vec_ws, -1.0f);
      
      // calc span_friction_v
      // =+= replace span_friction_v by global pre_tension
      IVP_U_Float_Point qvec_Fos;
      qvec_Fos.subtract( F_1_Fos, F_O_Fos);
      m_cache_F->get_object_cache()->transform_vector_to_world_coords( &qvec_Fos, &info->span_friction_v[0]);
    }
    
#endif
#ifdef IVP_USE_S_VALS_FOR_PRETENSION
    IVP_U_Point tp;      IVP_CLS.give_world_coords_AT(F, m_cache_F,&tp);
    IVP_U_Point tp_next; IVP_CLS.give_world_coords_AT(F->get_next(), m_cache_F, &tp_next);
    IVP_U_Point tp_prev; IVP_CLS.give_world_coords_AT(F->get_prev(), m_cache_F, &tp_prev);
    
    IVP_U_Point R, Q, Pvec;
    Q.subtract(&tp, &tp_next);      
    R.subtract(&tp_prev, &tp_next);

    IVP_DOUBLE QQ = Q.quad_length();
    IVP_DOUBLE RR = R.quad_length();
    
    Pvec.subtract( pp, &tp_next);
    
    IVP_DOUBLE QR = R.dot_product(&Q);

    IVP_DOUBLE QQRR = QQ * RR;
    IVP_DOUBLE QRQR = QR * QR;
    IVP_DOUBLE Det = (QQRR - QRQR);

    IVP_DOUBLE sq = Pvec.dot_product(&Q);
    IVP_DOUBLE sr = Pvec.dot_product(&R);
    
    IVP_DOUBLE unscaled_q = ( RR * sq - sr * QR );
    IVP_DOUBLE unscaled_r = ( QQ * sr - sq * QR );

    if ( unscaled_q < 0.0f || unscaled_r < 0.0f || unscaled_r+unscaled_q > Det) {
	info->friction_situation_went_invalid = IVP_TRUE;
    }

    info->surf_normal.calc_cross_product(&Q,&R);   // @@@OS optimization: store surf_normal in CS coords, or at least 1.0f/sqrt(len) 
    info->surf_normal.fast_normize();


    IVP_DOUBLE iQ = sqrtf(1.0f / QQ );
    info->span_friction_v[0].set_multiple(&Q, iQ);

    info->gap_distance = info->surf_normal.dot_product(&tp) - info->surf_normal.dot_product(pp);


    IVP_DOUBLE iDet = 1.0f / Det;
    IVP_DOUBLE q = unscaled_q * iDet;
    IVP_DOUBLE r = unscaled_r * iDet;

    IVP_IF(1) {
	IVP_U_Point point_on_surf;
	point_on_surf.set(&tp_next);
	point_on_surf.add_multiple(&Q,q);
	point_on_surf.add_multiple(&R,r);
	IVP_U_Point dist_vec_world;
	dist_vec_world.subtract(&point_on_surf,pp);
	IVP_DOUBLE test_val = info->surf_normal.dot_product(&dist_vec_world);
	if( test_val < 0.0f ) {
	    printf("recalc_friction_s_point_surface wrong surf_normal %f should be >0\n",test_val);
	}
    }

    IVP_DOUBLE old_q = s_coords[0];
    IVP_DOUBLE old_r = s_coords[1];

    diff_contact_vec->set_multiple(&Q,old_q - q);
    diff_contact_vec->add_multiple(&R,old_r - r);

    
    s_coords[0] = q;
    s_coords[1] = r;
#endif
}

void IVP_Contact_Point::p_calc_friction_s_PP(const IVP_U_Point *pp0,const IVP_U_Point *pp1,IVP_Impact_Solver_Long_Term *info, IVP_U_Float_Point *diff_contact_vec) {
  IVP_USE(diff_contact_vec);
    IVP_U_Point dist_vec;
    dist_vec.subtract(pp1,pp0);
    last_gap_len = dist_vec.real_length_plus_normize();
    info->surf_normal.set(&dist_vec);

    IVP_U_Float_Point magic_world_random_vec;
    
    // surf_normal shall not point in same direction as magic_world_random_vec
    if( info->surf_normal.k[0] * info->surf_normal.k[0] < 0.9f) {
      magic_world_random_vec.set(1,0,0);
    }else{
      magic_world_random_vec.set(0,0,1);
    }
    info->span_friction_v[0].inline_calc_cross_product_and_normize(&info->surf_normal,&magic_world_random_vec);
    info->contact_point_ws.set(pp0);
#ifdef IVP_USE_S_VALS_FOR_PRETENSION    
    diff_contact_vec->set_to_zero();
#endif   
}

void IVP_Contact_Point::p_calc_friction_s_PK(const IVP_U_Point *pp, const IVP_Compact_Edge *K,
					    IVP_Cache_Ledge_Point *m_cache_K, IVP_Impact_Solver_Long_Term *info, IVP_U_Float_Point *diff_contact_vec)
{
    // if status indicates friction mode, point is always kept the same
  IVP_USE(diff_contact_vec);
    IVP_U_Point tp;      IVP_CLS.give_world_coords_AT(K, m_cache_K,&tp);
    IVP_U_Point tp_next; IVP_CLS.give_world_coords_AT(K->get_next(), m_cache_K, &tp_next);
    
    IVP_U_Float_Point vec1, vec2;
    vec1.subtract(&tp_next, &tp);
    vec2.subtract(pp, &tp);
    IVP_DOUBLE i_len = IVP_Inline_Math::isqrt_double( vec1.quad_length() );

    IVP_U_Float_Point vert_12;
    vert_12.inline_calc_cross_product( &vec1, &vec2 );
    IVP_DOUBLE distance = vert_12.real_length() * i_len;
    last_gap_len = distance;

    if(  distance * distance > P_DOUBLE_EPS ) {
	vert_12.set_multiple( &vert_12, i_len / distance );
    } else {
	vert_12.set(1.0f,0.0f,0.0f);
    }
    info->surf_normal.inline_calc_cross_product_and_normize(&vec1,&vert_12);
    info->contact_point_ws.set(pp);
    
    info->span_friction_v[0].set(&vert_12);
    IVP_IF(1) {
	IVP_DOUBLE len= info->span_friction_v[0].real_length();
	len=1.0f-len;
	if( IVP_Inline_Math::fabsd(len) > 0.001f) {
	    printf("span_friction_v_pk not normized\n");
	}	
    }

    if (cp_status == IVP_CPBS_NEEDS_RECHECK){
	this->cp_status = IVP_CPBS_NORMAL;
	IVP_DOUBLE iqlen =  i_len * i_len;
	IVP_DOUBLE s = vec1.dot_product(&vec2) * iqlen;
	if (s < 0.0f || s > 1.0f)  {
	  info->friction_is_broken = IVP_TRUE;
	}
    }

#ifdef IVP_USE_S_VALS_FOR_PRETENSION    
    IVP_DOUBLE old_s = s_coords[0];
    s_coords[0] = s;
    diff_contact_vec->add_multiple(&vec1,old_s - s);
#endif    
}


void IVP_Contact_Point::p_calc_friction_ss_KK(const IVP_Compact_Edge *K, const IVP_Compact_Edge *L,
					    IVP_Cache_Ledge_Point *m_cache_K,
					    IVP_Cache_Ledge_Point *m_cache_L,IVP_Impact_Solver_Long_Term *info, IVP_U_Float_Point *diff_contact_vec)
{
  IVP_DOUBLE sl, sk;

  
  IVP_U_Point norm, Kvec, Lvec;
    // very bad code down there #+# (this is a copy paste from an ugly old OS version
  IVP_U_Point Kp;      IVP_CLS.give_world_coords_AT(K, m_cache_K, &Kp);
  IVP_U_Point Kp_next; IVP_CLS.give_world_coords_AT(K->get_next(), m_cache_K, &Kp_next);
  IVP_U_Point Lp;      IVP_CLS.give_world_coords_AT(L, m_cache_L, &Lp);
  IVP_U_Point Lp_next; IVP_CLS.give_world_coords_AT(L->get_next(), m_cache_L, &Lp_next);

  Lvec.subtract(&Lp_next, &Lp);
  Lvec.fast_normize();
  Kvec.subtract(&Kp_next, &Kp);
  Kvec.fast_normize();
  
  norm.calc_cross_product(&Kvec, &Lvec);

  IVP_DOUBLE quad_dist = norm.quad_length();
    
  if(quad_dist > (P_DOUBLE_RES * P_DOUBLE_RES)){ // not parallel
    IVP_U_Point K_area;	// area vertical to K and norm
    IVP_U_Point L_area;	// area vertical to L and norm
    K_area.calc_cross_product(&Kvec, &norm);
    L_area.calc_cross_product(&Lvec, &norm);

    IVP_DOUBLE a,b,x;
    a = K_area.dot_product( &Lp );
    b = K_area.dot_product( &Lp_next );
    IVP_DOUBLE inv_a_b = 1.0f / (a-b);
    x = K_area.dot_product( &Kp );
    sl = (a-x) * inv_a_b;

    a = L_area.dot_product( &Kp );
    b = L_area.dot_product( &Kp_next );
    inv_a_b = 1.0f / (a-b);
    x = L_area.dot_product( &Lp );
    sk = (a-x) * inv_a_b;

    //TL -- modify if necessary  #+#  very bad code down there, vec0 won't be normized soon
    IVP_U_Point vec0,vec1;
    vec0.subtract(&Kp_next, &Kp);
    vec1.subtract(&Lp_next, &Lp);

    IVP_U_Point world_edge_space0,world_edge_space1;
    world_edge_space0.set_interpolate(&Kp,&Kp_next,sk);
    world_edge_space1.set_interpolate(&Lp,&Lp_next,sl);

    IVP_U_Point world_diff_vec;
    world_diff_vec.subtract(&world_edge_space1,&world_edge_space0);

    IVP_DOUBLE length = world_diff_vec.real_length();

    if(length > P_DOUBLE_EPS) {
        last_gap_len = length; 
	info->surf_normal.set_multiple(&world_diff_vec, 1.0f/length);
    }else{
        last_gap_len = 0.0f; 
	info->surf_normal.set_multiple(&norm, 1.0f/ IVP_Inline_Math::ivp_sqrtf(quad_dist));
    }

    info->span_friction_v[0].set(&Kvec);
    info->contact_point_ws.set(&world_edge_space0);

    if (cp_status == IVP_CPBS_NEEDS_RECHECK){
	this->cp_status = IVP_CPBS_NORMAL;
	if (sl < 0.0f || sl > 1.0f || sk < 0.0f || sk > 1.0f) {
	    info->friction_is_broken = IVP_TRUE;
	}
    }

#ifdef IVP_USE_S_VALS_FOR_PRETENSION        
    IVP_DOUBLE old_sk = s_coords[0];
    IVP_DOUBLE old_sl = s_coords[1];
    diff_contact_vec->set_multiple(&vec0,sk - old_sk);
    diff_contact_vec->add_multiple(&vec1,old_sl - sl);    
    s_coords[0] = sk;
    s_coords[1] = sl;
#endif  

  } else {
      info->friction_is_broken = IVP_TRUE;
      diff_contact_vec->set_to_zero();
      last_gap_len = ivp_mindist_settings.keeper_dist;
      info->contact_point_ws.set(&Kp);
      info->surf_normal.set(1.0f,0.0f,0.0f);
      info->span_friction_v[0].set(0.0f,1.0f,0.0f);
      sk=0.0f;
      sl=0.0f;
  }
}

void IVP_Contact_Point::reset_time(IVP_Time offset){
    last_time_of_recalc_friction_s_vals -= offset;
}

void IVP_Contact_Point::recalc_friction_s_vals(){
    IVP_Environment *env = get_synapse(0)->get_object()->get_environment();
    
    IVP_Impact_Solver_Long_Term *info = (IVP_Impact_Solver_Long_Term*)env->get_sim_unit_mem()->get_mem_transaction(sizeof(IVP_Impact_Solver_Long_Term));
    info->init_tmp_contact_info();
    this->tmp_contact_info = info;
    	    
    IVP_Synapse_Friction *syn0 = this->get_synapse(0);
    IVP_Synapse_Friction *syn1 = this->get_synapse(1);
    
    env->get_statistic_manager()->processed_fmindists++;

    IVP_SYNAPSE_POLYGON_STATUS stat0 = syn0->get_status();
    IVP_SYNAPSE_POLYGON_STATUS stat1 = syn1->get_status();
    
    // set: 	this->info.gap_distance, info->surf_normal, info->contact_points_ws[0],
    //		this->span_friction_v[0]
    //          [and add to diff_vec_contact]
    { 
      IVP_U_Float_Point diff_vec_contact; diff_vec_contact.set_to_zero();
	IVP_Cache_Ledge_Point m_cache_0;
	IVP_Cache_Ledge_Point m_cache_1;	

	const IVP_Compact_Edge *e1 = syn1->edge;
	m_cache_1.init_cache_ledge_point(syn1->l_obj,e1->get_compact_ledge());
	m_cache_1.tmp.synapse_friction = syn1;

	const IVP_Compact_Edge *e0 = syn0->edge;
	m_cache_0.init_cache_ledge_point(syn0->l_obj,e0->get_compact_ledge());
	m_cache_0.tmp.synapse_friction = syn0;
    
	IVP_U_Point point0_position_ws; // for ball and point cases

	switch(stat0){
	    
	case IVP_ST_BALL:{
	    point0_position_ws.set(m_cache_0.get_object_cache()->m_world_f_object.get_position());
	    this->cp_status = IVP_CPBS_NEEDS_RECHECK;
	    goto ball_same_as_point;
	}
   
	case IVP_ST_POINT:{
	    IVP_CLS.give_world_coords_AT(e0, &m_cache_0, &point0_position_ws);

	ball_same_as_point:	    
	    switch(stat1){
	    case IVP_ST_BALL:{
		IVP_U_Point *tp2 = m_cache_1.get_object_cache()->m_world_f_object.get_position();
		p_calc_friction_s_PP(&point0_position_ws,tp2,info,&diff_vec_contact);
		break;
	    }
	    
	    case IVP_ST_POINT:{
		IVP_U_Point tp2; IVP_CLS.give_world_coords_AT(e1, &m_cache_1, &tp2);
		p_calc_friction_s_PP(&point0_position_ws,&tp2,info,&diff_vec_contact);
		break;
	    }
	    case IVP_ST_EDGE:{
		p_calc_friction_s_PK(&point0_position_ws, e1, &m_cache_1,info,&diff_vec_contact);
		break;
	    }
	    case IVP_ST_TRIANGLE:{
		p_calc_friction_qr_PF(&point0_position_ws, e1, &m_cache_1,info,&diff_vec_contact);
		break;
	    }
	    default:
		CORE;
	    }
	    break;
	};
	case IVP_ST_EDGE:{
	    IVP_ASSERT( syn1->get_status() == IVP_ST_EDGE );
	    p_calc_friction_ss_KK(e0, e1, &m_cache_0, &m_cache_1,info,&diff_vec_contact);
	    break;
	};
	default:
	    CORE;
	}
	{  // reduce len of friction contact by extra radius
	    IVP_FLOAT first_radius = syn0->l_obj->get_extra_radius();
	    info->contact_point_ws.add_multiple(&info->surf_normal, first_radius);
	    last_gap_len -= first_radius + syn1->l_obj->get_extra_radius();
	}

	//TL: workaround for penetrating balls (shifted mass center)
	if ( last_gap_len < 0.0f ) {
		last_gap_len = 0.0f;
	}    
	m_cache_0.remove_reference();
	m_cache_1.remove_reference();
    }
    info->span_friction_v[0].normize();
    info->span_friction_v[1].inline_calc_cross_product(&info->surf_normal,&info->span_friction_v[0]);
        
// TL:
// IVP_Contact_Point needs for later calculations:
//   both points in world coords
//   both points in object coords
//   normized vector in world coords pointing to first synapse
//   two normized vectors in obj coords pointing to synapse
//   ...
// #+# maybe optimize! e.g.: obj-coords of point synapse never change !

    IVP_U_Float_Point delta_velocity_ws;
    {// calculate some values plus pretension for core 0

	IVP_Core *core0  = syn0->get_object()->get_core();
	
	if (!core0->physical_unmoveable) { // && !core0->pinned){ //@@CBPIN
	    IVP_U_Float_Point surface_normal_cs; // pointing to synapse

	    const IVP_U_Matrix *m_world_f_core = core0->get_m_world_f_core_PSI();
	    m_world_f_core->inline_vimult4(&info->contact_point_ws, &info->contact_point_cs[0]);
	    m_world_f_core->inline_vimult3(&info->surf_normal,&surface_normal_cs);

	    info->contact_cross_nomal_cs[0].inline_calc_cross_product(&info->contact_point_cs[0],& surface_normal_cs );
	    core0->get_surface_speed_on_test(&info->contact_point_cs[0],&core0->speed,&core0->rot_speed,&delta_velocity_ws);

	    IVP_U_Float_Point hp; hp.set_pairwise_mult( &info->contact_cross_nomal_cs[0], core0->get_inv_rot_inertia());
	    info->inv_virtual_mass = info->contact_cross_nomal_cs[0].dot_product( &hp) + core0->get_inv_mass();
	}else{
	    core0 = NULL;
	    delta_velocity_ws.set_to_zero();
	    info->inv_virtual_mass = 0.0f;
	    info->contact_point_cs[0].set_to_zero();
	    info->contact_cross_nomal_cs[0].set_to_zero();
	}
	info->contact_core[0] = core0;
    }
    { // core 1
	IVP_Core *core1  = syn1->get_object()->get_core();
	if (!core1->physical_unmoveable) {// && !core1->pinned){ //@@CBPIN
	    IVP_U_Float_Point surface_normal_cs; // pointing to synapse

	    const IVP_U_Matrix *m_world_f_core = core1->get_m_world_f_core_PSI();
	    m_world_f_core->inline_vimult4(&info->contact_point_ws, &info->contact_point_cs[1]);
	    m_world_f_core->inline_vimult3(&info->surf_normal, &surface_normal_cs);

	    info->contact_cross_nomal_cs[1].inline_calc_cross_product(&info->contact_point_cs[1],&surface_normal_cs );

	    IVP_U_Float_Point dh;
	    core1->get_surface_speed_on_test(&info->contact_point_cs[1],&core1->speed,&core1->rot_speed,&dh);
	    delta_velocity_ws.subtract(&dh);

	    IVP_U_Float_Point hp; hp.set_pairwise_mult( &info->contact_cross_nomal_cs[1], core1->get_inv_rot_inertia());
	    info->inv_virtual_mass += info->contact_cross_nomal_cs[1].dot_product( & hp) + core1->get_inv_mass();
	}else{
	    info->contact_point_cs[1].set_to_zero();
	    info->contact_cross_nomal_cs[1].set_to_zero();
	    core1 = NULL;
	}
	info->contact_core[1] = core1;
    }
    info->virtual_mass = 1.0f/ info->inv_virtual_mass;
    {
	IVP_Time ctime = syn0->get_object()->environment->get_current_time();
	IVP_DOUBLE dt = ctime - this->last_time_of_recalc_friction_s_vals;
	this->last_time_of_recalc_friction_s_vals = ctime;

	//vector of v[0]*s[0] + v[1]*s[1] is real world force vector of force that has to be done on first obj (is pointing to second obj)
	this->span_friction_s[0] -= delta_velocity_ws.dot_product(&info->span_friction_v[0]) * dt;
	this->span_friction_s[1] -= delta_velocity_ws.dot_product(&info->span_friction_v[1]) * dt;
    this->last_contact_point_ws.set( &info->contact_point_ws );
	}
}


IVP_Contact_Point *IVP_Friction_Manager::get_associated_contact_point(IVP_Mindist *mindist) {
    // does a contact_point with same objs and position_status already exist? 
    {
	IVP_Synapse_Friction *syn0;
	IVP_Real_Object *obj0 = mindist->get_synapse(0)->get_object();
	IVP_Real_Object *obj1 = mindist->get_synapse(1)->get_object();
	// search all friction mindist, which are connected to obj0
	// start deep search only if obj1 is connected to mindist also
	for (syn0 = obj0->get_first_friction_synapse(); syn0; syn0 = syn0->get_next()){
	    IVP_Contact_Point *md = syn0->get_contact_point();
	    if ( md->get_synapse(0)->get_object() == obj1 ||  md->get_synapse(1)->get_object() == obj1 ){
		if ( md->is_same_as(mindist)){
		    return md;		    
		}
	    }
	}
    }
    return NULL;
}

void IVP_Friction_Manager::delete_all_contact_points_of_object(IVP_Real_Object *obj0) {
    while (IVP_Synapse_Friction *syn0 = obj0->get_first_friction_synapse()){
        IVP_Contact_Point *md = syn0->get_contact_point();
	P_DELETE(md);
    }
}



// generates a friction mindist when no one exists. Is there already a friction dist, the old one is returned
IVP_Contact_Point *IVP_Friction_Manager::generate_contact_point(IVP_Mindist *mindist,IVP_BOOL *successful){
  
	//
	// Valid call with non-exact and Ipion forces a write to null to crash in its IVP_ASSERT!
	//

//    IVP_ASSERT(mindist->mindist_status == IVP_MD_EXACT); // already enabled?
    
	if ( mindist->mindist_status != IVP_MD_EXACT )
	{
		*successful = IVP_FALSE;
		return NULL;
	}

    // at this time, only polygon PF is allowed for friction mindists
    // KK: not yet, PP & PK never

    IVP_Contact_Point *already_exists_fmd = get_associated_contact_point(mindist);
    if(already_exists_fmd) {
        *successful=IVP_FALSE;
        return already_exists_fmd;
    }

// make new friction mindist
    IVP_Contact_Point *friction =  new IVP_Contact_Point(mindist);
    //insert_contact_point(friction);
    //printf("insert_nmd %lx\n",(long)friction&0x0000ffff);

    *successful=IVP_TRUE;
    return friction;
}

