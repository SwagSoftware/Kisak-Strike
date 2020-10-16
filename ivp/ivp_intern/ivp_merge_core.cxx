// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>
#include <ivp_core_macros.hxx>

#include <ivp_merge_core.hxx>


// merge original cores of linked list of objects

// transforms all object point and anchor coordinates

// results sets m_world_f_core_last_psi, rot_speed, speed




// set m_world_f_core_last_psi.vv

void IVP_Merge_Core::find_mass_center(){

    if (unmovable_core){
	mcore->m_world_f_core_last_psi.vv.set(& unmovable_core->m_world_f_core_last_psi.vv );
    }else{
	IVP_U_Point center;
	center.set_to_zero();
	IVP_DOUBLE mass_sum = P_DOUBLE_EPS;
	for (int i=0;i< n_cores; i++){
	    IVP_Core *core = cores[i];
	    IVP_DOUBLE mass = core->get_mass();
	    center.add_multiple( &core->m_world_f_core_last_psi.vv, mass );
	    mass_sum += mass;
	}
	mcore->m_world_f_core_last_psi.vv.set_multiple(&center, 1.0f / mass_sum);
    }
}

void IVP_Merge_Core::find_damp_factors(){
    if (!unmovable_core){
	IVP_DOUBLE mass_sum = P_DOUBLE_EPS;
	for (int i=0;i< n_cores; i++){
	    IVP_Core *core = cores[i];
	    IVP_DOUBLE mass = core->get_mass();
	    mcore->speed_damp_factor += (IVP_FLOAT)(core->speed_damp_factor * mass);

	    mcore->rot_speed_damp_factor.add_multiple(&core->rot_speed_damp_factor, mass);
	    // mcore->rot_speed_damp_factor += core->rot_speed_damp_factor * mass;
	    mass_sum +=  core->get_mass();
	}
	IVP_DOUBLE inv_mass = 1.0f / mass_sum;
	mcore->speed_damp_factor *= (IVP_FLOAT)(inv_mass);
	mcore->rot_speed_damp_factor.mult(inv_mass);
    }
}

// set m_world_f_core_last_psi
void IVP_Merge_Core::find_main_axis(){
    if (unmovable_core){
	mcore->m_world_f_core_last_psi = unmovable_core->m_world_f_core_last_psi;
	mcore->q_world_f_core_last_psi = unmovable_core->q_world_f_core_last_psi;
	mcore->pos_world_f_core_last_psi = unmovable_core->pos_world_f_core_last_psi;
	mcore->delta_world_f_core_psis.set_to_zero();
    }else{
	int max_i = 0;
	IVP_DOUBLE max_mass = 0.0f;
	
	// simply search maximum mass
	for (int i=0;i< n_cores; i++){
	    IVP_Core *core = cores[i];
	    if (core->get_mass() > max_mass){
		max_mass = core->get_mass();
		max_i = i;
	    }
	}
	IVP_Core *c = cores[max_i];
	mcore->m_world_f_core_last_psi = c->m_world_f_core_last_psi;
	mcore->q_world_f_core_last_psi = c->q_world_f_core_last_psi;
	mcore->pos_world_f_core_last_psi = c->pos_world_f_core_last_psi;
	mcore->delta_world_f_core_psis.set_to_zero();
    }
}

void IVP_Merge_Core::set_radius(){
    IVP_DOUBLE max_radius = 0.0f;
    for (int i=0;i< n_cores; i++){
	IVP_Core *core = cores[i];
	IVP_DOUBLE radius = core->m_world_f_core_last_psi.vv.quad_distance_to(
	    &mcore->m_world_f_core_last_psi.vv);
	radius = IVP_Inline_Math::sqrtd(radius);
	radius += core->upper_limit_radius;
	if (radius > max_radius) max_radius = radius;
    }
    mcore->upper_limit_radius = (IVP_FLOAT)max_radius;
    mcore->max_surface_deviation = (IVP_FLOAT)max_radius;
}
    

void IVP_Merge_Core::set_speed(){
    if (unmovable_core){
	mcore->rot_speed.set_to_zero();
	((IVP_U_Float_Point&)*mcore->get_rot_inertia()).set(1.0f,1.0f,1.0f);
	mcore->speed.set_to_zero();
	return;
    }
    
    IVP_U_Float_Point sum_moment;
    IVP_U_Float_Point sum_rot_inertia;
    IVP_U_Float_Point sum_rot_inertia_speed;
    
    IVP_DOUBLE mass_sum = 0.0f;
    sum_moment.set_to_zero();
    sum_rot_inertia.set_to_zero();
    sum_rot_inertia_speed.set_to_zero();
    
    for (int i=0;i< n_cores; i++){
	IVP_Core *core = cores[i];
	
	mass_sum += core->get_mass();
	sum_moment.add_multiple(&core->speed, core->get_mass());

	{
	    IVP_U_Float_Point quad_sum_rot_inertia;
	    quad_sum_rot_inertia.set_to_zero();
	    IVP_U_Matrix *m = &core->m_world_f_core_last_psi;

	    // add spin momentom by original spin momentom
	    for (int s = 0;s<3;s++){	// s for source
		IVP_U_Float_Point world_achse;
		m->get_col( IVP_COORDINATE_INDEX(s), &world_achse);
		IVP_U_Float_Point projected_achse;	// spin of old axis projected to new axis
		// new axis are columns in m_world_f_mcore
		mcore->m_world_f_core_last_psi.vimult3( & world_achse, &projected_achse );
		sum_rot_inertia_speed.add_multiple( &projected_achse, core->get_rot_inertia()->k[s] * core->rot_speed.k[s]);

		projected_achse.set_pairwise_mult(&projected_achse, &projected_achse);
		IVP_DOUBLE rot_inertia_for_this_axis = core->get_rot_inertia()->k[s];
		quad_sum_rot_inertia.add_multiple( &projected_achse, rot_inertia_for_this_axis * rot_inertia_for_this_axis);
	    }
	    quad_sum_rot_inertia.line_sqrt();
	    sum_rot_inertia.add(&quad_sum_rot_inertia);
	}

	// add spin momentum by different mass centers
	{
	    IVP_U_Float_Point m_center_in_mcore;
	    IVP_U_Float_Point speed_in_mcore;
	    IVP_U_Float_Point distances_2d;
	    mcore->m_world_f_core_last_psi.vimult4( &core->m_world_f_core_last_psi.vv, &m_center_in_mcore);
	    mcore->m_world_f_core_last_psi.vimult3( &core->speed, &speed_in_mcore);

	    // see test_push_core for details
	    distances_2d.inline_calc_cross_product(&m_center_in_mcore,&speed_in_mcore);
	    sum_rot_inertia_speed.add_multiple( &distances_2d, core->get_mass());

	    IVP_U_Float_Point quad_center;
	    quad_center.set_pairwise_mult(&m_center_in_mcore, &m_center_in_mcore);
	    sum_rot_inertia.k[0] += IVP_Inline_Math::sqrtd(quad_center.k[1] + quad_center.k[2]) * core->get_mass();
	    sum_rot_inertia.k[1] += IVP_Inline_Math::sqrtd(quad_center.k[0] + quad_center.k[2]) * core->get_mass();
	    sum_rot_inertia.k[2] += IVP_Inline_Math::sqrtd(quad_center.k[0] + quad_center.k[1]) * core->get_mass();
	}
    }
    mcore->speed.set_multiple(&sum_moment, 1.0f/mass_sum);

    IVP_U_Float_Point inv_rot_inertia;
    for (int d=0;d<3;d++){	// d for destination
	inv_rot_inertia.k[d] = 1.0f / sum_rot_inertia.k[d];
    }
    IVP_U_Float_Hesse &ri = (IVP_U_Float_Hesse &) *mcore->get_rot_inertia();
    ri.set(&sum_rot_inertia);
    ri.hesse_val = IVP_FLOAT(mass_sum);
    mcore->rot_speed.set_pairwise_mult( &sum_rot_inertia_speed, &inv_rot_inertia);
}

IVP_Merge_Core::IVP_Merge_Core() {
    P_MEM_CLEAR(this);
}

IVP_Merge_Core::IVP_Merge_Core(IVP_Core_Merged *mcore_, IVP_Core *first_core, IVP_Core *second_core){
    P_MEM_CLEAR(this);
    mcore = mcore_;
    n_cores = 2;
    cores = &core_stack[0];
    cores[0] = first_core;
    cores[1] = second_core;
}

IVP_Merge_Core::~IVP_Merge_Core(){
    if (cores != &core_stack[0]){
	delete cores;
    }
}


void IVP_Merge_Core::synchronize_motion(){
    IVP_DOUBLE d_time = mcore->environment->get_current_time() - mcore->environment->get_old_time_of_last_PSI();
    if (d_time > 0.0f){	// not within PSI
	for (int i=0;i< n_cores; i++){
	    IVP_Core *core = cores[i];
	    if (IVP_MTIS_SIMULATED(core->movement_state)){
	      core->synchronize_with_rot_z( );
	    }
	}
    }
}

void IVP_Merge_Core::check_for_unmovable_core(){
    unmovable_core = NULL;
    mcore->fast_piling_allowed_flag = IVP_TRUE;
    movement_type = IVP_MT_NOT_SIM;
    
    for (int i=0;i< n_cores; i++){
	IVP_Core *core = cores[i];
	if (core->fast_piling_allowed() == IVP_FALSE){
	    mcore->fast_piling_allowed_flag = IVP_FALSE;
	}
	if (core->physical_unmoveable == IVP_TRUE){
	    unmovable_core = core;
	}else{
	    if (core->movement_state < IVP_MT_NOT_SIM){
		movement_type = (IVP_Movement_Type)core->movement_state;
	    }
	}
    }
    mcore->movement_state = IVP_MT_NOT_SIM;
    if (unmovable_core){
      mcore->movement_state = IVP_MT_STATIC;
	mcore->physical_unmoveable = IVP_TRUE;
    }else{
	mcore->physical_unmoveable = IVP_FALSE;
    }
}

void IVP_Merge_Core::place_objects(){
    for (int i=0;i< n_cores; i++){
	IVP_Core *core = cores[i];
	IVP_U_Matrix m_ncore_f_core;
	mcore->m_world_f_core_last_psi.mimult4( & core->m_world_f_core_last_psi, &m_ncore_f_core);
	int o;
	for (o = core->objects.len()-1;o>=0;o--){
	    IVP_Real_Object *obj = core->objects.element_at(o);
	    IVP_U_Matrix m_core_f_object;
	    obj->calc_m_core_f_object(&m_core_f_object);

	    IVP_U_Matrix m_ncore_f_object;
	    m_ncore_f_core.mmult4( &m_core_f_object, &m_ncore_f_object);

	    IVP_U_Matrix m_object_f_ncore;
	    m_object_f_ncore.set_transpose(&m_ncore_f_object);

	    obj->set_new_m_object_f_core( &m_object_f_ncore);
	}
    }


}

void IVP_Merge_Core::calc_calc(){
    check_for_unmovable_core();
    
    synchronize_motion();
    find_main_axis();		// sets m_world_f_core_last_psi.mm
    find_mass_center();		// sets m_world_f_core_last_psi.vv
    place_objects();		// set new_m_object_f_core of all objects
    mcore->m_world_f_core_when_created = mcore->m_world_f_core_last_psi;
    
    find_damp_factors();

    set_speed();
    set_radius();
    mcore->calc_calc();
}


// merge cores,
// note: does not call calc_next_PSI_matrix
void IVP_Core_Merged::set_by_merge(IVP_Core *c0,IVP_Core *c1){
    IVP_Merge_Core merge(this,c0,c1);
    merge.calc_calc();
}


