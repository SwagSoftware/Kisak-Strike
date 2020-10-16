// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.




#ifndef WIN32
#	pragma implementation "ivp_core.hxx"
#	pragma implementation "ivp_core_macros.hxx"
#endif

#include <ivp_physics.hxx>
#include <ivu_matrix_macros.hxx>
#include <ivu_float.hxx>
#include <ivu_memory.hxx>

#include <ivp_core_macros.hxx>
#include <ivp_sim_unit.hxx>
#include <ivp_hull_manager.hxx>

#include <ivp_merge_core.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_debug_manager.hxx>

#include <ivp_mindist_intern.hxx>
#include <ivp_friction.hxx>
#include <ivp_i_friction_hash.hxx>

#include <ivp_calc_next_psi_solver.hxx>

/* Note: When z is the coordinate of the Z axis,
 * and x and y lay in plane orthogonal to z
 * then following equations are valid in a modulo 3 ring:
 * x+1 = y  and y+1 = z
 * this means Axis 2 -> 0 1
 *            Axis 0 -> 1 2
 *            Axis 1 -> 2 0
 */

// The same in german:

/* Sei z die Koordinate der Achse Z
 * und sind x und y der dazu senkrechten Ebene dann gilt
 * x+1 = y   und y+1 =z im Ring Z3
 * also	Achse 2	-> 0 1
 *		Achse 0	-> 1 2
 *		Achse 1	-> 2 0
 */

#define IVP_SAFETY_FACTOR_HULL 1.1f

#define IVP_MINIMAL_TRANS_SPEED_PER_PSI_SHORT 0.01f
#define IVP_MINIMAL_ROT_SPEED_PER_PSI_SHORT 0.005f
#define IVP_MINIMAL_TRANS_SPEED_PER_PSI_LONG 0.1f
#define IVP_MINIMAL_ROT_SPEED_PER_PSI_LONG 0.2f

#define IVP_HAS_MOVED_LONG_TIME 4.0f //sec


#define MAX_PLAUSIBLE_LEN 100000.0f //slower than meter/sec  //XXX 0 added by chris!



// get surface speed; take rotation speed and center speed as input
void IVP_Core::get_surface_speed_on_test(const IVP_U_Float_Point *point_cs,
					 const IVP_U_Float_Point *center_speed_ws,
					 const IVP_U_Float_Point *rot_speed_cs,
					 IVP_U_Float_Point *speed_out_ws) const{
    IVP_U_Float_Point speed_cs;
    speed_cs.inline_calc_cross_product(rot_speed_cs,point_cs);
    this->m_world_f_core_last_psi.inline_vmult3(&speed_cs,speed_out_ws); // transform to world coords
    speed_out_ws->add(center_speed_ws);
}

void IVP_Core::get_surface_speed_ws( const IVP_U_Point *position_ws_in, IVP_U_Float_Point *speed_ws_out) {
    IVP_U_Float_Point av_ws;
    get_m_world_f_core_PSI()->vmult3( &this->rot_speed, &av_ws);
    IVP_U_Float_Point pos_rel; 	pos_rel.subtract( position_ws_in, get_position_PSI());
    IVP_U_Float_Point cross;    cross.inline_calc_cross_product(&av_ws,&pos_rel);
    speed_ws_out->add(&speed, &cross);
}


void IVP_Core::get_diff_surface_speed_of_two_cores(const IVP_Core *this_core, const IVP_Core *other_core,
						   const IVP_U_Float_Point *obj_point_this, const IVP_U_Float_Point *obj_point_other,
						   IVP_U_Float_Point *delta_velocity_ws){
    if(this_core && !this_core->physical_unmoveable)    {
	this_core->get_surface_speed_on_test(obj_point_this,&this_core->speed,&this_core->rot_speed,delta_velocity_ws);
    } else {
	delta_velocity_ws->set_to_zero();
    }

    if(other_core && !other_core->physical_unmoveable)    {
	IVP_U_Float_Point velocity_ws_2;
	other_core->get_surface_speed_on_test(obj_point_other,&other_core->speed,&other_core->rot_speed,&velocity_ws_2);
        delta_velocity_ws->subtract(&velocity_ws_2);
    }
}

void IVP_Core::get_diff_surface_speed_of_two_cores_on_test(const IVP_Core *this_core, const IVP_Core *other_core,
							   const IVP_U_Float_Point *obj_point_this, const IVP_U_Float_Point *obj_point_other,
							   const IVP_U_Float_Point *trans_speed0, const IVP_U_Float_Point *rot_speed0,
							   const IVP_U_Float_Point *trans_speed1, const IVP_U_Float_Point *rot_speed1,
							   IVP_U_Float_Point *diff_speed_this_minus_other_out)
{
    IVP_U_Float_Point world_this,world_other;
    if(!this_core->physical_unmoveable)    {
	this_core->get_surface_speed_on_test(obj_point_this,trans_speed0,rot_speed0,&world_this);
    } else {
	world_this.set_to_zero();
    }

    if(!other_core->physical_unmoveable)    {
	other_core->get_surface_speed_on_test(obj_point_other,trans_speed1,rot_speed1,&world_other);
    } else {
	world_other.set_to_zero();
    }
    diff_speed_this_minus_other_out->subtract(&world_this,&world_other);
}



IVP_DOUBLE IVP_Core::calc_virt_mass_worst_case(const IVP_U_Float_Point *core_point) const {
    IVP_U_Float_Point worst_direction;
    worst_direction.k[0]=core_point->k[1]*core_point->k[1] + core_point->k[2]*core_point->k[2];
    worst_direction.k[1]=core_point->k[0]*core_point->k[0] + core_point->k[2]*core_point->k[2];
    worst_direction.k[2]=core_point->k[0]*core_point->k[0] + core_point->k[1]*core_point->k[1];
    worst_direction.set_pairwise_mult(&worst_direction,this->get_inv_rot_inertia());
    IVP_DOUBLE max_of_coords=worst_direction.k[0];
    if(worst_direction.k[1]>max_of_coords) {
	max_of_coords=worst_direction.k[1];
    }
    if(worst_direction.k[2]>max_of_coords) {
	max_of_coords=worst_direction.k[2];
    }
    IVP_DOUBLE dv;

	if(!pinned) //@@CBPIN
	{
		dv = this->get_inv_mass() + max_of_coords;
	}
	else
	{
		dv = 1.0f + max_of_coords;
	}

    IVP_DOUBLE ret = 1.0f/dv;
    IVP_ASSERT(ret < 1e31f); // OG test for NaN
    return ret;
}

// also calculates some material dependent redundants
void IVP_Core::revive_adjacent_to_unmoveable() {
        IVP_ASSERT( this->physical_unmoveable );

	this->environment->get_sim_unit_mem()->start_memory_transaction();

	int d;
	for(d = this->objects.len()-1;d>=0;d--) {
	    IVP_Real_Object *r_obj=this->objects.element_at(d);
	    IVP_Synapse_Friction *fr_synapse,*next_syn;
	    IVP_Contact_Point *fr_mindist;
	    IVP_Friction_System *fs;
	    fr_synapse=r_obj->get_first_friction_synapse();
	    while( fr_synapse ) {
		next_syn=fr_synapse->get_next();

		fr_mindist=fr_synapse->get_contact_point();
		fs=fr_mindist->l_friction_system;
		fs->union_find_necessary=IVP_TRUE;
		IVP_Core *other_core;
		other_core=fr_mindist->get_synapse(0)->l_obj->get_core();
		if( other_core == this ) {
		    other_core=fr_mindist->get_synapse(1)->l_obj->get_core();
		}
		if( other_core->physical_unmoveable ) {
		    //printf("found_unnecassary_fmd\n");
		    P_DELETE( fr_mindist );
		} else {
		    other_core->ensure_all_core_objs_in_simulation();
			fr_mindist->recalc_friction_s_vals();
		    IVP_Impact_Solver_Long_Term info;
		    fr_mindist->read_materials_for_contact_situation(&info); //this is done only to get friction_factor ; maybe make extra function for friction_factor
		    fr_mindist->calc_virtual_mass_of_mindist();
		}
		fr_synapse=next_syn;
	    }
	}

	this->environment->get_sim_unit_mem()->end_memory_transaction();
}

void IVP_Core::values_changed_recalc_redundants() {
    int d;

    environment->sim_unit_mem->start_memory_transaction();
    
    calc_calc();

    //EMAURER
    environment->sim_unit_mem->end_memory_transaction();        
    
    IVP_Movement_Type mt_for_objs;
    if( this->physical_unmoveable ) {
	mt_for_objs=IVP_MT_STATIC;
	this->movement_state=IVP_MT_NOT_SIM; //flag changed from moveable to unmoveable
    } else {
	if( this->movement_state > IVP_MT_NOT_SIM ) {
	    this->movement_state=IVP_MT_NOT_SIM; //flag changed from unmoveable to moveable
	}
	mt_for_objs=(IVP_Movement_Type)movement_state;
    }

    for(d = this->objects.len()-1;d>=0;d--) {
	IVP_Real_Object *r_obj=this->objects.element_at(d);
	r_obj->set_movement_state(mt_for_objs);
	//r_obj->revive_nearest_objects_grow_fs();
    }

    if( !this->physical_unmoveable ) {
        //the core is movable

        //revive core for simulation
	this->ensure_core_to_be_in_simulation();

        //EMAURER
        environment->sim_unit_mem->start_memory_transaction();
	
	//change all friction mindist values (material may have changed)
	for(d = this->objects.len()-1;d>=0;d--) {
	    IVP_Real_Object *r_obj=this->objects.element_at(d);

	    IVP_Contact_Point *fr_mindist;
	    IVP_Synapse_Friction *fr_synapse;
	    IVP_Friction_System *fs;
	    fr_synapse=r_obj->get_first_friction_synapse();
	    if( fr_synapse ) {
		fr_mindist=fr_synapse->get_contact_point();
		fs=fr_mindist->l_friction_system;
	    }
	    
	    for(;fr_synapse;fr_synapse=fr_synapse->get_next()) {
		fr_mindist=fr_synapse->get_contact_point();
		IVP_Friction_System *next_fs=fr_mindist->l_friction_system;
		if(next_fs != fs) {
		    //this->environment->get_friction_manager()->fusion_friction_systems(fs,next_fs);
		    fs->fusion_friction_systems(next_fs);
		}
		IVP_Impact_Solver_Long_Term info;
		fr_mindist->recalc_friction_s_vals();
		fr_mindist->read_materials_for_contact_situation(&info); //this is only done to calculate friction_factor ; maybe make extra function for friction_factor
		fr_mindist->calc_virtual_mass_of_mindist();
		
	    }
	}
	//EMAURER
        environment->sim_unit_mem->end_memory_transaction();    
    } else {
        //the core is unmovable
      
	this->stop_movement_without_collision_recheck();

	//change friction values and revive other adjacent cores
	revive_adjacent_to_unmoveable();
	
	//this loop is slow as it is quadratic in number of fr_mindists
	for(d = this->objects.len()-1;d>=0;d--) {
	    IVP_Real_Object *r_obj=this->objects.element_at(d);
	    IVP_Synapse_Friction *fr_synapse,*next_syn;
	    IVP_Contact_Point *fr_mindist;
	    fr_synapse=r_obj->get_first_friction_synapse();
	    while( fr_synapse ) {
		next_syn=fr_synapse->get_next();
		fr_mindist=fr_synapse->get_contact_point();
		IVP_Core *other_core;
		other_core=fr_mindist->get_synapse(0)->l_obj->get_core();
		if( other_core == this ) {
		    other_core=fr_mindist->get_synapse(1)->l_obj->get_core();
		}
		
		fr_synapse=next_syn;
	    }
	}
    }
    //EMAURER
    //environment->sim_unit_mem->end_memory_transaction();    
}

//lwss add
void IVP_Core::apply_velocity_limit()
{
    // this was a real PITA to reverse, but is correct! Someone give me a medal.
    IVP_Anomaly_Limits *limits = this->environment->anomaly_limits;
    if( limits->max_velocity > 0.0 )
    {
        float flSpeed = this->speed.real_length();
        if( flSpeed > limits->max_velocity )
        {
            float scaleBack = limits->max_velocity / flSpeed;
            this->speed.mult( scaleBack );
        }

        float flSpeedChange = this->speed_change.real_length();
        if( flSpeedChange > limits->max_velocity )
        {
            float scaleBack = limits->max_velocity / flSpeedChange;
            this->speed_change.mult( scaleBack );
        }
    }
    if( limits->max_angular_velocity_per_psi > 0.0 )
    {
        float flRotSpeed = this->rot_speed.real_length();
        float flMaxAngularVelocity = limits->max_angular_velocity_per_psi * this->environment->inv_delta_PSI_time; // unsure about inv_Delta_psi_time
        if( flRotSpeed > flMaxAngularVelocity )
        {
            float scaleBack = flMaxAngularVelocity / flRotSpeed;
            this->rot_speed.mult( scaleBack );
        }

        float flSpeedChange = this->speed_change.real_length();
        if( flSpeedChange > flMaxAngularVelocity )
        {
            float scaleBack = flMaxAngularVelocity / flSpeedChange;
            this->rot_speed_change.mult( scaleBack );
        }
    }
}
//lwss end

IVP_DOUBLE IVP_Core::calc_correct_virt_mass(const IVP_U_Float_Point *core_point,const IVP_U_Float_Point *direction_core,const IVP_U_Float_Point *direction_world) const {
    if(!pinned) //@@CBPIN
    {
	IVP_U_Float_Point ret_center_speed,ret_rot_speed;
	this->test_push_core(core_point,direction_core,direction_world,&ret_center_speed,&ret_rot_speed);

	IVP_U_Float_Point ret_world_speed;
	this->get_surface_speed_on_test(core_point,&ret_center_speed,&ret_rot_speed,&ret_world_speed);
	IVP_DOUBLE speed_after_test_push=ret_world_speed.real_length();
	return 1.0f/speed_after_test_push;
    }
    else
    {
	return 1.0f;
    }
}

void p_calc_2d_distances_to_axis(const IVP_U_Float_Point *point,const IVP_U_Float_Point *direction, IVP_U_Float_Point *distances){
/* estimate (2d) distance of straight(point,direction) to 0,0,0
 * after setting all k[0],k[1], k[2] to zero */
    IVP_DOUBLE qlen = direction->real_length();
    if (qlen > P_DOUBLE_EPS){
	IVP_DOUBLE iqlen = 1.0f/qlen;
	distances->calc_cross_product( direction, point);
	distances->mult(iqlen);
    }else{
	distances->set_to_zero();
    }
} 


IVP_DOUBLE IVP_Core::calc_virt_mass(const IVP_U_Float_Point *core_point,const  IVP_U_Float_Point *direction)const{
    /* calculates the virtual mass of on object at point point
     * and direction direction_cs, if direction == 0 than get mimimal mass */
    IVP_U_Float_Point distances_2d;
    const IVP_Core *pc = this;
    IVP_DOUBLE dv;
    if (direction){
	p_calc_2d_distances_to_axis(core_point,direction,&distances_2d);
	distances_2d.set_pairwise_mult(&distances_2d, &distances_2d);		// use ^2 because speed = rot_speed * radius
	
	/* Now virtual push object by Ft = mv = 1.0f kg*m/sec
	 * and sum dv
	 * than resolve mdv = Ft to m */
	distances_2d.set_pairwise_mult(&distances_2d,pc->get_inv_rot_inertia());
	dv = pc->get_inv_mass() + distances_2d.real_length();
    }else{
	distances_2d.set(core_point);

	distances_2d.set_pairwise_mult(&distances_2d, &distances_2d);		// use ^2 because speed = rot_speed * radius
	
	/* Now virtual push object by Ft = mv = 1.0f kg*m/sec
	 * and sum dv
	 * than resolve mdv = Ft to m */
	distances_2d.set_pairwise_mult(&distances_2d,pc->get_inv_rot_inertia());
	dv = pc->get_inv_mass() + distances_2d.real_length();
    }
    return 1.0f/dv;
}

#if 0
IVP_DOUBLE IVP_Core::get_energy(){
    IVP_DOUBLE energy = speed.quad_length() * this->mass;
    IVP_U_Float_Point hp;
    hp.set_pairwise_mult( &rot_speed, &rot_speed);
    energy += hp.dot_product(&rot_inertia);
    return 0.5f * energy;
}
#endif

IVP_DOUBLE IVP_Core::get_energy_on_test(const IVP_U_Float_Point *speed_vec,const IVP_U_Float_Point *rot_speed_vec){
    IVP_DOUBLE energy = speed_vec->quad_length() * this->get_mass();
    IVP_U_Float_Point hp;
    hp.set_pairwise_mult( rot_speed_vec, rot_speed_vec);
    energy += hp.dot_product(get_rot_inertia());
    return 0.5f * energy;
}

void IVP_Core::set_radius(IVP_FLOAT upper_limit, IVP_FLOAT max_dev){
    upper_limit_radius = upper_limit;
    max_surface_deviation = max_dev;
}


void IVP_Core::calc_at_matrix( IVP_Time current_time, IVP_U_Matrix *m_world_f_core_out) const {
    while(IVP_MTIS_SIMULATED(movement_state)){
	IVP_DOUBLE d_time = current_time - time_of_last_psi;
	if (d_time == 0.0f) break;	// e.g. beginn of psi
	// quaternion based solution
	{
	  IVP_U_Quat res;
	  res.set_interpolate_smoothly( &q_world_f_core_last_psi, &q_world_f_core_next_psi, d_time * i_delta_time );
	  res.set_matrix(m_world_f_core_out);
	  m_world_f_core_out->get_position()->add_multiple( &pos_world_f_core_last_psi, &delta_world_f_core_psis, d_time );
	}
	return;
    }
    *m_world_f_core_out = m_world_f_core_last_psi; // no movement
}
    

void IVP_Core::abort_all_async_pushes()
{
    this->rot_speed_change.set_to_zero();
    this->speed_change.set_to_zero();
}


void IVP_Core::global_damp_core(IVP_DOUBLE d_time) {
    
	IVP_Core *my_core=this;

	if (my_core->movement_state>=IVP_MT_SLOW){
	    IVP_FLOAT s_add_on = 0.1f;
	    IVP_U_Float_Point add_on(s_add_on, s_add_on, s_add_on); // somewhat more dampening for slow objects
	    IVP_U_Float_Point rot_damp_factor;
	    rot_damp_factor.add(&my_core->rot_speed_damp_factor, &add_on);
	    
	    my_core->damp_object(d_time,
				 &rot_damp_factor,
				 my_core->speed_damp_factor + s_add_on); //60 1.5
	}else{
	    my_core->damp_object(d_time,
				 &my_core->rot_speed_damp_factor,
				 my_core->speed_damp_factor); //60 1.5
	}	
}

// object is affected after commit_all_async_pushes
void IVP_Core::center_push_core_multiple_ws(const IVP_U_Float_Point *delta_speed,IVP_DOUBLE factor){
    this->speed.add_multiple(delta_speed, factor * get_inv_mass());
}

void IVP_Core::async_center_push_core_multiple_ws(const IVP_U_Float_Point *delta_speed,IVP_DOUBLE factor){
    this->speed_change.add_multiple(delta_speed, factor * get_inv_mass());
}

void IVP_Core::damp_object(IVP_DOUBLE delta_time_, const IVP_U_Float_Point *rotation_factor, IVP_DOUBLE speed_factor){
    IVP_U_Float_Point rot;
    rot.set_multiple ( rotation_factor , delta_time_);

    IVP_DOUBLE qlen = rot.quad_length();

    IVP_U_Float_Point r_factor;
    if (qlen < 0.5f){  // small dampening
	r_factor.set( 1.0f - rot.k[0],  1.0f - rot.k[1],  1.0f - rot.k[2]);
    }else{
	r_factor.set( IVP_Inline_Math::ivp_expf(-rot.k[0]),
		      IVP_Inline_Math::ivp_expf(-rot.k[1]),
		      IVP_Inline_Math::ivp_expf(-rot.k[2]) );
    }
    
    IVP_DOUBLE sf = speed_factor * delta_time_;
    if (sf < 0.25f){
	speed_factor = 1.0f - sf;
    }else{
	speed_factor = IVP_Inline_Math::ivp_expf(-sf);
    }
    IVP_Core *pc = this;
    pc->rot_speed.set_pairwise_mult(&pc->rot_speed, &r_factor);
    pc->speed.mult(speed_factor);
}

void IVP_Core::set_rotation_inertia( const IVP_U_Float_Point *r){
    rot_inertia.set(r);
    this->calc_calc();
}


void IVP_Core::get_surface_speed(const IVP_U_Float_Point *point_core, IVP_U_Float_Point *speed_world_out) const{
    this->get_surface_speed_on_test(point_core,&this->speed,&this->rot_speed,speed_world_out);
}


// change rot_speed_change and speed_change, needs movement_transaction // #+# to be killed
// for (springs)
void IVP_Core::async_push_core(const IVP_U_Float_Point		*point_cs,
				     const IVP_U_Float_Point	*impulse_in_core,
				     const IVP_U_Float_Point	*impulse_in_world){
    IVP_U_Float_Point rot_change,center_change;
    this->test_push_core(point_cs,impulse_in_core,impulse_in_world,&center_change,&rot_change); //stores results in variables rot_change and center_change

    rot_speed_change.add( &rot_change);
    speed_change.add( & center_change );
}

// change rot_speed and speed, instantly // #+# to be killed
void IVP_Core::push_core(const IVP_U_Float_Point	*point_cs,
			 const IVP_U_Float_Point	*impulse_in_core,
			 const IVP_U_Float_Point	*impulse_in_world){
    
    IVP_U_Float_Point rot_change,center_change;
    this->test_push_core(point_cs,impulse_in_core,impulse_in_world,&center_change,&rot_change); //stores results in variables rot_change and center_change
    rot_speed.add( &rot_change);
    speed.add( & center_change );
}

// change rot_speed and speed, instantly
void IVP_Core::push_core_ws(const IVP_U_Point *world_point,
			    const IVP_U_Float_Point *impulse_in_world)
{
    const IVP_U_Matrix *m_world_f_core = get_m_world_f_core_PSI();
    IVP_U_Float_Point point_d_ws; point_d_ws.subtract(world_point, m_world_f_core->get_position());
    IVP_U_Float_Point cross_point_dir;
    cross_point_dir.calc_cross_product( &point_d_ws, impulse_in_world);
    m_world_f_core->inline_vimult3( &cross_point_dir, &cross_point_dir);
    cross_point_dir.set_pairwise_mult( &cross_point_dir, get_inv_rot_inertia());
    rot_speed.add( &cross_point_dir);
    speed.add_multiple( impulse_in_world, get_inv_mass() );
}

// push object to change 
void IVP_Core::async_push_core_ws(const IVP_U_Point *world_point,
				  const IVP_U_Float_Point *impulse_in_world)
{    
    const IVP_U_Matrix *m_world_f_core = get_m_world_f_core_PSI();
    IVP_U_Float_Point point_d_ws; point_d_ws.subtract(world_point, m_world_f_core->get_position());
    IVP_U_Float_Point cross_point_dir;
    cross_point_dir.calc_cross_product( &point_d_ws, impulse_in_world);
    m_world_f_core->inline_vimult3( &cross_point_dir, &cross_point_dir);
    cross_point_dir.set_pairwise_mult( &cross_point_dir, get_inv_rot_inertia());
    rot_speed_change.add( &cross_point_dir);
    speed_change.add_multiple( impulse_in_world, get_inv_mass() );
}

void IVP_Core::async_rot_push_core_multiple_ws( const IVP_U_Float_Point *angular_impulse_ws, IVP_DOUBLE factor){
    const IVP_U_Matrix *m_world_f_core = get_m_world_f_core_PSI();
    IVP_U_Float_Point angular_impulse_cs;
    m_world_f_core->vimult3 ( angular_impulse_ws, &angular_impulse_cs);

    IVP_U_Float_Point h;
    h.set_pairwise_mult( &angular_impulse_cs, get_inv_rot_inertia());
    rot_speed_change.add_multiple(&h,factor);
}

void IVP_Core::rot_push_core_multiple_ws( const IVP_U_Float_Point *angular_impulse_ws, IVP_DOUBLE factor){
    const IVP_U_Matrix *m_world_f_core = get_m_world_f_core_PSI();
    IVP_U_Float_Point angular_impulse_cs;
    m_world_f_core->vimult3 ( angular_impulse_ws, &angular_impulse_cs);

    IVP_U_Float_Point h;
    h.set_pairwise_mult( &angular_impulse_cs, get_inv_rot_inertia());
    rot_speed.add_multiple(&h,factor);
}




// don't store result of push directly in core variables but in temporary variables
void IVP_Core::test_push_core(const IVP_U_Float_Point *point_cs,
			      const IVP_U_Float_Point *impulse_in_core,const IVP_U_Float_Point *impulse_in_world,
			      IVP_U_Float_Point *speed_out,IVP_U_Float_Point *rot_out) const{
    IVP_U_Float_Point distances_2d;
    distances_2d.inline_calc_cross_product(point_cs,impulse_in_core);    
    rot_out->set_pairwise_mult(&distances_2d,this->get_inv_rot_inertia());
    speed_out->set_multiple(impulse_in_world, this->get_inv_mass());
}



// don't store result of rot push directly in core variables but in temporary variables
void IVP_Core::test_rot_push_core_multiple_cs( const IVP_U_Float_Point *normized_core_axis,
					       IVP_DOUBLE rot_impulse,
					       IVP_U_Float_Point *delta_rot_speed_out)
{
    IVP_U_Float_Point h;
    h.set_pairwise_mult( normized_core_axis, get_inv_rot_inertia());
    delta_rot_speed_out->set_multiple(&h, rot_impulse);
}


IVP_DOUBLE IVP_Core::get_rot_inertia_cs(const IVP_U_Float_Point *normized_core_axis){
    IVP_U_Float_Point h;
    h.set_pairwise_mult(normized_core_axis, get_rot_inertia());
    IVP_DOUBLE len = h.real_length();
    return len;
}

IVP_DOUBLE IVP_Core::get_rot_speed_cs(const IVP_U_Float_Point *normized_core_axis){
    return rot_speed.dot_product(normized_core_axis);
}

void IVP_Core::async_rot_push_core_multiple_cs( const IVP_U_Float_Point *normized_core_axis, IVP_DOUBLE rot_impulse){
    IVP_U_Float_Point h;
    h.set_pairwise_mult( normized_core_axis, get_inv_rot_inertia());
    rot_speed_change.add_multiple(&h,rot_impulse);
}

void IVP_Core::rot_push_core_multiple_cs( const IVP_U_Float_Point *normized_core_axis, IVP_DOUBLE rot_impulse){
    IVP_U_Float_Point h;
    h.set_pairwise_mult( normized_core_axis, get_inv_rot_inertia());
    rot_speed.add_multiple(&h,rot_impulse);
}

void IVP_Core::rot_push_core_cs(const IVP_U_Float_Point *rot_impulse_cs)
{
    IVP_U_Float_Point h;
    h.set_pairwise_mult(rot_impulse_cs, get_inv_rot_inertia());
    this->rot_speed.add(&h);
}


// #+# too many calls to this function
void IVP_Core::commit_all_async_pushes(){   

    this->rot_speed.add(&this->rot_speed_change);
    this->speed.add(&this->speed_change);
    this->speed_change.set_to_zero(); 
    this->rot_speed_change.set_to_zero();
    IVP_IF(1){
    	core_plausible_check();
    }
}

void IVP_Core::calc_next_PSI_matrix_zero_speed(IVP_Event_Sim *es){
    i_delta_time = es->i_delta_time;

    q_world_f_core_next_psi = q_world_f_core_last_psi;
    
    this->abs_omega = 0.0f;
    this->speed.set_to_zero();
    this->delta_world_f_core_psis.set_to_zero();
    this->current_speed = 0.0f;
    this->max_surface_rot_speed = 0.0f;
    this->rotation_axis_world_space.set(1.0f,0.0f,0.0f);

    //IVP_ASSERT( current_sim_man_slot <= 0 );
}

void IVP_Core::reset_time( IVP_Time offset){
    time_of_last_psi -= offset;
    for (int i = objects.len()-1; i>=0; i--){
	IVP_Real_Object *o = objects.element_at(i);
	o->reset_time( offset );
    }
}

void IVP_Core::core_add_link_to_obj(IVP_Real_Object *add_obj) {
	IVP_IF(1) {
	    for (int j = objects.len()-1; j>=0;j--){
		IVP_ASSERT(objects.element_at(j) != add_obj);
	    }
	}
	this->objects.add(add_obj);
}

void IVP_Core::unlink_obj_from_core_and_maybe_destroy(IVP_Real_Object *remove_obj)
{
  
    this->objects.remove(remove_obj);
    if(this->objects.len()==0) {
	// I'm not longer used
	if(IVP_MTIS_SIMULATED(movement_state)) {
	    this->stop_physical_movement();
	    //environment->core_sim_manager->remove_sim_core(this);
	}
   
	P_DELETE_THIS(this);
    }
}

void IVP_Core::stop_movement_without_collision_recheck() {
    //IVP_ASSERT(IVP_MTIS_SIMULATED(movement_state));
    this->movement_state=IVP_MT_NOT_SIM;
    this->speed.set_to_zero();
    this->rot_speed.set_to_zero();
    this->speed_change.set_to_zero();
    this->rot_speed_change.set_to_zero();
    this->delta_world_f_core_psis.set_to_zero();

    IVP_Event_Sim es(environment);
    calc_next_PSI_matrix_zero_speed(&es);
}

// because of not wanted interpolations on calm objects 
void IVP_Core::stop_physical_movement()
{
    stop_movement_without_collision_recheck();
    IVP_Time time = environment->get_current_time();
    for(int c = objects.len()-1;c>=0;c--){
	IVP_Real_Object *r_obj=this->objects.element_at(c);
	IVP_ASSERT(r_obj->get_movement_state()!=IVP_MT_STATIC);
	r_obj->set_movement_state(IVP_MT_NOT_SIM);
	environment->get_mindist_manager()->recheck_ov_element(r_obj);
	IVP_Hull_Manager *h_manager = r_obj->get_hull_manager(); 
	h_manager->exit_hull_for_simulation(time);
	if (r_obj->cache_object){
	    environment->get_cache_object_manager()->invalid_cache_object(r_obj);
	}
    }
}

void IVP_Core::reset_freeze_check_values() {
    IVP_Time time = environment->get_current_time();
    this->time_of_calm_reference[0] = time.get_time();
    this->time_of_calm_reference[1] = time.get_time();
}

// init core for calc_next_PSI_matrix
void IVP_Core::init_core_for_simulation(){
    IVP_ASSERT(movement_state == IVP_MT_NOT_SIM);
    movement_state = IVP_MT_MOVING;

    //speed_change.set_to_zero();
    //rot_speed_change.set_to_zero();
    
    IVP_Time time = environment->get_current_time();

    IVP_Time last_time = time - environment->get_delta_PSI_time();
    this->time_of_last_psi = last_time;

    reset_freeze_check_values();
    
    this->time_of_last_psi = time;
    for(int c = objects.len()-1;c>=0;c--){
	IVP_Real_Object *r_obj=this->objects.element_at(c);
	IVP_ASSERT(r_obj->get_movement_state()!=IVP_MT_STATIC);
	r_obj->set_movement_state(IVP_MT_MOVING);
	IVP_Hull_Manager *h_manager = r_obj->get_hull_manager(); 
	h_manager->init_hull_for_simulation();
	r_obj->get_all_near_mindists();
    }
}

/** sets rot_speed so that it is synchronized with interpolation
 *  ensure that calc_next_PSI_matrix is called prior to this method
 *  does not call calc_next_PSI_matrix */
void IVP_Core::synchronize_with_rot_z(){
    IVP_ASSERT(tmp_null.old_sync_info == NULL); 

    IVP_U_Memory *mem=environment->sim_unit_mem;
    void *p=mem->get_mem_transaction(sizeof(IVP_Old_Sync_Rot_Z));
    
    tmp_null.old_sync_info=(IVP_Old_Sync_Rot_Z*)p;
    tmp_null.old_sync_info->was_pushed_during_i_s=IVP_FALSE;
    
    IVP_Time current_time = environment->get_current_time();

    IVP_IF(1) {
        IVP_Debug_Manager *dm=environment->get_debug_manager();
	if(dm->file_out_impacts) {
	    fprintf(dm->out_deb_file,"doing_synchronize %lx at %f\n",0x0000ffff&(long)this,current_time.get_time());
	}
    }
    
    tmp_null.old_sync_info->old_sync_rot_speed. set(&rot_speed);
    tmp_null.old_sync_info->old_sync_q_world_f_core_next_psi = q_world_f_core_next_psi;

	IVP_U_Quat q_core_f_core;    q_core_f_core.set_invert_mult( &q_world_f_core_last_psi, &q_world_f_core_next_psi);
	inline_calc_at_quaternion( current_time, & q_world_f_core_next_psi );
	q_world_f_core_next_psi.set_matrix( &m_world_f_core_last_psi );
	inline_calc_at_position( current_time, & m_world_f_core_last_psi.vv );    

#ifdef IVP_FAST_WHEELS_ENABLED
	if (!this->rot_inertias_are_equal)
#endif
	{
	// new quaternion based solution to calculate the rotation speed:
	  IVP_DOUBLE f = 2.0f * i_delta_time;
	  rot_speed.set( f * IVP_Inline_Math::asind( q_core_f_core.x),
			 f * IVP_Inline_Math::asind( q_core_f_core.y),
			 f * IVP_Inline_Math::asind( q_core_f_core.z) );
	}
}

/****************************************
 * undos synchronize_with_rot_z (only if not calc_next_PSI_matrix is not called
 **************************/
void IVP_Core::undo_synchronize_rot_z() {    
    IVP_IF(1) {
        IVP_Debug_Manager *dm=environment->get_debug_manager();
	if(dm->file_out_impacts) {
	    //lwss - x64 fixes
	    //fprintf(dm->out_deb_file,"undoing_synchro %x at %f\n",0x0000ffff&(IVP_INT32)this,environment->get_current_time().get_time());
	    //lwss end
	}
    }
    rot_speed.      set(&tmp_null.old_sync_info->old_sync_rot_speed);
    q_world_f_core_next_psi = tmp_null.old_sync_info->old_sync_q_world_f_core_next_psi;
    tmp_null.old_sync_info=NULL;
    //q_world_f_core_last_psi .set_matrix(&m_world_f_core_last_psi);
}




IVP_Vec_PCore::IVP_Vec_PCore(const IVP_Core *pc,const IVP_U_Float_Point *p){
    pc->m_world_f_core_last_psi.vimult3(p,this);
}

void IVP_Core::calc_calc(){
    IVP_ASSERT(get_rot_inertia()->real_length() > P_DOUBLE_EPS);
    IVP_U_Float_Hesse *iri = (IVP_U_Float_Hesse *)get_inv_rot_inertia();

    const IVP_U_Float_Point *ri = get_rot_inertia();

    iri->set( 1.0f/ri->k[0],1.0f/ri->k[1], 1.0f/ri->k[2]);
    iri->hesse_val = 1.0f/get_mass();

    IVP_U_Float_Point diff_vec( iri->k[1] - iri->k[2],
				iri->k[2] - iri->k[0],
				iri->k[0] - iri->k[1]);

    IVP_DOUBLE qdiff = diff_vec.quad_length();
    IVP_DOUBLE qrlen = iri->quad_length();
    const IVP_DOUBLE eps = 0.1f;
    if ( qdiff < qrlen * (eps * eps) ){
        rot_inertias_are_equal = IVP_TRUE;
    }else{
        rot_inertias_are_equal = IVP_FALSE;
    }

    inv_object_diameter = 0.5f / upper_limit_radius;
}


void IVP_Core::transform_PSI_matrizes_core(const IVP_U_Matrix *m_core_f_core){
  m_world_f_core_last_psi.mmult4(m_core_f_core, &m_world_f_core_last_psi);
  pos_world_f_core_last_psi.set ( m_world_f_core_last_psi.get_position());
  q_world_f_core_last_psi.set_quaternion( &m_world_f_core_last_psi);
  q_world_f_core_next_psi = q_world_f_core_last_psi;
}

    
void IVP_Core::init(IVP_Real_Object *io){
    P_MEM_CLEAR(this);
    objects.reset();
    objects.add(io);
    environment = io->get_environment();

    //please change: unmoveable and not sim objs get no simulation unit -> no sim unit per default
    sim_unit_of_core = new IVP_Simulation_Unit();
    sim_unit_of_core->add_sim_unit_core( this );
    sim_unit_of_core->set_unit_movement_type(IVP_MT_NOT_SIM);
    this->add_core_controller(environment->standard_gravity_controller);
    //current_sim_man_slot = -1; //not simulated
    movement_state=IVP_MT_NOT_SIM;
}

IVP_Core::IVP_Core(IVP_Real_Object *io){
    init(io);
    environment->get_sim_units_manager()->add_sim_unit_to_manager(sim_unit_of_core);
}

IVP_Core::IVP_Core(IVP_Real_Object *io, const IVP_U_Quat *q_world_f_object_init, const IVP_U_Point *position, IVP_BOOL physical_unmoveable_, IVP_BOOL enable_piling_optimization)
{
    init(io);
    physical_unmoveable = physical_unmoveable_;
    // init_core_for_simulation();

    environment->get_sim_units_manager()->add_sim_unit_to_manager(sim_unit_of_core);

    q_world_f_core_last_psi = *q_world_f_object_init;
    q_world_f_core_last_psi.normize_quat();
    q_world_f_core_next_psi = q_world_f_core_last_psi;
    pos_world_f_core_last_psi.set(position);
    delta_world_f_core_psis.set_to_zero();

    // set redundant matrix
    q_world_f_core_last_psi.set_matrix(&m_world_f_core_last_psi);
    m_world_f_core_last_psi.vv.set(position);  
    
    set_radius( 1000.0f,1000.0f);
    this->set_fast_piling_allowed( enable_piling_optimization );
}

IVP_Core::~IVP_Core()
{
  this->environment->remove_revive_core(this);
  
  if(this->physical_unmoveable == IVP_TRUE) {
    if(this->core_friction_info.for_unmoveables.l_friction_info_hash != NULL) {
      P_DELETE( this->core_friction_info.for_unmoveables.l_friction_info_hash );
    }
  } else {
      int i;
      IVP_Controller *my_controller;
      for(i=controllers_of_core.len()-1;i>=0;i--) {
	  my_controller=controllers_of_core.element_at(i);
	  my_controller->core_is_going_to_be_deleted_event(this);
      }
  }
  if(this->sim_unit_of_core) {
      this->sim_unit_of_core->sim_unit_remove_core(this);
  }
  P_DELETE(spin_clipping);
}

IVP_Core_Merged::IVP_Core_Merged(IVP_Real_Object *real_obj): IVP_Core( real_obj ){
	this->objects.remove(real_obj);
}

IVP_Core_Merged::IVP_Core_Merged(IVP_Core *core0, IVP_Core *core1):
    IVP_Core(core0->objects.element_at(0))
{
    // Core Merge Constructor
    P_MEM_CLEAR(this); //warning: this is not so good (old values are destroyed, e.g. a vector object in IVP_Core)
    physical_unmoveable = (IVP_BOOL)(core0->physical_unmoveable | core1->physical_unmoveable);
    environment = core0->environment;

    movement_state=IVP_MT_MOVING;

    set_by_merge(core0, core1);
    IVP_Event_Sim es(environment);
    IVP_Calc_Next_PSI_Solver nps(this);
    nps.calc_next_PSI_matrix( &es, NULL );
}

// @@@ maybe not necessary each PSI !!!!
IVP_Movement_Type IVP_Core::calc_movement_state(IVP_Time psi_time) {

    // check short time movements
    while(1){	// break if object has moved
	IVP_DOUBLE qdist = pos_world_f_core_last_psi.quad_distance_to(&position_world_f_core_calm_reference[0]);
	if(qdist > IVP_MINIMAL_TRANS_SPEED_PER_PSI_SHORT * IVP_MINIMAL_TRANS_SPEED_PER_PSI_SHORT) break;

	IVP_DOUBLE qrot_sum = q_world_f_core_next_psi.inline_estimate_q_diff_to( &q_world_f_core_calm_reference[0] );
	if (qrot_sum * upper_limit_radius * upper_limit_radius > IVP_MINIMAL_ROT_SPEED_PER_PSI_SHORT * IVP_MINIMAL_ROT_SPEED_PER_PSI_SHORT) break;

	// don't check long term for slow objects
	if (psi_time - time_of_calm_reference[0] > environment->get_freeze_manager()->freeze_check_dtime){
	    return  IVP_MT_CALM;
	}
	return  IVP_MT_SLOW;
    }
    q_world_f_core_calm_reference[0].set(&q_world_f_core_next_psi);
    position_world_f_core_calm_reference[0].set( & pos_world_f_core_last_psi );
    this->time_of_calm_reference[0] = psi_time.get_time();
    
    
    while(1){	// break if object has moved
	IVP_DOUBLE q_dist = pos_world_f_core_last_psi.quad_distance_to(&position_world_f_core_calm_reference[1]);
	if(q_dist > IVP_MINIMAL_TRANS_SPEED_PER_PSI_LONG*IVP_MINIMAL_TRANS_SPEED_PER_PSI_LONG) break;

	IVP_DOUBLE qrot_sum = q_world_f_core_last_psi.inline_estimate_q_diff_to( &q_world_f_core_calm_reference[1] );
	if (qrot_sum * upper_limit_radius * upper_limit_radius > IVP_MINIMAL_ROT_SPEED_PER_PSI_LONG*IVP_MINIMAL_ROT_SPEED_PER_PSI_LONG) break;

	if (psi_time - time_of_calm_reference[1] > IVP_HAS_MOVED_LONG_TIME){
	    return  IVP_MT_CALM;	// enforce calm down after some seconds
	}
	return  IVP_MT_MOVING;
    }
    
    q_world_f_core_calm_reference[1].set(&q_world_f_core_last_psi);
    position_world_f_core_calm_reference[1].set( & pos_world_f_core_last_psi );
    this->time_of_calm_reference[1]= psi_time.get_time();

    return IVP_MT_MOVING; 
}



// create a new collision core
void IVP_Core::create_collision_merged_core_with(IVP_Core *other_core){
    IVP_ASSERT( other_core != this);
    return;
    IVP_Core_Collision *ccore = new IVP_Core_Collision(this,other_core);

    // update coordinates of all real objects
    {
	IVP_U_Matrix *m_world_f_core_merged = &ccore->m_world_f_core_last_psi;
	if ( this->m_world_f_core_last_psi.quad_distance_to(m_world_f_core_merged) > P_DOUBLE_EPS){
	    IVP_Core *core = this;
	    IVP_U_Matrix new_m_core_f_core;
	    m_world_f_core_merged->mimult4(&core->m_world_f_core_last_psi, & new_m_core_f_core);
	    for(int c = objects.len()-1;c>=0;c--){
		IVP_Real_Object *obj=this->objects.element_at(c);
		// 1. step, calc new_m_core_f_core
		IVP_U_Matrix new_m_object_f_core;
		IVP_U_Matrix old_m_core_f_object;
		obj->calc_m_core_f_object(&old_m_core_f_object);
		old_m_core_f_object.mimult4( &new_m_core_f_core, &new_m_object_f_core);
		new_m_object_f_core.orthonormize();
		obj->set_new_m_object_f_core(&new_m_object_f_core);
		ccore->objects.add(obj);
		obj->physical_core = ccore;
	    }
	}
	if ( other_core->m_world_f_core_last_psi.quad_distance_to(m_world_f_core_merged) > P_DOUBLE_EPS){
	    IVP_Core *core = other_core;
	    IVP_U_Matrix new_m_core_f_core;
	    m_world_f_core_merged->mimult4(&core->m_world_f_core_last_psi, & new_m_core_f_core);
	    for(int c2 = other_core->objects.len()-1;c2>=0;c2--){
		IVP_Real_Object *obj=other_core->objects.element_at(c2);
		// 1. step, calc new_m_core_f_core
		IVP_U_Matrix new_m_object_f_core;
		IVP_U_Matrix old_m_core_f_object;
		obj->calc_m_core_f_object(&old_m_core_f_object);

		old_m_core_f_object.mimult4( &new_m_core_f_core, &new_m_object_f_core);
		new_m_object_f_core.orthonormize();
		obj->set_new_m_object_f_core(&new_m_object_f_core);
		ccore->objects.add(obj);
		obj->physical_core = ccore;
	    }
	}
    }


    // set pointers 
    this->merged_core_which_replace_this_core = ccore;
    other_core->merged_core_which_replace_this_core = ccore;  
}



//////// split core and activate friction core !!!!!!!
void IVP_Core_Collision::split_collision_merged_core_next_PSI(){
    if (this->merged_core_which_replace_this_core != NULL) return;	// already touched
    
    return; //@@@@@
    // set original friction core speeds and position
#if 0
    {
	for(int c = objects.len()-1;c>=0;c--){
		IVP_Real_Object *obj=this->objects.element_at(c);
		IVP_USE(obj);
	}
    }
    //this is old unused code
    
    IVP_Core_Sim_Manager *sim_man = environment->get_core_sim_manager();
    sim_man->remove_sim_core(this);
    CORE
    IVP_Core *old_core = NULL;
    for (obj_search = this->first_object; obj_search; obj_search = next_obj){
	next_obj = obj_search->next_in_core;
	if (obj_search->friction_core == old_core) continue;		// search new core
	old_core = obj_search->friction_core;

	//// loop over all original cores
	IVP_U_Matrix m_CORE_f_core;
	old_core->set_matrizes_and_speed(this, &m_CORE_f_core);		// copy values from this
	old_core->merged_core_which_replace_this_core = NULL;
	
	IVP_Real_Object *obj;
	// search end of list
	for ( obj = obj_search; obj->next_in_core && obj->next_in_core->friction_core == old_core; obj = obj->next_in_core);

	// split linked lists
	if (obj->next_in_core) obj->next_in_core->prev_in_core = NULL;
	obj->next_in_core = NULL;

	// reset core coordinates of objects
	{
	    for (obj = old_core->first_object; obj; obj=obj->next_in_core){
		
		IVP_U_Matrix m_object_f_CORE;
		obj->m_object_f_core.mmult4( &m_CORE_f_core, &m_object_f_CORE);
		m_object_f_CORE.orthonormize();
		obj->set_new_m_object_f_core(&m_object_f_CORE);
		obj->physical_core = old_core;
	    }
	}

	//
	sim_man->add_sim_core(old_core);
    }
#endif    
}

void IVP_Core::set_matrizes_and_speed(IVP_Core_Merged *template_core, IVP_U_Matrix *m_CORE_f_core_out){

    /************** for all old cores do: ***********/
    // Note: Convention: all matrixnames for current time are in capitel letters
    
    IVP_ASSERT( template_core->merged_core_which_replace_this_core == NULL);
    
    ///// sum up m_world_now_f_world_when_merged
    IVP_U_Matrix m_WORLD_f_world;
	    // check for hierarchy and optimize when no hierarchy is given
    if (template_core == this->merged_core_which_replace_this_core){
	template_core->m_world_f_core_last_psi.mi2mult4( &template_core->m_world_f_core_when_created,
							 &m_WORLD_f_world);
    }else{
	m_WORLD_f_world.init();
	IVP_Core_Merged *core;
	for (core = this->merged_core_which_replace_this_core;
	     core;
	     core = this->merged_core_which_replace_this_core){
	    IVP_U_Matrix m_WORLD_f_world_one_core;
	    core->m_world_f_core_last_psi.mi2mult4( &core->m_world_f_core_when_created,
						    &m_WORLD_f_world_one_core);
	    m_WORLD_f_world_one_core.mmult4( & m_WORLD_f_world,
					     & m_WORLD_f_world);
	}
    }
    m_WORLD_f_world.orthonormize();
    
    /// calc m_CORE_f_core
    IVP_U_Matrix m_CORE_f_core;
    {
	IVP_U_Matrix m_WORLD_f_core;
	m_WORLD_f_world.mmult4( &this->m_world_f_core_last_psi,
				&m_WORLD_f_core);
	m_world_f_core_last_psi.mimult4( &m_WORLD_f_core,
					 &m_CORE_f_core);
    }
    *m_CORE_f_core_out = m_CORE_f_core;    
    
    // set matrizes
    m_WORLD_f_world.mmult4( &m_world_f_core_last_psi, &m_world_f_core_last_psi);

    /// set speed + rot_speed
    this->speed.set(&template_core->speed);
    IVP_U_Float_Point rot_speed_world;
    template_core->m_world_f_core_last_psi.vmult3(&template_core->speed, &rot_speed_world);
    this->m_world_f_core_last_psi.vimult3(&rot_speed_world, & this->rot_speed);
}


IVP_Core_Collision::IVP_Core_Collision(IVP_Core *core0, IVP_Core *core1)
    : IVP_Core_Merged(core0, core1){
	next_collision_core = NULL;
}


    
// update all mindist between objects of this core and all other objects with
// invalid mindist_event_already_done
// + sets this mindist_event_already_done
void IVP_Core::update_exact_mindist_events_of_core(){
    this->mindist_event_already_done = environment->mindist_event_timestamp_reference; //flag in core to avoid IVP_DOUBLE calculating
    for(int c = objects.len()-1;c>=0;c--){
	IVP_Real_Object *obj=this->objects.element_at(c);
	obj->update_exact_mindist_events_of_object();
    }
}

void IVP_Core::set_mass(IVP_FLOAT new_mass){
  IVP_DOUBLE factor = new_mass/ this->get_mass();
  IVP_U_Float_Hesse *ri = (IVP_U_Float_Hesse *)get_rot_inertia();
  ri->mult( factor );
  ri->hesse_val = new_mass;
  this->calc_calc();
}

void IVP_Core::core_plausible_check() {
    IVP_DOUBLE rot_change_len,trans_change_len;
    rot_change_len=rot_speed_change.real_length();
    trans_change_len=speed_change.real_length();
    IVP_ASSERT(rot_change_len<MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_change_len<MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(rot_change_len>-MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_change_len>-MAX_PLAUSIBLE_LEN);

    IVP_DOUBLE rot_len,trans_len;
    rot_len=rot_speed.real_length();
    trans_len=speed.real_length();
    IVP_ASSERT(rot_len<MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_len<MAX_PLAUSIBLE_LEN);    
    IVP_ASSERT(rot_len>-MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(trans_len>-MAX_PLAUSIBLE_LEN);    
}

void IVP_Core::rot_speed_plausible_check(const IVP_U_Float_Point *rot_speed_test){
    IVP_DOUBLE rot_len;
    rot_len = rot_speed_test->real_length();
    IVP_ASSERT(rot_len<MAX_PLAUSIBLE_LEN);
    IVP_ASSERT(rot_len>-MAX_PLAUSIBLE_LEN);
}


//return NULL when no friction_info is available for Friction_System
IVP_Friction_Info_For_Core *IVP_Core::get_friction_info(IVP_Friction_System *my_fr_system)
{
    if( this->physical_unmoveable == IVP_TRUE ) {
	IVP_Friction_Hash *my_hash=this->core_friction_info.for_unmoveables.l_friction_info_hash;
	if(my_hash) {
	    IVP_Friction_Info_For_Core *fr_info;
	    fr_info=my_hash->find_friction_info( my_fr_system );
#ifdef DEBUG_FRICTION_CONSISTENCY
	    IVP_IF( my_fr_system->l_environment->get_debug_manager()->check_fs ) {
	    IVP_Friction_Info_For_Core *test_info;
		test_info=NULL;
		int i;
		for(i=0;i<list_debug_hash.len();i++) {
		    IVP_Friction_Info_For_Core *search;
		    search=list_debug_hash.element_at(i);
		    if(search->l_friction_system == my_fr_system) {
			test_info=search;
			break;
		    }
		}
		IVP_ASSERT( test_info == fr_info );
	    }
#endif
	    return fr_info;
	} else {
	    return NULL;
	}
    } else {
	IVP_Friction_Info_For_Core *my_info=this->core_friction_info.for_moveables.moveable_core_friction_info;
	if(my_info) {
	    if(my_info->l_friction_system == my_fr_system) {
		return my_info;
	    }
	}
	return NULL;
    }
}


IVP_Friction_Info_For_Core *IVP_Core::moveable_core_has_friction_info() {
    IVP_ASSERT( this->physical_unmoveable == IVP_FALSE );
    return this->core_friction_info.for_moveables.moveable_core_friction_info;
}

void IVP_Core::add_friction_info(IVP_Friction_Info_For_Core *my_fr_info)
{
    if( this->physical_unmoveable == IVP_TRUE ) {
	if( this->core_friction_info.for_unmoveables.l_friction_info_hash == NULL ) {
	    this->core_friction_info.for_unmoveables.l_friction_info_hash = new IVP_Friction_Hash(2);
	}
	IVP_IF(1) {
	    IVP_ASSERT( get_friction_info(my_fr_info->l_friction_system) == NULL );
	}
	this->core_friction_info.for_unmoveables.l_friction_info_hash->add_friction_info( my_fr_info );
#ifdef DEBUG_FRICTION_CONSISTENCY
	IVP_IF(environment->get_debug_manager()->check_fs) {
	    this->list_debug_hash.add( my_fr_info );
	}
#endif
    } else {
	IVP_ASSERT( this->core_friction_info.for_moveables.moveable_core_friction_info == NULL );
	this->core_friction_info.for_moveables.moveable_core_friction_info = my_fr_info;
    }    
}

void IVP_Core::unmovable_core_debug_friction_hash() {
  for(int c = objects.len()-1;c>=0;c--){
    IVP_Real_Object *obj=this->objects.element_at(c);

    IVP_Synapse_Friction *fr_synapse;
    for(fr_synapse=obj->get_first_friction_synapse();fr_synapse;fr_synapse=fr_synapse->get_next()) {
      IVP_Contact_Point *fr_mindist=fr_synapse->get_contact_point();
      IVP_Friction_System *fr_sys = fr_mindist->l_friction_system;
      fr_sys = fr_sys;

      IVP_ASSERT( get_friction_info(fr_sys)->l_friction_system == fr_sys ); //error after deleting hash entry
    }
  }
  //printf("debug_friction_hash_ok\n");
}

void IVP_Core::unlink_friction_info(IVP_Friction_Info_For_Core *my_fr_info)
{
    IVP_Friction_Info_For_Core *my_info;
    if( this->physical_unmoveable == IVP_TRUE ) {
	my_info = this->core_friction_info.for_unmoveables.l_friction_info_hash->remove_friction_info( my_fr_info );
	IVP_ASSERT( my_info );
#ifdef DEBUG_FRICTION_CONSISTENCY
	IVP_IF( environment->get_debug_manager()->check_fs ) {
	    int i=0;
	    IVP_Friction_Info_For_Core*my_test_info=NULL;
	    while(my_info!=NULL) {
		IVP_ASSERT(i<list_debug_hash.len());
		if(list_debug_hash.element_at(i)==my_fr_info) {
		    my_test_info=my_fr_info;
		    list_debug_hash.remove_at(i);
		    break;
		}
		i++;
	    }
	    IVP_ASSERT( my_info==my_test_info );
	}
#endif
    } else {
	IVP_ASSERT( this->core_friction_info.for_moveables.moveable_core_friction_info == my_fr_info );
	this->core_friction_info.for_moveables.moveable_core_friction_info = NULL;
    }    
}

void IVP_Core::delete_friction_info(IVP_Friction_Info_For_Core *my_fr_info) {
    unlink_friction_info(my_fr_info);
    P_DELETE(my_fr_info);
}


void IVP_Core::ensure_core_to_be_in_simulation() {
    if(this->physical_unmoveable) {
	return;
    }
    
    if((this->movement_state==IVP_MT_NOT_SIM))
    {
	IVP_Environment *env=this->environment;
	//IVP_ASSERT( env->state==IVP_ES_AT );
	this->sim_unit_of_core->sim_unit_revive_for_simulation(env);
    } else {
	this->sim_unit_of_core->sim_unit_ensure_cores_movement();
    }
}


// TL: is this needed any longer ???
void IVP_Core::ensure_all_core_objs_in_simulation()
{
    IVP_ASSERT( !this->physical_unmoveable );
    for(int c = objects.len()-1;c>=0;c--){
	IVP_Real_Object *obj=this->objects.element_at(c);
	obj->ensure_in_simulation();
    }
};

// TL: is this needed any longer ???
void IVP_Core::ensure_all_core_objs_in_simulation_now()
{
    IVP_ASSERT( !this->physical_unmoveable );
    for(int c = objects.len()-1;c>=0;c--){
	IVP_Real_Object *obj=this->objects.element_at(c);
	obj->ensure_in_simulation_now();
    }
};
