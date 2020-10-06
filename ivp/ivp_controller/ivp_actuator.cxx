// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef WIN32
#	pragma implementation "ivp_actuator.hxx"
#endif
#include <ivp_physics.hxx>

#include <ivp_cache_object.hxx>
#include <ivp_solver_core_reaction.hxx>

#include <ivu_active_value.hxx>
#include <ivp_actuator.hxx>
#include <ivp_hull_manager.hxx>

#include <ivp_radar.hxx>
#include <ivp_radar_appl.hxx>

IVP_Template_Rot_Mot::IVP_Template_Rot_Mot(){
    P_MEM_CLEAR(this);
}

IVP_Template_Torque::IVP_Template_Torque(){
    P_MEM_CLEAR(this);
}

IVP_Template_Stabilizer::IVP_Template_Stabilizer(){
    P_MEM_CLEAR(this);
}
IVP_Template_Check_Dist::IVP_Template_Check_Dist(){
    P_MEM_CLEAR(this);
}



//////////////////////////
IVP_Template_Force::IVP_Template_Force(){
    P_MEM_CLEAR(this);
    push_first_object = IVP_TRUE;
    push_second_object = IVP_FALSE;
}

IVP_Template_Two_Point::IVP_Template_Two_Point(){
    P_MEM_CLEAR(this);
}

IVP_Template_Four_Point::IVP_Template_Four_Point(){
    P_MEM_CLEAR(this);
}

//////////////////////////

IVP_Extra_Info::IVP_Extra_Info()
{
    P_MEM_CLEAR(this);    
    return;
}

//////////////////////////

void IVP_Template_Anchor::set_anchor_position_ws(IVP_Real_Object *obj, const IVP_U_Point *coords_ws)
{
    this->object = obj;
    this->coords_world = *coords_ws;
    return;
}


void IVP_Template_Anchor::set_anchor_position_ws(IVP_Real_Object *obj, const IVP_DOUBLE x, const IVP_DOUBLE y, const IVP_DOUBLE z)
{
    this->object = obj;
    this->coords_world.k[0] = x;
    this->coords_world.k[1] = y;
    this->coords_world.k[2] = z;
    return;
}


void IVP_Template_Anchor::set_anchor_position_os(IVP_Real_Object *obj, const IVP_U_Float_Point *coords_os)
{
    this->object = obj;

    IVP_Cache_Object *cache = obj->get_cache_object_no_lock();
    cache->transform_position_to_world_coords(coords_os, &this->coords_world);

	return;
}


void IVP_Template_Anchor::set_anchor_position_os(IVP_Real_Object *obj, const IVP_DOUBLE x, const IVP_DOUBLE y, const IVP_DOUBLE z)
{
    this->object = obj;
    IVP_U_Float_Point coords_os(x, y, z);

    IVP_Cache_Object *cache = obj->get_cache_object_no_lock();
    cache->transform_position_to_world_coords(&coords_os, &this->coords_world);
}

#if 1
void IVP_Template_Anchor::set_anchor_position_cs(IVP_Real_Object *obj, const IVP_U_Float_Point *coords_cs)
{
    this->object = obj;
    
    IVP_U_Matrix mat;
	obj->calc_m_core_f_object(&mat);
	IVP_U_Float_Point coords_os;
	mat.vimult4(coords_cs, &coords_os);

	this->set_anchor_position_os(obj, &coords_os);
	return;
}


void IVP_Template_Anchor::set_anchor_position_cs(IVP_Real_Object *obj, const IVP_DOUBLE x, const IVP_DOUBLE y, const IVP_DOUBLE z)
{
    this->object = obj;

    IVP_U_Matrix mat;
	obj->calc_m_core_f_object(&mat);
	IVP_U_Float_Point coords_os;
	IVP_U_Float_Point coords_cs(x, y, z);
	mat.vimult4(&coords_cs, &coords_os);

	this->set_anchor_position_os(obj, &coords_os);
	return;
}

#endif

void IVP_Anchor::init_anchor(IVP_Actuator *ac, IVP_Template_Anchor *ta){
    this->l_anchor_object = ta->object;
    this->l_actuator = ac;
    IVP_U_Matrix m_core_f_object;
    l_anchor_object->calc_m_core_f_object(&m_core_f_object);

    IVP_Cache_Object *co = ta->object->get_cache_object_no_lock();
    IVP_U_Point obj_pos;
    co->transform_position_to_object_coords( &ta->coords_world, &obj_pos);
    this->object_pos.set(&obj_pos);
    m_core_f_object.vmult4(&object_pos,&core_pos);
    l_anchor_object->insert_anchor(this);
}


IVP_Actuator::IVP_Actuator(IVP_Environment *){
}

IVP_Actuator::~IVP_Actuator(){
}

void IVP_Actuator::anchor_will_be_deleted_event(IVP_Anchor *){
    // one end of actuator is broken -> remove actuator
    P_DELETE_THIS(this);
}

void IVP_Actuator::core_is_going_to_be_deleted_event(IVP_Core *) {
    P_DELETE_THIS(this);
}

///////////// TWO POINT //////////////

IVP_Actuator_Two_Point::IVP_Actuator_Two_Point(IVP_Environment *env,
					       IVP_Template_Two_Point *two_point_templ,
					       IVP_ACTUATOR_TYPE )
    : IVP_Actuator(env)
{
    this->client_data = two_point_templ->client_data;
    IVP_Anchor *anch0  = this->get_actuator_anchor(0);
    IVP_Anchor *anch1 =  this->get_actuator_anchor(1);
    anch0->init_anchor( this, two_point_templ->anchors[0] );
    anch1->init_anchor( this, two_point_templ->anchors[1] );

    //controller

    IVP_Core *core0,*core1;
    core0= anch0->l_anchor_object->get_core();
    core1= anch1->l_anchor_object->get_core();

    if(!core0->physical_unmoveable) {
        actuator_controlled_cores.add(core0);
    }
    if(!core1->physical_unmoveable) {
      if(core1!=core0) {
        actuator_controlled_cores.add(core1);
      }
    }
    core0->environment->get_controller_manager()->announce_controller_to_environment(this);
}



IVP_Actuator_Two_Point::~IVP_Actuator_Two_Point()
{
    IVP_Core *core0;
    core0 = get_actuator_anchor(0)->l_anchor_object->get_core();
    core0->environment->get_controller_manager()->remove_controller_from_environment(this,IVP_TRUE); //silently    
}

///////////// FOUR POINT //////////////

IVP_Actuator_Four_Point::IVP_Actuator_Four_Point(IVP_Environment *env,
					       IVP_Template_Four_Point *four_point_templ,
					       IVP_ACTUATOR_TYPE )
    : IVP_Actuator(env)
{
    this->client_data = four_point_templ->client_data;
  int i;
  for(i=0; i<4; i++){
    IVP_Anchor *anch = get_actuator_anchor(i);
    anch->init_anchor(this, four_point_templ->anchors[i]);

    IVP_Core *core = anch->l_anchor_object->get_core();
    if(core->physical_unmoveable) {
        actuator_controlled_cores.install(core);	
    }
  }
}



IVP_Actuator_Four_Point::~IVP_Actuator_Four_Point()
{
    IVP_Core *core0;
    core0 = get_actuator_anchor(0)->l_anchor_object->get_core();
    core0->environment->get_controller_manager()->remove_controller_from_environment(this,IVP_TRUE); //silently
}

///////////// FOUR POINT //////////////


/////////// CHECK DIST ///////////////

void IVP_Anchor_Check_Dist::init_anchor_check_dist(IVP_Real_Object *object, IVP_U_Point *position_world_space, IVP_Actuator_Check_Dist *i_act_check_dist){
    this->l_actuator_check_dist = i_act_check_dist;
    this->real_object = object;
    
    IVP_U_Matrix m_world_f_object;
    object->get_m_world_f_object_AT( &m_world_f_object);

    IVP_U_Point obj_pos;
    m_world_f_object.vimult4(position_world_space,&obj_pos);
    object_pos.set(&obj_pos);

}

void IVP_Anchor_Check_Dist::hull_limit_exceeded_event(IVP_Hull_Manager *, IVP_HTIME){
    IVP_Actuator_Check_Dist *act = this->l_actuator_check_dist;
    act->hull_limit_exceeded_event();
}

IVP_Actuator_Check_Dist::IVP_Actuator_Check_Dist(IVP_Environment *, IVP_Template_Check_Dist *templ ) 
{
    this->client_data = templ->client_data;
    this->mod_is_outside = templ->mod_is_outside;
    this->range = templ->range;

    
    this->anchors[0].init_anchor_check_dist(templ->objects[0],&templ->position_world_space[0],this); 
    this->anchors[1].init_anchor_check_dist(templ->objects[1],&templ->position_world_space[1],this); 
    this->is_outside = IVP_FALSE;
    
    IVP_Anchor_Check_Dist *anchor_0 = &anchors[0];
    IVP_Anchor_Check_Dist *anchor_1 = &anchors[1];
    IVP_Hull_Manager *hull_man_0 = anchor_0->real_object->get_hull_manager();
    IVP_Hull_Manager *hull_man_1 = anchor_1->real_object->get_hull_manager();

    hull_man_0->insert_lazy_synapse(anchor_0, 0, 0);
    hull_man_1->insert_lazy_synapse(anchor_1, 0, 0);

    this->hull_limit_exceeded_event(); // does all we want
}

void IVP_Actuator_Check_Dist::set_range( IVP_DOUBLE new_range){
    this->range = new_range;
    this->hull_limit_exceeded_event();
}


void IVP_Actuator_Two_Point::ensure_actuator_in_simulation(){
    get_actuator_anchor(0)->l_anchor_object->get_environment()->get_controller_manager()->ensure_controller_in_simulation(this);
}




/************************************************************/
/************************************************************/
/************************************************************/
/************************************************************/

void IVP_Actuator_Check_Dist::fire_check_dist_event(IVP_BOOL distance_shorter){
    for (int i = listeners_check_dist_event.len()-1;i>=0;i--){
	listeners_check_dist_event.element_at(i)->check_dist_event(this,distance_shorter);
    }
}

void IVP_Actuator_Check_Dist::fire_check_dist_is_going_to_be_deleted_event(){
    for (int i = listeners_check_dist_event.len()-1;i>=0;i--){
	listeners_check_dist_event.element_at(i)->check_dist_is_going_to_be_deleted_event(this);
    }
}

void IVP_Actuator_Check_Dist::hull_limit_exceeded_event(){
    // e.g. called by IVP_Anchor_Check_Dist->hull_limit_exceeded_event()

    /** calc current len **/
    IVP_U_Point pos_ws[2];
    for (int i = 0;i<2;i++){
	IVP_U_Matrix m_world_f_object;
	anchors[i].real_object->get_m_world_f_object_AT(&m_world_f_object);
	m_world_f_object.vmult4( &anchors[i].object_pos, &pos_ws[i]);
    }
    
    IVP_DOUBLE qlen = pos_ws[0].quad_distance_to(&pos_ws[1]);
    IVP_DOUBLE len = IVP_Inline_Math::ivp_sqrtf(qlen);

    /** check len vs range **/
    if(len < this->range){
	if(this->is_outside){
	    this->is_outside = IVP_FALSE;
	    this->fire_check_dist_event(IVP_FALSE);
	}
    } else {
	if( !this->is_outside){
	    this->is_outside = IVP_TRUE;
	    this->fire_check_dist_event(IVP_TRUE);
	}	
    } // is_outside, else

    /** eventually set active_floats **/
    if(this->mod_is_outside)	mod_is_outside->set_int(this->is_outside, IVP_TRUE); // delayed!
        
    /** re-insert in hull managers **/
    IVP_DOUBLE new_delta_hull = IVP_Inline_Math::fabsd(len - this->range) * 0.5f; // @@@ maybe unequal hull vals, dependant on cur sur speed

    IVP_Anchor_Check_Dist *anchor_0 = &anchors[0];
    IVP_Anchor_Check_Dist *anchor_1 = &anchors[1];
    IVP_Hull_Manager *hull_man_0 = anchor_0->real_object->get_hull_manager();
    IVP_Hull_Manager *hull_man_1 = anchor_1->real_object->get_hull_manager();

    IVP_Time t_now = anchor_0->real_object->get_environment()->get_current_time();
    hull_man_0->update_lazy_synapse(anchor_0, t_now, new_delta_hull);
    hull_man_1->update_lazy_synapse(anchor_1, t_now, new_delta_hull);
}


IVP_Actuator_Check_Dist::~IVP_Actuator_Check_Dist(){
    fire_check_dist_is_going_to_be_deleted_event();
    for (int i=0;i<2;i++){
	IVP_Hull_Manager *hull_man = anchors[i].real_object->get_hull_manager();
	hull_man->remove_synapse( &anchors[i] );
    }
}

IVP_Anchor_Check_Dist::~IVP_Anchor_Check_Dist(){
}

void IVP_Anchor_Check_Dist::hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *){
    delete l_actuator_check_dist;	// no P_DELETE ad this call deletes this also
}

///////////// FORCE ////////////////

IVP_Actuator_Force::IVP_Actuator_Force(IVP_Environment *env, IVP_Template_Force *templ
                                    ) : IVP_Actuator_Two_Point(	env,templ,	IVP_ACTUATOR_TYPE_FORCE )
{
    this->force = templ->force;
    this->push_first_object = templ->push_first_object;
    this->push_second_object = templ->push_second_object;
}

IVP_Actuator_Force_Active::IVP_Actuator_Force_Active(IVP_Environment *env,IVP_Template_Force *templ
				       ) : IVP_Actuator_Force(env,templ)
{
    this->active_float_force = templ->active_float_force;
    if(this->active_float_force){
	this->active_float_force->add_dependency(this);
	active_float_changed(active_float_force);
    }
    
}

void IVP_Actuator_Force::set_force(IVP_DOUBLE nforce){
    if (nforce == force) return;
    force = nforce;
    this->ensure_actuator_in_simulation();
}


void IVP_Actuator_Force_Active::active_float_changed(IVP_U_Active_Float *af)
{
    if (af == this->active_float_force){
	this->set_force(af->get_float_value());
    }
}


IVP_Actuator_Force::~IVP_Actuator_Force()
{
    ;
}

IVP_Actuator_Force_Active::~IVP_Actuator_Force_Active()
{
    // clean up active_values ...
    if (this->active_float_force){
	this->active_float_force->remove_dependency(this);
    };
}


IVP_DOUBLE IVP_Actuator_Extra::get_force()
{
    return info.active_float_force->give_double_value();
}

///////////// EXTRA ////////////////

IVP_Actuator_Extra::IVP_Actuator_Extra(IVP_Environment *env,
				     IVP_Template_Extra *templ
                                    ) : IVP_Actuator_Two_Point(
					env,templ,IVP_ACTUATOR_TYPE_ETC )
{
    this->info=templ->info;

    if(this->info.active_float_force)	info.active_float_force->add_dependency(this);
    if(this->info.active_float_bomb)	info.active_float_bomb->add_dependency(this);

    if(this->info.mod_fc_height)	info.mod_fc_height->add_dependency(this);
    if(this->info.mod_fc_target_height)	info.mod_fc_target_height->add_dependency(this);
    if(this->info.mod_fc_dist)	info.mod_fc_dist->add_dependency(this);
    if(this->info.mod_fc_speed)	info.mod_fc_speed->add_dependency(this);
    
    // floating camera

    this->current_float_cam_pos.set(0.0f, -100.0f, 0.0f); // start position
    this->current_look_point.set_to_zero(); // start position
    this->current_vals_are_set = IVP_FALSE; // flag for first time
    ensure_actuator_in_simulation();
}

IVP_U_Point *IVP_Actuator_Extra::get_float_cam_props(IVP_DOUBLE *fc_height_out, IVP_DOUBLE *fc_target_height_out, IVP_DOUBLE *fc_dist_out, IVP_DOUBLE *fc_speed_out)
{
    // returns current IVP_FLOAT cam position
    *fc_height_out = info.mod_fc_height->give_double_value();
    *fc_target_height_out = info.mod_fc_target_height->give_double_value();
    *fc_dist_out = info.mod_fc_dist->give_double_value();
    *fc_speed_out = info.mod_fc_speed->give_double_value();

    return &this->current_float_cam_pos;
}

IVP_Actuator_Extra::~IVP_Actuator_Extra()
{
     if (info.mod_fc_height) info.mod_fc_height->remove_dependency(this);
     if (info.mod_fc_target_height) info.mod_fc_target_height->remove_dependency(this);
     if (info.mod_fc_dist) info.mod_fc_dist->remove_dependency(this);
     if (info.mod_fc_speed) info.mod_fc_speed->remove_dependency(this);

     if (info.active_float_force) info.active_float_force->remove_dependency(this);
     if (info.active_float_bomb) info.active_float_bomb->remove_dependency(this);
     
    // @@@
}



///////////// ROT MOT ////////////////

IVP_Actuator_Rot_Mot::IVP_Actuator_Rot_Mot(IVP_Environment *env,
				     IVP_Template_Rot_Mot *templ
                                    ) : IVP_Actuator_Two_Point(
					env,templ,IVP_ACTUATOR_TYPE_ROT_MOT )
{
    this->max_rotation_speed = templ->max_rotation_speed;
    this->power = templ->power;
    this->max_torque = templ->max_torque;
    this->rot_speed_out = 0;
    this->active_float_rotation_speed_out = templ->active_float_rotation_speed_out;
    if (active_float_rotation_speed_out) active_float_rotation_speed_out->add_reference();
    // rotation motor
    
    IVP_Real_Object *obj0 = get_actuator_anchor(0)->l_anchor_object;
    IVP_Real_Object *obj1 = get_actuator_anchor(1)->l_anchor_object;
    if ( obj0->get_original_core() != obj1->get_original_core()){
	printf("Rot Mot Actuation has to be fixed on one object\n");
    }

    axis_in_core_coord_system.subtract(&get_actuator_anchor(1)->core_pos,&get_actuator_anchor(0)->core_pos);
    axis_in_core_coord_system.normize();
}


IVP_Actuator_Rot_Mot_Active::IVP_Actuator_Rot_Mot_Active(IVP_Environment *env,  IVP_Template_Rot_Mot *templ
							 ) : IVP_Actuator_Rot_Mot( env,templ )
{
    active_float_max_rotation_speed = templ->active_float_max_rotation_speed;
    active_float_power = templ->active_float_power;
    active_float_max_torque = templ->active_float_max_torque;

    
    if (active_float_max_rotation_speed){
	active_float_max_rotation_speed->add_dependency(this);
	set_max_rotation_speed( active_float_max_rotation_speed->get_float_value());
    }
    if (active_float_power){
	active_float_power->add_dependency(this);
	set_power( active_float_power->get_float_value());
    }
    if (active_float_max_torque){
	active_float_max_torque->add_dependency(this);
	set_max_torque( active_float_max_torque->get_float_value());
    }
}

void IVP_Actuator_Rot_Mot::set_max_rotation_speed( IVP_DOUBLE val){
    if (val == max_rotation_speed) return;
    max_rotation_speed = val;
    ensure_actuator_in_simulation();
}

void IVP_Actuator_Rot_Mot::set_power(IVP_DOUBLE val){
    if (val == power) return;
    power = val;
    ensure_actuator_in_simulation();
}

void IVP_Actuator_Rot_Mot::set_max_torque(IVP_DOUBLE val){
    if (val == max_torque) return;
    max_torque = val;
    ensure_actuator_in_simulation();
}

void IVP_Actuator_Rot_Mot_Active::active_float_changed(IVP_U_Active_Float *af){
    if (af == active_float_max_rotation_speed)	set_max_rotation_speed( active_float_max_rotation_speed->get_float_value());
    if (af == active_float_power)               set_power( active_float_power->get_float_value());
    if (af == active_float_max_torque)	        set_max_torque( active_float_max_torque->get_float_value());
}

IVP_Actuator_Rot_Mot_Active::~IVP_Actuator_Rot_Mot_Active(){
    if (active_float_max_rotation_speed) active_float_max_rotation_speed->remove_dependency(this);
    if (active_float_power) active_float_power->remove_dependency(this);
    if (active_float_max_torque) active_float_max_torque->remove_dependency(this);
}


IVP_Actuator_Rot_Mot::~IVP_Actuator_Rot_Mot()
{
    if (active_float_rotation_speed_out) active_float_rotation_speed_out->remove_reference();
}

void IVP_Actuator_Rot_Mot::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {
    if (power == 0) return;	// don't wake up object
  
    IVP_Real_Object *obj = get_actuator_anchor(0)->l_anchor_object;
    IVP_Core *core = obj->get_core();
    IVP_Core *orig_core = obj->get_original_core();

    IVP_U_Float_Point *axis;
    IVP_U_Float_Point h;
    if (core == orig_core){
	axis = &axis_in_core_coord_system;
    }else{
	const IVP_U_Matrix *m_w_f_c0 = get_actuator_anchor(0)->l_anchor_object->get_core()->get_m_world_f_core_PSI();
	const IVP_U_Matrix *m_w_f_c1 = get_actuator_anchor(1)->l_anchor_object->get_core()->get_m_world_f_core_PSI();
	IVP_U_Point pos1_ws;
	m_w_f_c1->vmult4( &get_actuator_anchor(1)->core_pos, &pos1_ws);
	IVP_U_Float_Point pos1_cs0;
	m_w_f_c0->vimult4( &pos1_ws, &pos1_cs0);
	
	h.subtract(&pos1_cs0,&get_actuator_anchor(0)->core_pos);
	h.normize();
	axis = &h;
    }

    IVP_DOUBLE domega = core->get_rot_speed_cs(axis);

    if (power * domega < 0){	// power in opposite direction
	domega = 0.0f;
    }
    this->rot_speed_out = domega; // memorize for sound e.g.
    if (active_float_rotation_speed_out){
	active_float_rotation_speed_out->set_double(rot_speed_out);
    }
    
    if (domega < 0.0f) domega = -domega;
    if (domega > max_rotation_speed){
	IVP_IF(0){
	    printf("Rot Mot rot speed threshold reached!\n");
	}
	return;	// too fast
    }
    if (domega < 0.1f) domega = 0.1f;
    
    
    IVP_DOUBLE rot_inertia_i = core->get_rot_inertia_cs(axis);
    
    IVP_DOUBLE rot_force = power /( domega * rot_inertia_i );

    // clip force
    while(max_torque != 0.0f){
	if( rot_force > max_torque){
	    IVP_IF(0){
		printf("RM Force clipped to %g, was %g\n", max_torque, rot_force);
	    }
	    rot_force = max_torque;
	    break;
	}
	if( rot_force < -max_torque){
	    IVP_IF(0){
		printf("RM Force clipped to %g, was %g\n", -max_torque, rot_force);
	    }
	    rot_force = -max_torque;
	}
        break;
    }
    IVP_DOUBLE rot_imp = rot_force * es->delta_time;
    core->async_rot_push_core_multiple_cs(axis, rot_imp);
}

///////////////////////

///////////// TORQUE ////////////////

IVP_Actuator_Torque::IVP_Actuator_Torque(IVP_Environment *env,    IVP_Template_Torque *templ
                                    ) : IVP_Actuator_Two_Point(
					env,	templ,	IVP_ACTUATOR_TYPE_TORQUE )
{
    this->max_rotation_speed = templ->max_rotation_speed;
    this->rot_speed_out = 0.0f;
    this->active_float_rotation_speed_out = templ->active_float_rotation_speed_out;
    if (active_float_rotation_speed_out) active_float_rotation_speed_out->add_reference();

    IVP_Real_Object *obj0 = get_actuator_anchor(0)->l_anchor_object;
    IVP_Real_Object *obj1 = get_actuator_anchor(1)->l_anchor_object;
    if ( obj0->get_original_core() != obj1->get_original_core()){
		printf("Both Anchors of a Torque_Actuator must be attached to just one object.\n");
		CORE;
    }
    axis_in_core_coord_system.subtract(&get_actuator_anchor(1)->core_pos,&get_actuator_anchor(0)->core_pos);
    axis_in_core_coord_system.normize();
    this->torque = templ->torque;	
    if (torque){
	ensure_actuator_in_simulation();
    }
}


IVP_Actuator_Torque_Active::IVP_Actuator_Torque_Active(IVP_Environment *env,  IVP_Template_Torque *templ
							 ) : IVP_Actuator_Torque( env,templ )
{
    active_float_max_rotation_speed = templ->active_float_max_rotation_speed;
    
    if (active_float_max_rotation_speed){
		active_float_max_rotation_speed->add_dependency(this);
		set_max_rotation_speed( active_float_max_rotation_speed->get_float_value());
    }

    active_float_torque = templ->active_float_torque;
    if (active_float_torque){
		active_float_torque->add_dependency(this);
		set_torque( active_float_torque->get_float_value());
    }
}

void IVP_Actuator_Torque::set_max_rotation_speed( IVP_DOUBLE val){
    if (val == max_rotation_speed) return;
    max_rotation_speed = val;
    ensure_actuator_in_simulation();
}

void IVP_Actuator_Torque::set_torque( IVP_DOUBLE val){
    if (val == torque) return;
    torque = val;
    ensure_actuator_in_simulation();
}

void IVP_Actuator_Torque_Active::active_float_changed(IVP_U_Active_Float *af){
    if (af == active_float_max_rotation_speed)	set_max_rotation_speed( af->get_float_value());
    if (af == active_float_torque)	            set_torque( af->get_float_value());
}

IVP_Actuator_Torque_Active::~IVP_Actuator_Torque_Active(){
    if (active_float_max_rotation_speed) active_float_max_rotation_speed->remove_dependency(this);
    if (active_float_torque) active_float_torque->remove_dependency(this);
}


IVP_Actuator_Torque::~IVP_Actuator_Torque()
{
    if (active_float_rotation_speed_out) active_float_rotation_speed_out->remove_reference();
}

void IVP_Actuator_Torque::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {
    // clear if not updated
	this->rot_speed_out = 0;
    if (this->torque == 0.0f){
	return;
    }

    IVP_Real_Object *obj = get_actuator_anchor(0)->l_anchor_object;
    IVP_Core *core = obj->get_core();

    if (!IVP_MTIS_SIMULATED(core->movement_state) || core->pinned){	// only rotate simulated cores
	return;
    }

    IVP_Core *orig_core = obj->get_original_core();

    IVP_U_Float_Point *axis;
    IVP_U_Float_Point h;
    if (core == orig_core){
		axis = &axis_in_core_coord_system;
    }else{
		h.subtract(&get_actuator_anchor(1)->core_pos,&get_actuator_anchor(0)->core_pos);
		h.normize();
		axis = &h;
    }

    IVP_DOUBLE domega = core->get_rot_speed_cs(axis);

    this->rot_speed_out = (IVP_FLOAT)domega; // memorize for sound e.g.
    if (active_float_rotation_speed_out){
	active_float_rotation_speed_out->set_double(rot_speed_out);
    }
    
    if (IVP_Inline_Math::fabsd(domega) > max_rotation_speed){
		return;	// too fast
    }
    IVP_DOUBLE rot_imp = torque * es->delta_time;
    core->async_rot_push_core_multiple_cs(axis, rot_imp);
}

///////////////////////


////////// ANCHOR //////////////


IVP_Anchor::~IVP_Anchor() {
    anchor_get_real_object()->remove_anchor(this);
}

void IVP_Anchor::object_is_going_to_be_deleted_event(IVP_Real_Object *obj){
    IVP_ASSERT( obj == anchor_get_real_object() );
    l_actuator->anchor_will_be_deleted_event(this);
}
////////// MANAGERS /////////////////



void IVP_Actuator_Extra::active_float_changed(IVP_U_Active_Float *)
{    
}

    
void IVP_Actuator_Force::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {
	
	IVP_DOUBLE force_val = this->force;
	if(force_val==0.0f) return; // nothing to push

	IVP_Anchor *anch0,*anch1;
	anch0= this->get_actuator_anchor(0);
	anch1= this->get_actuator_anchor(1);

	IVP_Real_Object *r_obj0,*r_obj1;
	r_obj0=anch0->anchor_get_real_object();
	r_obj1=anch1->anchor_get_real_object();

	IVP_Core *pc0 = r_obj0->physical_core;
	IVP_Core *pc1 = r_obj1->physical_core;
	
        IVP_U_Point dir_ws;
	IVP_U_Point pos0_ws;
	IVP_U_Point pos1_ws;

	pc0->get_m_world_f_core_PSI()->vmult4( &anch0->core_pos, &pos0_ws);
	pc1->get_m_world_f_core_PSI()->vmult4( &anch1->core_pos, &pos1_ws);
	dir_ws.subtract(&pos0_ws,&pos1_ws); //force direction world coords

	dir_ws.fast_normize();
	
	IVP_U_Float_Point force_dir;
	force_dir.set(&dir_ws);
	force_dir.mult(force_val * es->delta_time);

	if(this->push_first_object){	
		if (IVP_MTIS_SIMULATED(pc0->movement_state) && !pc0->pinned){ 
		    pc0->async_push_core_ws(&pos0_ws,&force_dir);
		}
	}

	if(this->push_second_object){
	    // also push at other end (reversed)
		if (IVP_MTIS_SIMULATED(pc1->movement_state) && !pc1->pinned){ 
		    IVP_U_Float_Point force_dir_rev;
		    force_dir_rev.set_multiple(&force_dir, -1);
		    pc1->async_push_core_ws(&pos1_ws,&force_dir_rev);
		}
	}
}

void IVP_Actuator_Extra::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {
    IVP_Actuator_Extra *extra=this;
	if(extra->info.is_puck_force){
	    extra->do_puck_force(es->delta_time);
	    return;
	}
	if(extra->info.is_float_cam ){
	    extra->do_float_cam(es->delta_time);
	    return;
	}	
}


void IVP_Actuator_Extra::do_puck_force(IVP_DOUBLE dtime)
{
	IVP_Anchor *anch0,*anch1;
	anch0= this->get_actuator_anchor(0);
	anch1= this->get_actuator_anchor(1);
	IVP_Core *pc0 = anch0->l_anchor_object->friction_core;
	IVP_Core *pc1 = anch1->l_anchor_object->friction_core;

	IVP_Real_Object *r_obj0,*r_obj1;
	r_obj0=anch0->anchor_get_real_object();
	r_obj1=anch1->anchor_get_real_object();
		
        IVP_U_Float_Point dir_ws;
	IVP_U_Point pos0_ws;
	IVP_U_Point pos1_ws;

	pc0->get_m_world_f_core_PSI()->vmult4( &anch0->core_pos, &pos0_ws);
	pc1->get_m_world_f_core_PSI()->vmult4( &anch1->core_pos, &pos1_ws);
	dir_ws.subtract(&pos0_ws,&pos1_ws); //force direction world coords
	dir_ws.k[1] = 0;
	dir_ws.fast_normize();
	    
	{
	    IVP_DOUBLE force = 0; /* (dlen - this->info.spring_len) * this->info.spring_constant; */

	    IVP_U_Float_Point force_vector;	// the force 
	    force_vector.set_multiple(&dir_ws, force * dtime);

	    if (IVP_MTIS_SIMULATED(pc1->movement_state) && !pc1->pinned){ 
		pc1->async_push_core_ws(&pos1_ws,&force_vector);
	    }   
	    if (IVP_MTIS_SIMULATED(pc0->movement_state) && !pc0->pinned){ 
		force_vector.mult(-1);
		pc0->async_push_core_ws(&pos0_ws,&force_vector);
	    }
	}
}





void IVP_Actuator_Extra::calc_float_cam_matrix(IVP_U_Matrix *cam_matrix_out)
{
    // calc camera matrix from current fc values
    IVP_ASSERT(this->info.is_float_cam);

    IVP_U_Point cam_dir;
    cam_dir.subtract(&this->current_look_point, &this->current_float_cam_pos);
    cam_dir.normize();
    
    IVP_U_Point v0, v1;

    IVP_U_Point hp;
    hp.set(0.05f, -1.0f, 0.05f); // to zenit
    v0.calc_cross_product(&cam_dir, &hp);
    v0.normize();
    
    v1.calc_cross_product(&cam_dir, &v0);
    v1.normize();

    // export calculated cam matrix
    cam_matrix_out->init_columns4(&v0, &v1, &cam_dir, &this->current_float_cam_pos);
}

void IVP_Actuator_Extra::do_float_cam(IVP_DOUBLE d_time)
{
    // could be called per psi

    IVP_U_Vector<IVP_Core> *my_cores=get_associated_controlled_cores();
    int i;
    for(i=my_cores->len()-1;i>=0;i--) {
	IVP_Core *my_core=my_cores->element_at(i);
	if(!my_core->physical_unmoveable) {
	    my_core->objects.element_at(0)->ensure_in_simulation();
	}
    }
    
    IVP_ASSERT(this->info.is_float_cam);

    IVP_DOUBLE fc_height, fc_target_height, fc_dist, fc_speed;
    IVP_U_Point *fc_pos = this->get_float_cam_props(&fc_height, &fc_target_height, &fc_dist, &fc_speed);

    // what direction does actuator currently take ?
    IVP_Anchor *anch0,*anch1;
    anch0= this->get_actuator_anchor(0);
    anch1= this->get_actuator_anchor(1);
    IVP_Core *pc0 = anch0->l_anchor_object->physical_core;
    IVP_Core *pc1 = anch1->l_anchor_object->physical_core;

    IVP_Real_Object *r_obj0,*r_obj1;
    r_obj0=anch0->anchor_get_real_object();
    r_obj1=anch1->anchor_get_real_object();

    IVP_U_Point dir_ws;
    IVP_U_Point pos0_ws;
    IVP_U_Point pos1_ws;
    
    pc0->get_m_world_f_core_PSI()->vmult4( &anch0->core_pos, &pos0_ws);
    pc1->get_m_world_f_core_PSI()->vmult4( &anch1->core_pos, &pos1_ws);
    dir_ws.subtract(&pos0_ws,&pos1_ws); //force direction world coords

    dir_ws.normize(); 
    dir_ws.k[1] = 0.0f; // proj on y    @@@ proj on gravitation vec

    // where does current camera want to be ?
    IVP_U_Point cur_pos;
    cur_pos.add_multiple(&pos0_ws, &dir_ws, fc_dist);
    cur_pos.k[1] -= fc_height; // @@@ grav vec

    // what position shall camera take ?
    // determine real speed using d_time
    this->current_float_cam_pos.set_interpolate(fc_pos, &cur_pos, fc_speed * d_time);

    // what direction shall camera look along ?

    // smoothing is necessary to avoid 'jumps'
    IVP_U_Point look_point;
    look_point.add_multiple(&pos0_ws, &dir_ws, 0.0f); // 5m  should be adjustable
    look_point.k[1] -= fc_target_height;
    if(this->current_vals_are_set == IVP_FALSE){
	this->current_vals_are_set = IVP_TRUE;
	this->current_look_point.set(&look_point);
    } else {
	this->current_look_point.set_interpolate(&current_look_point, &look_point, fc_speed); // @@@ or other (fix) speed ?
    }
}





///////////// STABILIZER ////////////////
///////////// STABILIZER ////////////////
///////////// STABILIZER ////////////////

IVP_Actuator_Stabilizer::IVP_Actuator_Stabilizer(IVP_Environment *env,
				     IVP_Template_Stabilizer *templ
                                    ) : IVP_Actuator_Four_Point(
					env,templ,IVP_ACTUATOR_TYPE_STABILIZER )
{
    stabi_constant = templ->stabi_constant;
}

IVP_Actuator_Stabilizer::~IVP_Actuator_Stabilizer(){
}

void IVP_Actuator_Stabilizer::set_stabi_constant(IVP_DOUBLE val){
    if (val == stabi_constant) return;
    stabi_constant = val;
    
    get_actuator_anchor(0)->l_anchor_object->ensure_in_simulation();
}

void IVP_Actuator_Stabilizer::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {

    IVP_Core *core[4];
    int i;
    for(i=0; i<4; i++){
      core[i] = get_actuator_anchor(i)->l_anchor_object->friction_core; // @@@OG friction_core???
    }
		
    IVP_U_Float_Point dir[2];
    IVP_U_Point pos0_ws[4];
    IVP_U_Point pos1_ws[4];
    IVP_DOUBLE dist[2];
    for(i=0; i<2; i++){
	IVP_Anchor *anch0,*anch1;
	anch0= this->get_actuator_anchor(i*2);
	anch1= this->get_actuator_anchor(i*2+1);

	IVP_Core *pc0 = anch0->l_anchor_object->friction_core;
	IVP_Core *pc1 = anch1->l_anchor_object->friction_core;

	pc0->get_m_world_f_core_PSI()->vmult4( &anch0->core_pos, &pos0_ws[i]);
	pc1->get_m_world_f_core_PSI()->vmult4( &anch1->core_pos, &pos1_ws[i]);
	dir[i].subtract(&pos0_ws[i],&pos1_ws[i]); //force direction world coords
	dist[i] = dir[i].real_length_plus_normize();
    }

    IVP_DOUBLE diff_dist = dist[1] - dist[0];

    IVP_DOUBLE force = diff_dist * this->stabi_constant * es->delta_time;
    
    for(i=0; i<2; i++){
      IVP_U_Float_Point force_vector; 
      force_vector.set_multiple(&dir[i], force);

      IVP_Core *pc0, *pc1;
      pc0 = core[i*2];
      pc1 = core[i*2+1];
      if (IVP_MTIS_SIMULATED(pc1->movement_state) && !pc1->pinned){ 
	pc1->async_push_core_ws(&pos1_ws[i], &force_vector);
      }   
      if (IVP_MTIS_SIMULATED(pc0->movement_state) && !pc0->pinned){ 
	force_vector.mult(-1);
	pc0->async_push_core_ws(&pos0_ws[i], &force_vector);
      }
      
      force *= -1; // reverted force direction for other anchor pair
    }
}

/////////////////

