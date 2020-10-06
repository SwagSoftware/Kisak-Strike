// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivp_object.hxx"
#	pragma implementation "ivp_real_object.hxx"
#endif
 
#include <ivp_physics.hxx>
#include <string.h>

#include <ivu_matrix_macros.hxx>
#include <ivu_float.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_sim_unit.hxx>
#include <ivp_physic_private.hxx>

#include <ivp_listener_object.hxx>
#include <ivp_templates.hxx>
#include <ivp_hull_manager.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_friction.hxx>
#include <ivp_actuator.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_debug_manager.hxx>
#include <ivp_i_friction_hash.hxx>
#include <ivp_i_controller_vhash.hxx>
#include <ivp_clustering_longrange.hxx>
#include <ivu_min_hash.hxx>
#include <ivp_radar.hxx>
#include <ivp_phantom.hxx>
#include <ivp_calc_next_psi_solver.hxx>


void IVP_Real_Object::change_nocoll_group_ident(const char *new_string ) {
    if (!new_string){
	nocoll_group_ident[0] = 0;
	return;
    }
    if (strlen(new_string) > IVP_NO_COLL_GROUP_STRING_LEN){
	CORE;
    }
    strncpy( nocoll_group_ident, new_string, IVP_NO_COLL_GROUP_STRING_LEN);
}

void IVP_Real_Object::set_pinned(IVP_BOOL is_pinned)
{
    IVP_Core* core = this->get_core();

    /**
     * Set the core pinned flag and zero the velocities
     */
    core->pinned = is_pinned;
    core->speed.set_to_zero();
    core->rot_speed.set_to_zero();

    if(is_pinned == IVP_TRUE)
    {
	/**
	 * Disable collision detection
	 */
	IVP_BOOL c_enabled = is_collision_detection_enabled();
	this->enable_collision_detection(IVP_FALSE);

	/**
	 * Pinning: Set the inertia tensor inverse and mass inverse to 0; this
	 * will make the object virtually of infinite mass (i.e. 1/mass ~= 0)
	 */
	core->inv_rot_inertia.set(0.0f, 0.0f, 0.0f);
	core->inv_rot_inertia.hesse_val = 0.0f;

	/**
	 * Reset collision detection
	 */
	this->enable_collision_detection(c_enabled);
    }
    else
    {
	/**
	 * Unpinning: Set the inertia tensor inverse to be the inverse of the
	 * inertia tensor, and the mass inverse to be the inverse of the mass.
	 * (i.e., restore the object mass values)
	 */
	const IVP_U_Float_Point* i = core->get_rot_inertia();
	core->inv_rot_inertia.set(1.0f / i->k[0], 1.0f / i->k[1], 1.0f / i->k[2]);
	core->inv_rot_inertia.hesse_val = 1.0f / core->get_mass();

        /**
	 * Rebuild mindists for this object
	 */
	this->get_environment()->get_mindist_manager()->recheck_ov_element(this);
    }
}

void IVP_Real_Object::change_fast_piling_allowed(IVP_BOOL bool_flag) {
    this->friction_core->set_fast_piling_allowed(bool_flag);
}

void IVP_Real_Object::change_mass( IVP_FLOAT new_mass ) {
    this->get_core()->set_mass(new_mass);
}

void IVP_Real_Object::change_unmovable_flag( IVP_BOOL flag ) {
    //printf("changing_unmov_flag to %d at time %f  csimfl %d  osimfl %d\n",flag,get_core()->environment->get_current_time(),this->get_core()->movement_state,this->object_movement_state);
    
    //friction info is stored differently in movables and unmovables, so destroy first
    IVP_Core *my_core=this->get_core();
    if( my_core->physical_unmoveable == flag ) {
	return;
    }
    int d;
    for(d = my_core->objects.len()-1;d>=0;d--) {
	IVP_Real_Object *r_obj=my_core->objects.element_at(d);
	r_obj->unlink_contact_points(IVP_FALSE);
    }
    //printf("for_umove_info %lx\n",(long)my_core->core_friction_info.for_unmoveables.l_friction_info_hash);
    if(my_core->physical_unmoveable) {
	P_DELETE(my_core->core_friction_info.for_unmoveables.l_friction_info_hash);
    } else {
	my_core->core_friction_info.for_moveables.moveable_core_friction_info=NULL;
    }
    if(my_core->movement_state < IVP_MT_NOT_SIM) {
        my_core->freeze_simulation_core();
    }
    
    my_core->physical_unmoveable = flag;
    if(flag==IVP_TRUE) {
	//switched from movable to unmovable
	//printf("switch_to_unmovable: controllers: %d\n",my_core->controllers_of_core.len());
	if(my_core->sim_unit_of_core) {
	    my_core->sim_unit_of_core->sim_unit_remove_core(my_core);
	    my_core->sim_unit_of_core=new IVP_Simulation_Unit();
	    my_core->sim_unit_of_core->add_sim_unit_core(my_core);
	    my_core->sim_unit_of_core->set_unit_movement_type(IVP_MT_NOT_SIM);
	    
	    //clear all controllers. CORRECT would be: give all controllers signal that core switched to unmovable
	    int ic;
	    for(ic=my_core->controllers_of_core.len()-1;ic>=0;ic--) {
		my_core->controllers_of_core.remove_at(ic);
	    }
	    
	    my_core->environment->get_sim_units_manager()->add_sim_unit_to_manager(my_core->sim_unit_of_core);
	    my_core->add_core_controller(my_core->environment->standard_gravity_controller);
	}	
    } else {
	//printf("switch_to_movable\n");
    }
}

void IVP_Real_Object::recompile_values_changed() {
    this->get_core()->values_changed_recalc_redundants();
}

void IVP_Real_Object::recompile_material_changed() {
    this->get_core()->values_changed_recalc_redundants();
}

void IVP_Real_Object::enable_collision_detection(IVP_BOOL enable){
    if (enable){
	if (!this->flags.collision_detection_enabled ){
	    get_environment()->get_mindist_manager()->enable_collision_detection_for_object(this);
	    this->flags.collision_detection_enabled = IVP_TRUE;
	}
    }else{
	if (this->flags.collision_detection_enabled ){
	    P_DELETE(this->ov_element);
	    this->flags.collision_detection_enabled = IVP_FALSE;
	    unlink_contact_points(IVP_TRUE); //do it silently (do not wake up objects)
	}
    }
}


void IVP_Real_Object::ensure_in_simulation_now(){
    // Note IVP_MT_STATIC != IVP_MT_NOT_SIM
    IVP_ASSERT( !this->get_core()->physical_unmoveable );
    if (this->physical_core->movement_state==IVP_MT_NOT_SIM){
	this->revive_object_for_simulation();
    }
};



void IVP_Real_Object::set_new_m_object_f_core( const IVP_U_Matrix *new_m_object_f_core){

    IVP_U_Matrix new_m_core_f_object; new_m_core_f_object.set_transpose(new_m_object_f_core);
    IVP_Anchor *a;
    for (a = anchors; a ; a= a->anchor_next_in_object){
	new_m_core_f_object.vmult4( &a->object_pos, &a->core_pos );
    }
    if ( new_m_core_f_object.vv.quad_length() < P_RES_EPS * P_RES_EPS){
	flags.shift_core_f_object_is_zero = IVP_TRUE;
	shift_core_f_object.set_to_zero();
    }else{
	shift_core_f_object.set( &new_m_core_f_object.vv );
	flags.shift_core_f_object_is_zero = IVP_FALSE;
    }
    const IVP_U_Matrix *m = new_m_object_f_core;
    if ( m->get_elem(0,0) == 1.0f && m->get_elem(1,1) == 1.0f && m->get_elem(2,2) == 1.0f ){
	P_DELETE( q_core_f_object );
    }else{
	if (!q_core_f_object) q_core_f_object = new IVP_U_Quat();
	q_core_f_object->set_quaternion(&new_m_core_f_object);
	q_core_f_object->normize_quat();
    }
    // invalid caches !!
    if (cache_object){
	get_environment()->get_cache_object_manager()->invalid_cache_object(this);			// removes cache_ledge_point too
    }

}


void IVP_Real_Object::set_new_quat_object_f_core( const IVP_U_Quat *new_quat_object_f_core, const IVP_U_Point *trans_object_f_core){
    IVP_U_Matrix m;
    new_quat_object_f_core->set_matrix(&m);
    m.vv.set(trans_object_f_core);
    set_new_m_object_f_core(&m);
}



void IVP_Real_Object::add_listener_collision(IVP_Listener_Collision *listener){
    environment->get_cluster_manager()->add_listener_collision(this,listener);
}

void IVP_Real_Object::remove_listener_collision(IVP_Listener_Collision *listener){
    environment->get_cluster_manager()->remove_listener_collision(this,listener);
}

void IVP_Real_Object::add_listener_object(IVP_Listener_Object *listener){
    environment->get_cluster_manager()->add_listener_object(this,listener);
}

void IVP_Real_Object::remove_listener_object(IVP_Listener_Object *listener){
    environment->get_cluster_manager()->remove_listener_object(this,listener);
}


void IVP_Real_Object::insert_anchor(IVP_Anchor *new_anchor)
{
    new_anchor->anchor_prev_in_object=NULL;
    new_anchor->anchor_next_in_object=this->anchors;
    if (this->anchors){
	this->anchors->anchor_prev_in_object=new_anchor;
    }
    this->anchors=new_anchor;
}

void IVP_Real_Object::remove_anchor(IVP_Anchor *destroy_anch)
{
    IVP_Anchor *previous=destroy_anch->anchor_prev_in_object;
    IVP_Anchor *following=destroy_anch->anchor_next_in_object;
    if(previous!=NULL)
    {
	previous->anchor_next_in_object=following;
    } else {
	this->anchors=following;
    }

    if(following!=NULL)
    {
	following->anchor_prev_in_object=previous;
    }
}

// update all mindists between this objects and other objects whose core has invalid 'mindist_event_already_done'
void IVP_Real_Object::update_exact_mindist_events_of_object() {
    IVP_Synapse_Real *syn, *syn_next;	// minimal dist may remove itself from list
    for (syn = this->get_first_exact_synapse(); syn; syn = syn_next){
	syn_next= syn->get_next();
	IVP_Mindist *mindist = syn->get_mindist();    
	IVP_Core *core0 = mindist->get_synapse(0)->get_object()->physical_core;
	IVP_Core *core1 = mindist->get_synapse(1)->get_object()->physical_core;
	//at least one core has already right time code
	if(core0->mindist_event_already_done != core1->mindist_event_already_done) {	  /// #+# OS: vector for recalc mindists
	  mindist->recalc_mindist();			// should not be needed as the current position has not changed
	  if (mindist->recalc_result == IVP_MDRR_OK){ 
	    mindist->update_exact_mindist_events(IVP_FALSE, IVP_EH_BIG_DELAY);	// check length, do not check for hull, update time manager ...
	  }
	}
    }    
}

IVP_BOOL IVP_Real_Object::disable_simulation() {    
    IVP_Core *core=get_core();
    if(core->physical_unmoveable) {
        return IVP_TRUE;
    }
    if(core->is_in_wakeup_vec) {
        core->environment->remove_revive_core(core);
    }
    
    if(core->movement_state < IVP_MT_NOT_SIM) {
           for(int c = core->objects.len()-1;c>=0;c--) {
	       IVP_Real_Object *r_obj=core->objects.element_at(c);

	       r_obj->unlink_contact_points(IVP_TRUE); //trick to prevent any friction systems to wake up my object again
           }           
	   
	   core->q_world_f_core_calm_reference[0].set(&core->q_world_f_core_next_psi);
           core->position_world_f_core_calm_reference[0].set( &core->pos_world_f_core_last_psi );
	   IVP_Time now_time=core->environment->get_current_time();
	   IVP_Time ref_time=now_time-20; //bad hack: this ensures that 'sim_unit_calc_movement_state' switches off simulation
           core->time_of_calm_reference[0] = ref_time;

	   core->sim_unit_of_core->do_sim_unit_union_find();

	   return core->sim_unit_of_core->sim_unit_calc_movement_state(core->environment);
    }
    return IVP_TRUE;
}

// called at AT-Time
// normally wake up at PSI-time not allowed (not allowed in simulate_sim_units loop)
void IVP_Real_Object::revive_object_for_simulation()
{
    if(this->get_movement_state() == IVP_MT_NOT_SIM){	// STATICS won't be revived too
        //IVP_Environment *env=this->get_environment();
	//IVP_ASSERT( env->state == IVP_ES_AT ); //TL: without this assert no controll
//	this->friction_core->init_core_for_simulation();

	this->friction_core->sim_unit_of_core->sim_unit_revive_for_simulation(this->friction_core->environment);
    }
    
}


IVP_Real_Object::IVP_Real_Object(IVP_Cluster *cluster,IVP_SurfaceManager *surface_manager_,
			       const IVP_Template_Real_Object *templ_obj, const IVP_U_Quat *q_world_f_object, const IVP_U_Point *position)
    : IVP_Real_Object_Fast(cluster, templ_obj)
{
    IVP_U_Quat q_world_f_object_static;
    if (!q_world_f_object){
	q_world_f_object = &q_world_f_object_static;
	q_world_f_object_static.init();
    }
//	ivp_message("IVP_Real_Object::IVP_Real_Object 0x%x - %s\n", this, templ_obj->get_name());
    
    exact_synapses = NULL;
    invalid_synapses = NULL;
    friction_synapses = NULL;
    surface_manager = surface_manager_;
    anchors = NULL;
    ov_element = NULL;			// will be set by IVP_Mindist_Manager::enable_collision_detection
    ((int *) &flags)[0] = 0;
    q_core_f_object = NULL;
    shift_core_f_object.set_to_zero();
    cache_object = NULL;
    controller_phantom = NULL;

    strncpy(nocoll_group_ident, templ_obj->get_nocoll_group_ident(),IVP_NO_COLL_GROUP_STRING_LEN);
    physical_core = new IVP_Core(this, q_world_f_object, position, templ_obj->physical_unmoveable ,templ_obj->enable_piling_optimization);
    this->friction_core=physical_core;
    this->original_core=physical_core;

    if (templ_obj->physical_unmoveable){
	this->set_movement_state(IVP_MT_STATIC);
    }else{
	this->set_movement_state(IVP_MT_NOT_SIM);
    }
    
    this->l_default_material = templ_obj->material;
//    IVP_ASSERT(l_default_material);

    physical_core->environment->get_cluster_manager()->add_object(this);

    this->client_data = templ_obj->client_data;
}

void IVP_Real_Object::set_new_surface_manager( IVP_SurfaceManager *new_sm){
    IVP_BOOL c_enabled = is_collision_detection_enabled();
    this->enable_collision_detection(IVP_FALSE);

    this->surface_manager = new_sm;

    IVP_FLOAT rad, rad_dev;
    
    IVP_OBJECT_TYPE type = get_type();
    switch (type){
    case IVP_POLYGON:
	{
	    IVP_U_Float_Point center;
	    center.set_negative( &this->shift_core_f_object);
	    surface_manager->get_radius_and_radius_dev_to_given_center(&center, &rad, &rad_dev);
	}
	break;
    case IVP_BALL:
	rad = 0.0f;
	rad_dev = 0.0f;
	break;
    default:
	CORE;
    }
    rad += this->get_extra_radius();

    if ( rad > get_core()->upper_limit_radius){
	get_core()->upper_limit_radius = rad;
    }
    if ( rad_dev > get_core()->max_surface_deviation){
	get_core()->max_surface_deviation = rad_dev;
    }
    this->enable_collision_detection( c_enabled);
}

void IVP_Real_Object::recalc_core_radius( ){

    IVP_FLOAT rad, rad_dev;
    
    IVP_OBJECT_TYPE type = get_type();
    switch (type){
    case IVP_POLYGON:
	{
	    IVP_U_Float_Point center;
	    center.set_negative( &this->shift_core_f_object);
	    surface_manager->get_radius_and_radius_dev_to_given_center(&center, &rad, &rad_dev);
	}
	break;
    case IVP_BALL:
	rad = rad_dev = this->shift_core_f_object.real_length();
	break;
    default:
	CORE;
    }
    rad += this->get_extra_radius();

    if ( rad > get_core()->upper_limit_radius){
	get_core()->upper_limit_radius = rad;
    }
    if ( rad_dev > get_core()->max_surface_deviation){
	get_core()->max_surface_deviation = rad_dev;
    }
}

void IVP_Real_Object::set_extra_radius( IVP_DOUBLE new_radius ){
	if ( new_radius > this->extra_radius){
		this->extra_radius = new_radius;
		this->recalc_core_radius();
	}else{
		this->extra_radius = new_radius;
		this->get_core()->upper_limit_radius = 0.0f;
		this->get_core()->max_surface_deviation = 0.0f;
		for ( int i = this->get_core()->objects.len()-1;i>=0; 	i--){
			IVP_Real_Object *o = this->get_core()->objects.element_at(i);
			o->recalc_core_radius();
		}
	}
}



IVP_Real_Object::~IVP_Real_Object()
{
    P_DELETE(controller_phantom);
    get_hull_manager()->delete_hull_manager(); // remove all internal 
    clear_internal_references();    // anchors and mindists

    {	// fire global 'object deleted' event
	IVP_Event_Object event_deleted;
	event_deleted.real_object = this;
	event_deleted.environment = environment;
	environment->fire_event_object_deleted(&event_deleted);
    }

    if (this->flags.collision_listener_exists){	// local friction listener
	environment->get_cluster_manager()->fire_event_collision_object_deleted(this);
    }

    if (this->flags.object_listener_exists){	// fire object-dependant 'object deleted' event
	IVP_Event_Object event_deleted;
	event_deleted.real_object = this;
	event_deleted.environment = environment;
	environment->get_cluster_manager()->fire_event_object_deleted(&event_deleted);
    }
    
    physical_core->environment->get_cluster_manager()->remove_object(this);

    // from this point, only internal (callback free) memory frees
    if (cache_object){
	get_environment()->get_cache_object_manager()->invalid_cache_object(this);
    }
    
    physical_core->unlink_obj_from_core_and_maybe_destroy(this);

    if(original_core!=physical_core) {
	original_core->unlink_obj_from_core_and_maybe_destroy(this);
    }

    if((friction_core!=original_core) && (friction_core!=physical_core)) {
	friction_core->unlink_obj_from_core_and_maybe_destroy(this);
    }

    P_DELETE(q_core_f_object);
}

void IVP_Real_Object::reset_time( IVP_Time offset){
    get_hull_manager()->reset_time(offset);
}

/**************************************************************************************
 *	Name:	    	delete_and_check_vicinity 	
 *	Description:	deletes object and revives surroundings
 **************************************************************************************/
void IVP_Real_Object::delete_and_check_vicinity(){
    if (!this) return;

    IVP_Core *my_core=this->get_core();
    if(!my_core->physical_unmoveable) {
        my_core->sim_unit_of_core->sim_unit_ensure_in_simulation();
    }
    revive_nearest_objects_grow_fs();
    IVP_Real_Object *tmp_o=this;
    P_DELETE(tmp_o);
}

    
/**************************************************************************************
 *	Name:	    	delete_silently 	
 *	Description:	Silently deletes object, vicinity will stay frozen
 **************************************************************************************/
void IVP_Real_Object::delete_silently(){
    IVP_Real_Object *tmp_o=this;
    P_DELETE(tmp_o);
}



void IVP_Real_Object::recalc_invalid_mindists_of_object() {
    IVP_Synapse_Real *first_syn,*next_syn;

    for ( first_syn = invalid_synapses; first_syn; first_syn = next_syn){
        next_syn=first_syn->get_next();

	IVP_Mindist *mdist = first_syn->get_mindist();

	IVP_ASSERT (mdist->mindist_function == IVP_MF_COLLISION);

	mdist->recalc_invalid_mindist();
	if (mdist->recalc_result == IVP_MDRR_INTRUSION){
	  continue;
	}

	IVP_Mindist_Manager *mm = environment->get_mindist_manager();
	mm->remove_invalid_mindist(mdist);
	mm->insert_exact_mindist(mdist);

	if (mdist->get_length() < 0){
	  continue;
	}
    }
}



void IVP_Real_Object::recalc_exact_mindists_of_object() {
    IVP_Synapse_Real *first_syn,*next_syn;
    IVP_Mindist_Manager *mm = environment->get_mindist_manager();
    
    for ( first_syn=exact_synapses; first_syn; first_syn = next_syn){      
      next_syn=first_syn->get_next();
      IVP_Mindist *mdist = first_syn->get_mindist();
      mm->recalc_exact_mindist( mdist );	
    }
}

void IVP_Real_Object::get_all_near_mindists() {
    IVP_Movement_Type temp_movement= (IVP_Movement_Type)physical_core->movement_state;
    
    physical_core->movement_state=IVP_MT_GET_MINDIST;
    physical_core->environment->get_mindist_manager()->recheck_ov_element(this);
    
    this->physical_core->movement_state=temp_movement;
}

void IVP_Real_Object::recheck_collision_filter(){
    if (this->ov_element){
	physical_core->environment->get_mindist_manager()->recheck_ov_element(this);
    }
}

void IVP_Real_Object::revive_nearest_objects_grow_fs() {
  if(this->get_core()->physical_unmoveable) {
    this->get_all_near_mindists();
    this->get_core()->grow_friction_system();
    this->get_core()->revive_adjacent_to_unmoveable();
  } else {
    this->get_core()->ensure_core_to_be_in_simulation();
  }
}

void IVP_Real_Object::unlink_contact_points(IVP_BOOL silent) {
    // then delete static friction datas @@@OS, loop only once + flag 
    int debug_once=0;
    IVP_Synapse_Friction *fr_synapse;
    while((fr_synapse=this->get_first_friction_synapse())!=NULL) {
	IVP_Contact_Point *fr_mindist=fr_synapse->get_contact_point();
        IVP_Friction_System *fr_sys=fr_mindist->l_friction_system;
	IVP_IF(debug_once) {
	    debug_once=1;
	    IVP_Synapse_Friction *debug_syn=this->get_first_friction_synapse();
	    printf("still_left_fd ");
	    while(debug_syn) {
		printf("fd %lx sys %lx  ",(long)debug_syn->get_contact_point(),
		    (long)debug_syn->get_contact_point()->l_friction_system); 
		debug_syn=debug_syn->get_next();
	    }
	    printf("\n");
	    fr_sys->debug_fs_out_ascii();
	    printf("newl\n");
	}
	IVP_Core *core0,*core1;
	core0=fr_mindist->get_synapse(0)->l_obj->friction_core;
	core1=fr_mindist->get_synapse(1)->l_obj->friction_core;
	if(!silent) {
	    core0->ensure_core_to_be_in_simulation();
	    core1->ensure_core_to_be_in_simulation();
	}
	fr_sys->delete_friction_distance(fr_mindist);
	if(fr_sys->friction_dist_number==0) {
	    P_DELETE(fr_sys);
	}
    }
}

//lwss add
void IVP_Real_Object::unlink_contact_points_for_object(IVP_Real_Object *object)
{
    IVP_Synapse_Friction *fr_synapse;
    fr_synapse = this->get_first_friction_synapse();
    while( fr_synapse )
    {
        IVP_Contact_Point *fr_mindist = fr_synapse->get_contact_point();
        IVP_Friction_System *fr_sys = fr_mindist->l_friction_system;

        if( fr_synapse->get_object() == object ) // TODO: there is another OR if-clause here.. unsure which one it is. matches another obj
        {
            fr_sys->delete_friction_distance( fr_mindist );
            if(fr_sys->friction_dist_number==0)
            {
                P_DELETE(fr_sys);
            }
        }
        fr_synapse = fr_synapse->get_next();
    }
}

void IVP_Real_Object::force_grow_friction_system()
{
    //lwss hack
    //TODO: this just allocates memory early? leave for now
}
//lwss end

void IVP_Real_Object::clear_internal_references() {
    // first clear actuators
    IVP_Anchor *my_anchor;
    IVP_Anchor *last_anchor = 0;
    while( (my_anchor=this->get_first_anchor()) != NULL) {
	my_anchor->object_is_going_to_be_deleted_event(this);
	if (my_anchor == last_anchor){ // check for errors in anchors
	    CORE;
	}
	last_anchor = my_anchor;
    }
    
    unlink_contact_points(IVP_TRUE); //do it silently (do not wake up objects)
    
    IVP_Synapse_Real *fr_synapse;
    while((fr_synapse=this->get_first_exact_synapse())!=NULL) {
	IVP_Mindist *my_mindist=fr_synapse->get_mindist();
	P_DELETE(my_mindist);
    }
}


IVP_Object::IVP_Object(IVP_Cluster *father, const IVP_Template_Object *templ){
    this->init(father->environment);
    father->add_object(this);
    this->name = p_strdup(templ->get_name());
}


IVP_Object::IVP_Object(IVP_Environment *env){
    this->init(env);
}

void IVP_Object::init(IVP_Environment *env){
    next_in_cluster = prev_in_cluster = father_cluster = NULL;
    environment = env;
    name=0;
	this->set_type(IVP_NONE);
}

IVP_Object::~IVP_Object(){
    if (father_cluster){
	this->father_cluster->remove_object(this);
    }
    P_FREE(this->name);
	IVP_ASSERT(environment);
	environment = NULL;
}

void IVP_Real_Object::calc_m_core_f_object( IVP_U_Matrix *m_core_f_object )
{
    m_core_f_object->init();

    if ( q_core_f_object )
	{
		q_core_f_object->set_matrix( m_core_f_object );
    }
	else
	{
		m_core_f_object->init();
    }

    m_core_f_object->vv.set( &shift_core_f_object );
}

void   IVP_Real_Object::get_m_world_f_object_AT   (IVP_U_Matrix *m_world_f_object_out) const
   { calc_at_matrix(get_environment()->get_current_time(), m_world_f_object_out); };
    
void   IVP_Real_Object::get_quat_world_f_object_AT(IVP_U_Quat *quat_world_f_object, IVP_U_Point *position) const // ditto (using quaternions)
    {	calc_at_quaternion(get_environment()->get_current_time(), quat_world_f_object, position);   }


IVP_Cluster::IVP_Cluster(IVP_Cluster *father, IVP_Template_Cluster *templ): IVP_Object(father, templ) {
    objects = NULL;
	this->set_type(IVP_CLUSTER);
};

IVP_Cluster::IVP_Cluster(IVP_Environment *env): IVP_Object(env){
   objects = NULL;
   this->set_type(IVP_CLUSTER);
};

IVP_Cluster::~IVP_Cluster(){	// silently deletes a cluster
    while(objects){
	delete objects;
//	objects->delete_silently();
    }
}

void IVP_Cluster::add_object(IVP_Object *object){
    object->next_in_cluster = objects;
    object->prev_in_cluster = NULL;
    if (objects != NULL){
	objects->prev_in_cluster = object;
    }
    objects = object;
    object->father_cluster = this;
};


void IVP_Cluster::remove_object(IVP_Object *object){
    if ( object->prev_in_cluster == NULL){
	objects = object->next_in_cluster;
    }else{
	object->prev_in_cluster->next_in_cluster = object->next_in_cluster;
    }

    if ( object->next_in_cluster != NULL){
	object->next_in_cluster->prev_in_cluster = object->prev_in_cluster;
    }
};


void IVP_Real_Object::calc_at_matrix(IVP_Time current_time,IVP_U_Matrix	*m_world_f_object_out ) const
{
    IVP_U_Quat		q_world_f_object;
    IVP_Core *core = get_core();
    core->inline_calc_at_quaternion(current_time, &q_world_f_object);
    core->inline_calc_at_position(current_time, &m_world_f_object_out->vv);
    q_world_f_object.set_matrix( m_world_f_object_out );

    if (!flags.shift_core_f_object_is_zero){
	m_world_f_object_out->vmult4( &this->shift_core_f_object , &m_world_f_object_out->vv);
    }
    
    if ( this->q_core_f_object){
 	q_world_f_object.inline_set_mult_quat( &q_world_f_object, this->q_core_f_object );
	q_world_f_object.set_matrix( m_world_f_object_out );
    }
}

void IVP_Real_Object::calc_at_quaternion(IVP_Time      current_time,
					  IVP_U_Quat  *q_world_f_object_out,
					  IVP_U_Point *position_out) const
{
    IVP_Core *core = this->get_core();
    core->inline_calc_at_quaternion(current_time, q_world_f_object_out);
    core->inline_calc_at_position(current_time, position_out);

    if (!flags.shift_core_f_object_is_zero){
	IVP_U_Matrix m_world_f_object;
	q_world_f_object_out->set_matrix( &m_world_f_object );
	m_world_f_object.vv.set(position_out);
	m_world_f_object.vmult4( &this->shift_core_f_object , position_out);
    }
    
    if ( this->q_core_f_object){
	q_world_f_object_out->inline_set_mult_quat( q_world_f_object_out, this->q_core_f_object );
    }
}




void IVP_Real_Object::init_object_core(IVP_Environment *i_environment, const IVP_Template_Real_Object *templ){
    // used for instances
    IVP_U_Float_Point center;
    IVP_U_Quat mass_axis;
    IVP_U_Matrix m_object_f_core;
    mass_axis.init();
    
    IVP_OBJECT_TYPE type = get_type();
    
    if (templ->mass_center_override){
	center.set(templ->mass_center_override->get_position());
    }else{
	surface_manager->get_mass_center(&center);
    }
    
    mass_axis.set_matrix(&m_object_f_core);   
    m_object_f_core.vv.set(&center);
    
    set_new_m_object_f_core( &m_object_f_core );

    IVP_Core *core = get_core();
    // radius

    IVP_FLOAT rad, rad_dev;
    switch (type){
    case IVP_POLYGON:
	surface_manager->get_radius_and_radius_dev_to_given_center(&center, &rad, &rad_dev);
	break;
    case IVP_BALL:
	rad = 0.0f;
	rad_dev = 0.0f;
	break;
    default:
	CORE;
    }
    rad += this->get_extra_radius();
    
    core->set_radius(rad,rad_dev);

    IVP_DOUBLE mass = templ->mass;
    if (templ->mass < P_RES_EPS){
	mass = 1.0f;
    }
    // mass
    IVP_U_Float_Hesse &rot_inertia = (IVP_U_Float_Hesse &)* core->get_rot_inertia();

    if (templ->rot_inertia_is_factor){
	IVP_U_Float_Point rot_in;
	switch (type){
	case IVP_POLYGON:
	    surface_manager->get_rotation_inertia( &rot_in );
	    break;
	case IVP_BALL:{
	    IVP_DOUBLE rot = (2.0f/5.0f)* rad * rad;
	    rot_in.set(rot,rot,rot);
	    break;
	}
	default:
	    CORE;
	}
	    
	rot_inertia.set_pairwise_mult( &rot_in, &templ->rot_inertia);
	rot_inertia.mult(mass);
    }else{
	rot_inertia.set(&templ->rot_inertia);
    }

    if( templ->auto_check_rot_inertia != 0.0f ) {
        IVP_FLOAT min_inertia_value = rot_inertia.real_length() * templ->auto_check_rot_inertia;
	int hh;
        for(hh=0;hh<3;hh++) {
	    if(rot_inertia.k[hh]<min_inertia_value) {
	        rot_inertia.k[hh]=min_inertia_value;
	    }
	}
    }

    rot_inertia.hesse_val = mass;


    // speed damps
    core->speed_damp_factor = (IVP_FLOAT)templ->speed_damp_factor;
    core->rot_speed_damp_factor.set( &templ->rot_speed_damp_factor);
    

    IVP_U_Matrix m_core_f_object;
    m_core_f_object.set_transpose(&m_object_f_core);

    core->rot_speed.set_to_zero();
    core->speed.set_to_zero();

    core->calc_calc();
    core->transform_PSI_matrizes_core(&m_object_f_core);
    IVP_Event_Sim es( i_environment, i_environment->get_next_PSI_time() - i_environment->get_current_time());
    core->calc_next_PSI_matrix_zero_speed( &es );
}


int IVP_Real_Object::get_collision_check_reference_count(){ // see IVP_Universe_Manager for details
    IVP_OV_Element *ov = get_ov_element();
    if (!ov) return -1;		// reference count for non colliding objects
    return ov->collision_fvector.len();
}

void IVP_Real_Object::get_geom_center_world_space(IVP_U_Point *geom_center_out) const{
    const IVP_U_Matrix *m_world_f_core = get_core()->get_m_world_f_core_PSI();
    geom_center_out->set(&m_world_f_core->vv);
}

IVP_FLOAT IVP_Real_Object::get_geom_radius() const{
    return get_core()->upper_limit_radius;
}

IVP_FLOAT IVP_Real_Object::get_geom_center_speed()const{
    return get_core()->speed.real_length();
}

void IVP_Real_Object::get_geom_center_speed_vec(IVP_U_Point *speed_ws_out) const {
    speed_ws_out->set(&get_core()->speed);
}

IVP_Controller_VHash::~IVP_Controller_VHash() 
{
#if 0 /*TL  */
    for (int i=this->len()-1; i>=0; i--) {
	IVP_Controller *controller;
	controller = (IVP_Controller *)this->element_at(i);
	if (controller){
	    controller->object_is_going_to_be_deleted_event(real_object);
	}
    }
#endif    
}

int IVP_Controller_VHash::controller_to_index(IVP_Controller *controller)
{
    return hash_index( (char *)&controller, sizeof(controller) );
}

IVP_BOOL IVP_Controller_VHash::compare(void *elem0, void *elem1) const
{
    if ( elem0 !=elem1 ) return(IVP_FALSE);   
    return(IVP_TRUE);
}


void IVP_Real_Object::do_radar_checking(IVP_Radar *radar){
    // loop through all synapses
    IVP_Radar_Hit hit;
    IVP_DOUBLE max_range = radar->max_range;
    {
	IVP_Synapse_Real *syn;
	// check real synapse
	for (syn = exact_synapses; syn; syn = syn->get_next()){
	    IVP_Mindist *md = syn->get_mindist();
	    int THIS = 0;
	    if ( md->get_synapse(1)->get_object() == this ) THIS = 1;
	
	    IVP_Synapse_Real *syn0 = md->get_synapse(THIS);
	    IVP_Synapse_Real *syn1 = md->get_synapse(1-THIS);
	    hit.this_object = syn0->get_object();
	    hit.other_object = syn1->get_object();

	    IVP_DOUBLE dist = md->get_length();
	    if (dist <= max_range){
		hit.dist = dist;
		radar->radar_hit(&hit);
	    }
	}
    }
    
    // check hulls
    IVP_Hull_Manager *hm = get_hull_manager();
    IVP_U_Min_List *ss = hm->get_sorted_synapses();

    max_range = radar->max_range + hm->get_current_hull_time();
    IVP_U_Min_List_Enumerator mindists(ss);
    IVP_Listener_Hull *supers;
    while ( (supers = (IVP_Listener_Hull*)mindists.get_next_element_lt(max_range))!= NULL ){
	if (supers->get_type() != IVP_HULL_ELEM_POLYGON) continue;
	IVP_Synapse_Real *syn = (IVP_Synapse_Real*)supers;
	IVP_Mindist *md = syn->get_mindist();
	md->recalc_mindist();	// @@@@ lots of cpu cycles wasted here !!!
	if (md->recalc_result != IVP_MDRR_OK) continue;
	
	int is_this = 0;
	if ( md->get_synapse(1)->get_object() == this ) is_this = 1;
	
	IVP_Synapse_Real *syn0 = md->get_synapse(is_this);
	IVP_Synapse_Real *syn1 = md->get_synapse(1-is_this);
	hit.this_object = syn0->get_object();
	hit.other_object = syn1->get_object();

	IVP_DOUBLE dist = md->get_length();
	if (dist <= max_range){
	    hit.dist = dist;
	    radar->radar_hit(&hit);
	}
    }
    // check spheres    
}

void IVP_Real_Object::convert_to_phantom(const IVP_Template_Phantom  *tmpl)
{
	P_DELETE(controller_phantom);
	controller_phantom = new IVP_Controller_Phantom(this,tmpl);
}

void IVP_Real_Object::beam_object_to_new_position( const IVP_U_Quat *rotation_world_f_object, const IVP_U_Point *position_w_f_o, IVP_BOOL optimize_for_repeated_calls)
{
	IVP_Core *core = this->get_core();
	IVP_U_Quat rotation_world_f_core(*rotation_world_f_object);
	IVP_U_Point position_w_f_c(*position_w_f_o);
	
	if (!flags.shift_core_f_object_is_zero)
	{
		IVP_U_Matrix m_world_f_object;
		rotation_world_f_object->set_matrix( &m_world_f_object );
		
		m_world_f_object.get_position()->set(position_w_f_o);
		IVP_U_Float_Point inv_shift_core_f_object; 
		inv_shift_core_f_object.set_negative(&shift_core_f_object);
		m_world_f_object.vmult4( &inv_shift_core_f_object , &position_w_f_c);
	}
	
	if ( this->q_core_f_object)
	{
		rotation_world_f_core.set_div_unit_quat( rotation_world_f_object, this->q_core_f_object );
	}

	IVP_Calc_Next_PSI_Solver nps(core);
	nps.set_transformation(&rotation_world_f_core, &position_w_f_c, optimize_for_repeated_calls);
}


void IVP_Real_Object::async_add_speed_object_ws( const IVP_U_Float_Point *speed_vec ) 
{
	this->ensure_in_simulation();
	this->physical_core->speed_change.add(speed_vec);
	//this->physical_core->environment->add_delayed_push_core(this->physical_core);
}

void IVP_Real_Object::async_add_rot_speed_object_cs( const IVP_U_Float_Point *rotation_vec ) 
{
	this->ensure_in_simulation();
	IVP_Core *core = this->get_core();
	core->rot_speed_change.add(rotation_vec);
	//core->environment->add_delayed_push_core(core);
}

void IVP_Real_Object::async_push_object_ws( const IVP_U_Point *position_ws_, const IVP_U_Float_Point *impulse_ws_)
{
    this->ensure_in_simulation();
    IVP_Core *core = this->get_core();
    IVP_U_Matrix m_world_f_core;
    core->calc_at_matrix( get_environment()->get_current_time(), & m_world_f_core);

    IVP_U_Float_Point impulse_cs;
    IVP_U_Float_Point position_cs;
    m_world_f_core.vimult4( position_ws_, &position_cs);
    m_world_f_core.vimult3( impulse_ws_, &impulse_cs);
    core->async_push_core(&position_cs, &impulse_cs, impulse_ws_);
}

void IVP_Real_Object::ensure_in_simulation(){
    // Note IVP_MT_STATIC != IVP_MT_NOT_SIM

    if (this->get_movement_state()!=IVP_MT_NOT_SIM){
	this->get_core()->reset_freeze_check_values();
    }else{
	//printf("add_reviving\n");
	IVP_ASSERT( !this->get_core()->physical_unmoveable );
	environment->add_revive_core(this->friction_core); 
    }

};
