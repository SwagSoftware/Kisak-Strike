// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#ifndef WIN32
#	pragma implementation "ivp_physic_private.hxx"
#	pragma implementation "ivp_listener_object.hxx"
#	pragma implementation "ivp_listener_collision.hxx"
#	pragma implementation "ivp_controller.hxx"
#endif

#include <ivp_listener_object.hxx>
#include <ivp_listener_collision.hxx>
#include <ivp_physic_private.hxx>
#include <ivp_i_object_vhash.hxx>
#include <ivp_i_collision_vhash.hxx>
#include <ivp_mindist_intern.hxx> //because of Mindist
#include <ivp_friction.hxx>
#include <ivu_active_value.hxx>
#include <ivp_actuator.hxx>

#include <ivp_debug_manager.hxx> //because of debug psi_synchrone
#include <ivp_merge_core.hxx>
#include <ivp_debug.hxx>
#include <ivp_universe_manager.hxx>
#include <ivp_authenticity.hxx>


void IVP_Listener_Collision::event_pre_collision( IVP_Event_Collision *){;};   // the user app sould override this
void IVP_Listener_Collision::event_post_collision( IVP_Event_Collision *){;};   // the user app sould override this
void IVP_Listener_Collision::event_collision_object_deleted( IVP_Real_Object *){;};  // only object private callbacks are called	

	// friction
    // set the IVP_LISTENER_COLLISION_CALLBACK_FRICTION bit in the constructor if you want to use this
void IVP_Listener_Collision::event_friction_created(IVP_Event_Friction *){;};   // the user app sould override this
void IVP_Listener_Collision::event_friction_deleted(IVP_Event_Friction *){;};   // the user app sould override this

IVP_Cluster *IVP_Cluster_Manager::get_root_cluster()
{
    return root_cluster;
}

IVP_Cluster_Manager::IVP_Cluster_Manager(IVP_Environment *env)
{
	P_MEM_CLEAR(this);
	environment = env;
    root_cluster = new IVP_Cluster(env);
    this->obj_callback_hash = new IVP_Object_Callback_Table_Hash(16);
    this->collision_callback_hash = new IVP_Collision_Callback_Table_Hash(16);
}

void IVP_Cluster_Manager::fire_event_object_deleted(IVP_Event_Object *event_obj)
{
    IVP_Object_Callback_Table *obj_table;
    IVP_Real_Object *ro = event_obj->real_object;
    obj_table = this->obj_callback_hash->find_table(ro);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Object *lis = obj_table->listeners.element_at(i);
	    lis->event_object_deleted(event_obj);
	    if (i>0 && !obj_callback_hash->find_table(ro)) break;	// object deleted
	}
    }
}

void IVP_Cluster_Manager::fire_event_object_frozen(IVP_Event_Object *event_obj)
{
    IVP_Object_Callback_Table *obj_table;
    IVP_Real_Object *ro = event_obj->real_object;
    obj_table = this->obj_callback_hash->find_table(ro);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Object *lis = obj_table->listeners.element_at(i);
	    lis->event_object_frozen(event_obj);
	    if (i>0 && !obj_callback_hash->find_table(ro)) break;	// object deleted
	}
    }
}

void IVP_Cluster_Manager::fire_event_object_created(IVP_Event_Object *event_obj)
{
    IVP_Object_Callback_Table *obj_table;
    IVP_Real_Object *ro = event_obj->real_object;
    obj_table = this->obj_callback_hash->find_table(ro);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Object *lis = obj_table->listeners.element_at(i);
	    lis->event_object_created(event_obj);
	    if (i>0 && !obj_callback_hash->find_table(ro)) break;	// object deleted
	}
    }
}

void IVP_Cluster_Manager::fire_event_object_revived(IVP_Event_Object *event_obj)
{
    IVP_Object_Callback_Table *obj_table;
    IVP_Real_Object *ro = event_obj->real_object;
    obj_table = this->obj_callback_hash->find_table(ro);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Object *lis = obj_table->listeners.element_at(i);
	    lis->event_object_revived(event_obj);
	    if (i>0 && !obj_callback_hash->find_table(ro)) break;	// object deleted
	}
    }
}

void IVP_Cluster_Manager::fire_event_pre_collision(IVP_Real_Object *real_object, IVP_Event_Collision *event_obj)
{
    IVP_Collision_Callback_Table *obj_table;
    obj_table = this->collision_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Collision *lis = obj_table->listeners.element_at(i);
	    if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_PRE_COLLISION)) continue;

	    lis->event_pre_collision(event_obj);
	    if (i>0 && !collision_callback_hash->find_table(real_object)) break;	// object deleted
	}
    }
}

void IVP_Cluster_Manager::fire_event_post_collision(IVP_Real_Object *real_object, IVP_Event_Collision *event_obj)
{
    IVP_Collision_Callback_Table *obj_table;
    obj_table = this->collision_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Collision *lis = obj_table->listeners.element_at(i);
	    if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_POST_COLLISION)) continue;

	    lis->event_post_collision(event_obj);
	    if (i>0 && !collision_callback_hash->find_table(real_object)) break;	// object deleted
	}
    }
}

void IVP_Cluster_Manager::fire_event_collision_object_deleted(IVP_Real_Object *real_object)
{
    IVP_Collision_Callback_Table *obj_table;
    obj_table = this->collision_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Collision *lis = obj_table->listeners.element_at(i);
	    if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_OBJECT_DELETED)) continue;

	    lis->event_collision_object_deleted(real_object);
	    if (i>0 && !collision_callback_hash->find_table(real_object)) break;	// object deleted
	}
    }
}

void IVP_Cluster_Manager::fire_event_friction_created(IVP_Real_Object *real_object, IVP_Event_Friction *event_friction)
{
    IVP_Collision_Callback_Table *obj_table;
    obj_table = this->collision_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Collision *lis = obj_table->listeners.element_at(i);
	    if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_FRICTION)) continue;
	    lis->event_friction_created(event_friction);
	    if (i>0 && !collision_callback_hash->find_table(real_object)) break;	// object deleted
	}
    }
}

void IVP_Cluster_Manager::fire_event_friction_deleted(IVP_Real_Object *real_object, IVP_Event_Friction *event_friction)
{
    IVP_Collision_Callback_Table *obj_table;
    obj_table = this->collision_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	int i;
	for (i=obj_table->listeners.len()-1; i>=0; i--) {
	    IVP_Listener_Collision *lis = obj_table->listeners.element_at(i);
	    if (!(lis->get_enabled_callbacks() & IVP_LISTENER_COLLISION_CALLBACK_FRICTION)) continue;
	    lis->event_friction_deleted(event_friction);
	    if (i>0 && !collision_callback_hash->find_table(real_object)) break;	// object deleted
	}
    }
}





IVP_Real_Object *IVP_Cluster_Manager::get_next_real_object_in_cluster_tree(IVP_Object *object){

    IVP_Object *next_object;
    if (number_of_real_objects == 0) return NULL;
    if (object == NULL){
    	IVP_Cluster *root = get_root_cluster();
	object = root->get_first_object_of_cluster();
	next_object = object; // no object supplied: we get the root cluster and we want the first object in root cluster!
	if (!object) return NULL;
    }
    else {
	next_object = object->next_in_cluster; // object supplied: just get the next object in cluster
    }


    IVP_OBJECT_TYPE type = object->get_type();    
    // try to go down as deep as possible
    if(type == IVP_CLUSTER){
	IVP_Cluster *cluster = object->to_cluster();
	while(1){
	    object = cluster->get_first_object_of_cluster();
	    if (object){
		type = object->get_type();
		if (type != IVP_CLUSTER) return object->to_real();
		cluster = object->to_cluster();
		continue;
	    }else{
		break; // just get next_in_cluster
	    }
	}
    }

    IVP_Object *next;
    // go up until a next pointer is found

    for (next=next_object; !next; next=object->next_in_cluster) {
	IVP_Object *old_father = object;
	object = object->father_cluster;
	if (!object){
    	    IVP_Cluster *root = get_root_cluster();
	    if ( root == old_father ) return(NULL); /* necessary to avoid endless loop caused by the last object
						    *	(ball's father is cluster; cluster has no father,
						    *   therefore we would "get_root_cluster()" which is the same as
						    *   the old cluster and whose first object is the "static ball" 
						    *    --> perfect endless loop :-) [SF, 25 nov 1999]
						    */
	    object = root->get_first_object_of_cluster();
	    if (!object) return NULL;
	}
    }

    // now a next is found
    object = next;
    type = object->get_type();

    if (type == IVP_CLUSTER){
	return get_next_real_object_in_cluster_tree(object);
    }

    return object->to_real();
}

void IVP_Cluster_Manager::check_for_unused_objects(IVP_Universe_Manager *um){
    const IVP_Universe_Manager_Settings *ums = um->provide_universe_settings();

    if (number_of_real_objects < ums->num_objects_in_environment_threshold_0) return;
    {
	int i = int( ums->check_objects_per_second_threshold_0 * environment->get_delta_PSI_time());
	for ( ; i>=0; i-- ){
	    IVP_Real_Object *an_object = this->an_object_to_be_checked;
	    this->an_object_to_be_checked = get_next_real_object_in_cluster_tree(this->an_object_to_be_checked);
	    if (!an_object) return; // no objects available
	    if (an_object->get_collision_check_reference_count() != 0) continue;
	    um->object_no_longer_needed(an_object);
	    if (number_of_real_objects < ums->num_objects_in_environment_threshold_0)	    return;
	}
    }

    if (number_of_real_objects < ums->num_objects_in_environment_threshold_1) return;
    {
	int i = int( ums->check_objects_per_second_threshold_1 * environment->get_delta_PSI_time());
	for ( ; i>=0; i-- ){
	    IVP_Real_Object *an_object = this->an_object_to_be_checked;
	    if (!an_object) return; // no objects available
	    this->an_object_to_be_checked = get_next_real_object_in_cluster_tree(this->an_object_to_be_checked);
	    if (an_object->get_collision_check_reference_count() != 1) continue;
	    um->object_no_longer_needed(an_object);
	    if (number_of_real_objects < ums->num_objects_in_environment_threshold_1)	    return;
	}
    }

    return;
}

IVP_Universe_Manager_Settings::IVP_Universe_Manager_Settings(){
	num_objects_in_environment_threshold_0 = 1;
	check_objects_per_second_threshold_0 = 1;

	num_objects_in_environment_threshold_1 = 1000000;
	check_objects_per_second_threshold_1 = 10;
};

IVP_Object_Callback_Table::~IVP_Object_Callback_Table()
{
    return;
}

IVP_Collision_Callback_Table::~IVP_Collision_Callback_Table()
{
    return;
}


IVP_Cluster_Manager::~IVP_Cluster_Manager() 
{ 
    P_DELETE( root_cluster );
    P_DELETE(this->obj_callback_hash);
    P_DELETE(this->collision_callback_hash);
}

void IVP_Cluster_Manager::add_listener_object(IVP_Real_Object *real_object, IVP_Listener_Object *listener)
{
    IVP_Object_Callback_Table *obj_table;
    obj_table = this->obj_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	obj_table->listeners.add(listener);
    }
    else {
	obj_table = new IVP_Object_Callback_Table();
	obj_table->real_object = real_object;
	obj_table->listeners.add(listener);
	this->obj_callback_hash->add_table(obj_table);        
	real_object->flags.object_listener_exists = 1;
    }
    return;
}

void IVP_Cluster_Manager::add_listener_collision(IVP_Real_Object *real_object, IVP_Listener_Collision *listener)
{
    IVP_Collision_Callback_Table *obj_table;
    obj_table = this->collision_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	obj_table->listeners.add(listener);
    }
    else {
	obj_table = new IVP_Collision_Callback_Table();
	obj_table->real_object = real_object;
	obj_table->listeners.add(listener);
	this->collision_callback_hash->add_table(obj_table);        
	real_object->flags.collision_listener_exists = 1;
    }
    return;
}

void IVP_Cluster_Manager::remove_listener_object(IVP_Real_Object *real_object, IVP_Listener_Object *listener)
{
    IVP_Object_Callback_Table *obj_table;
    obj_table = this->obj_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	obj_table->listeners.remove(listener);
	if ( obj_table->listeners.len() == 0 ) {
	    this->obj_callback_hash->remove_table(real_object);
	    delete obj_table;
	    real_object->flags.object_listener_exists = 0;
	}
    }
    return;
}

void IVP_Cluster_Manager::remove_listener_collision(IVP_Real_Object *real_object, IVP_Listener_Collision *listener)
{
    IVP_Collision_Callback_Table *obj_table;
    obj_table = this->collision_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	obj_table->listeners.remove(listener);
	if ( obj_table->listeners.len() == 0 ) {
	    this->collision_callback_hash->remove_table(real_object);
	    delete obj_table;
	    real_object->flags.collision_listener_exists = 0;
	}
    }
    return;
}

void IVP_Cluster_Manager::add_object(IVP_Real_Object * /*real_object*/ ){
    this->number_of_real_objects++;
}


void IVP_Cluster_Manager::remove_object(IVP_Real_Object *real_object)
{
    this->number_of_real_objects--;
    IVP_Object_Callback_Table *obj_table;
    obj_table = this->obj_callback_hash->find_table(real_object);
    if ( obj_table != NULL ) {
	this->obj_callback_hash->remove_table(real_object);
	delete obj_table;
    }

    IVP_Collision_Callback_Table *coll_table;
    coll_table = this->collision_callback_hash->find_table(real_object);
    if ( coll_table != NULL ) {
	this->collision_callback_hash->remove_table(real_object);
	delete coll_table;
    }

    if (real_object == an_object_to_be_checked){
        an_object_to_be_checked = get_next_real_object_in_cluster_tree(an_object_to_be_checked);
		if (real_object == an_object_to_be_checked){
			an_object_to_be_checked = NULL;
		}
    }

    IVP_Universe_Manager *um = environment->get_universe_manager();
    if (um){
	    um->event_object_deleted(real_object);
    }

    return;
}


// a not simulated object is revived and will be simulated again in next PSI
// returns IVP_TRUE when a friction system was grown -> needed for sim_unit revival algorithm
IVP_BOOL IVP_Core::revive_simulation_core()
{
    IVP_Core *core=this;
  
    IVP_ASSERT( !core->physical_unmoveable );
    IVP_ASSERT( core->movement_state == IVP_MT_NOT_SIM);

    for(int c = core->objects.len()-1;c>=0;c--) {
	IVP_Real_Object *r_obj=core->objects.element_at(c);
	IVP_IF(1) {
	    IVP_ASSERT( r_obj->get_movement_state() >= IVP_MT_NOT_SIM );
	}
	r_obj->set_movement_state(IVP_MT_MOVING);

    }
    
    core->init_core_for_simulation();
    IVP_Event_Sim es(environment, environment->get_next_PSI_time() - environment->get_current_time());
    core->calc_next_PSI_matrix_zero_speed(&es);

    // @@@ hack, go back to last PSI at wakeup at PSI ( satisfy init_PSI )
    
    IVP_IF (environment->get_env_state() == IVP_ES_PSI){
      IVP_DOUBLE d_time = environment->get_delta_PSI_time();
      core->time_of_last_psi += -d_time;
    }

    IVP_IF(IVP_DEBUG_OBJECT0){
	const char *search0 = IVP_DEBUG_OBJECT0;
	const char *name0 = core->objects.element_at(0)->get_name();
	if (	!P_String::string_cmp(name0, search0, IVP_FALSE)){
	    if ( core->environment->get_current_time().get_time() >= IVP_DEBUG_TIME){
		printf("revive object %s time:%f\n",      name0,    environment->get_current_time().get_time());
	    }
	}
    }
    IVP_IF(IVP_DEBUG_OBJECT1){
	const char *search0 = IVP_DEBUG_OBJECT1;
	const char *name0 = core->objects.element_at(0)->get_name();
	if (	!P_String::string_cmp(name0, search0, IVP_FALSE)){
	    if ( core->environment->get_current_time().get_time() >= IVP_DEBUG_TIME){
		printf("revive object %s time:%f\n",      name0,    environment->get_current_time().get_time());
	    }
	}
    }

    //IVP_Friction_Info_For_Core *my_info=core->moveable_core_has_friction_info();
    //if(my_info==NULL) {
    	// grow a friction system (search for near objects and put them into one friction system )
    IVP_BOOL fs_was_grown=core->grow_friction_system();
	//}

    for(int d = core->objects.len()-1;d>=0;d--) {
	IVP_Real_Object *r_obj=core->objects.element_at(d);
    	{ // fire object-dependant 'object revived' event
	    IVP_Event_Object event_revived;
	    event_revived.real_object = r_obj;
	    event_revived.environment = core->environment;
	    core->environment->get_cluster_manager()->fire_event_object_revived(&event_revived);
	}

	{ // fire global 'object revived' event
	    IVP_Event_Object event_revived;
	    event_revived.real_object = r_obj;
	    event_revived.environment = core->environment;
	    core->environment->fire_event_object_revived(&event_revived);
	}
    }
    
    return fs_was_grown;
}

void IVP_Core::fire_event_object_frozen(){
   //  fire events
    for (int i=this->objects.len()-1; i>=0; i--) {
	IVP_Real_Object *real_object = this->objects.element_at(i);

	{ // fire object-dependant 'object frozen' event
	    IVP_Event_Object event_frozen;
	    event_frozen.real_object = real_object;
	    event_frozen.environment = this->environment;
	    this->environment->get_cluster_manager()->fire_event_object_frozen(&event_frozen);
	}

	{ // fire global 'object frozen' event
	    IVP_Event_Object event_frozen;
	    event_frozen.real_object = real_object;
	    event_frozen.environment = this->environment;
	    this->environment->fire_event_object_frozen(&event_frozen);
	}
	
    }
}

void IVP_Core::freeze_simulation_core(){
    IVP_Core *r_core=this;
    r_core->stop_physical_movement(); //because of Interpolations
    
    /// deactivate all mindists
    IVP_IF(IVP_DEBUG_OBJECT0){
	const char *search0 = IVP_DEBUG_OBJECT0;
	const char *name0 = r_core->objects.element_at(0)->get_name();
	if (	!P_String::string_cmp(name0, search0, IVP_FALSE)){
	    if ( r_core->environment->get_current_time().get_time() >= IVP_DEBUG_TIME){
		printf("freeze object %s time:%f\n",      name0,    environment->get_current_time().get_time());
	    }
	}
    }
    IVP_IF(IVP_DEBUG_OBJECT1){
	const char *search0 = IVP_DEBUG_OBJECT1;
	const char *name0 = r_core->objects.element_at(0)->get_name();
	if (	!P_String::string_cmp(name0, search0, IVP_FALSE)){
	    if ( r_core->environment->get_current_time().get_time() >= IVP_DEBUG_TIME){
		printf("freeze object %s time:%f\n",      name0,    environment->get_current_time().get_time());
	    }
	}
    }
    
    fire_event_object_frozen();
 
}

void IVP_Core::debug_out_movement_vars() {
    printf("core_status %lx  trans %f %f %f  rot %f %f %f\n",(long)this&0x0000ffff,speed.k[0],speed.k[1],speed.k[2],rot_speed.k[0],rot_speed.k[1],rot_speed.k[2]); 
}

void IVP_Core::debug_vec_movement_state() {
    IVP_Core *my_core=this;
    IVP_Core *one_object=this;
    IVP_IF( 0 ){
	char *out_text;
	IVP_U_Float_Point ivp_pointer;
	IVP_U_Point ivp_start;
	int v_color;
	ivp_start.set(my_core->get_position_PSI());	
	ivp_pointer.set(0.0f,-7.0f,0.0f);
	out_text=p_make_string("oob%lx_sp%.3f",(long)one_object&0x0000ffff,one_object->speed.real_length());//,one_object->get_energy_on_test(&one_object->speed,&one_object->rot_speed));
	if(my_core->movement_state==IVP_MT_CALM)	{
	    //out_text=p_export_error("%lxo_calm%lx",(long)one_object&0x0000ffff,(long)fr_i&0x0000ffff);
	    //sprintf(out_text,"%ldobj_calm%lx",counter,(long)one_object);
	    v_color=3;
	} else {
	    if(my_core->movement_state==IVP_MT_SLOW)
	    {
		//out_text=p_export_error("%lxdo_not_m%lx",(long)one_object&0x0000ffff,(long)fr_i&0x0000ffff);
		v_color=6;
	    } else {
		//out_text=p_export_error("%lxdo_moved%lx",(long)one_object&0x0000ffff,(long)fr_i&0x0000ffff);
		//sprintf(out_text,"%obj_moved%lx",(long)one_object);
		v_color=1;
	    }
	}
	this->environment->add_draw_vector(&ivp_start,&ivp_pointer,out_text,v_color);
	P_FREE(out_text);
    }
}

// check for not moving objects
// object is removed from simulation if not moving for 1 sec or slowling moving for 10 sec
// exception: objects belonging to friction systems
// friction systems are tested separately, all objects of fr_system are removed at a time


IVP_Friction_System::IVP_Friction_System(IVP_Environment *env)
{
    IVP_IF(env->get_debug_manager()->check_fs) {
	fprintf(env->get_debug_manager()->out_deb_file,"create_fs %f %lx\n",env->get_current_time().get_time(),(long)this);
    }
    //printf("creating_new_fs %lx at time %f\n",(long)this,env->get_current_time());
    l_environment=env;
    //first_friction_obj=NULL;
    union_find_necessary=IVP_FALSE;
    first_friction_dist=NULL;
    friction_obj_number=0;
    friction_dist_number=0;

    static_fs_handle.l_friction_system=this;
    energy_fs_handle.l_friction_system=this;
}

IVP_Friction_System::~IVP_Friction_System()
{
    //printf("deleting_of_fs %lx at time %f\n",(long)this,l_environment->get_current_time());
    IVP_IF(l_environment->get_debug_manager()->check_fs) {
	fprintf(l_environment->get_debug_manager()->out_deb_file,"delete_fs %f %lx\n",l_environment->get_current_time().get_time(),(long)this);
    }
    // deleteing of real friction systems filled with information is not yet implemented (not trivial : unlink whole friction infos)

    //Assertion: when
}

void IVP_Friction_System::fs_recalc_all_contact_points() {
    for (int i = fr_pairs_of_objs.len()-1; i>=0;i--){
	IVP_Friction_Core_Pair *my_pair = fr_pairs_of_objs.element_at(i);
	IVP_FLOAT energy_diff_sum=0.0f;

	for (int j = my_pair->fr_dists.len()-1; j>=0;j--){
	    IVP_Contact_Point *fr_mindist = my_pair->fr_dists.element_at(j);
	    IVP_FLOAT old_gap_len=fr_mindist->get_gap_length();
	    fr_mindist->recalc_friction_s_vals();
	    //energy considerations
	    //when gap is gettinger bigger, potential energy is gained
	    //when pressure is going up, do not mind
	    IVP_FLOAT gap_diff= old_gap_len - fr_mindist->get_gap_length();	    
	    energy_diff_sum+=fr_mindist->now_friction_pressure * gap_diff;
	}
	if(energy_diff_sum > 0.0f) {
	    my_pair->integrated_anti_energy += energy_diff_sum;
	}
    }    
}

// is union find necessary after removing of dist ?
IVP_BOOL IVP_Friction_System::dist_removed_update_pair_info(IVP_Contact_Point *old_dist)
{
    //manage info of obj pairs
    {
	IVP_Friction_Core_Pair *my_pair_info;
	IVP_Core *core0,*core1;
	core0=old_dist->get_synapse(0)->l_obj->physical_core;
	core1=old_dist->get_synapse(1)->l_obj->physical_core;
	my_pair_info=this->get_pair_info_for_objs(core0,core1);
	if(!my_pair_info)
	{
	    CORE;
	}

	my_pair_info->del_fr_dist_obj_pairs(old_dist);
	if(my_pair_info->number_of_pair_dists()==0)
	{
	    this->del_fr_pair(my_pair_info);
	    P_DELETE(my_pair_info);
	    // start union find
	    return IVP_TRUE;
	} else {
	    return IVP_FALSE;
	}
    }
}

void IVP_Friction_System::remove_dist_from_system(IVP_Contact_Point *old_dist)
{
    IVP_IF(l_environment->get_debug_manager()->check_fs) {
	fprintf(l_environment->get_debug_manager()->out_deb_file,"rem_dist_from_fs %f %lx from %lx\n",l_environment->get_current_time().get_time(),(long)old_dist,(long)this);
    }

    //manage list of all dists
    {
	IVP_Contact_Point *previous=old_dist->prev_dist_in_friction;
	IVP_Contact_Point *following=old_dist->next_dist_in_friction;
	if(following!=NULL)
	{
	    following->prev_dist_in_friction=previous;
	}
	if(previous)
	{
	    previous->next_dist_in_friction=following;
	} else {
	    this->first_friction_dist=following;
	}
    }
    friction_dist_number--;
}

void IVP_Friction_System::dist_added_update_pair_info(IVP_Contact_Point *new_dist)
{
    //manage info of obj pairs
    {
	IVP_Friction_Core_Pair *my_pair_info;
	IVP_Core *core0,*core1;
	core0=new_dist->get_synapse(0)->l_obj->physical_core;
	core1=new_dist->get_synapse(1)->l_obj->physical_core;
	my_pair_info=this->get_pair_info_for_objs(core0,core1);
	if(my_pair_info)
	{
	    ;
	} else {
	    my_pair_info=new IVP_Friction_Core_Pair();
	    my_pair_info->objs[0]=core0;
	    my_pair_info->objs[1]=core1;
	    this->add_fr_pair(my_pair_info);
	}
	my_pair_info->add_fr_dist_obj_pairs(new_dist);
    }
}

void IVP_Friction_System::add_dist_to_system(IVP_Contact_Point *new_dist)
{
    IVP_IF(l_environment->get_debug_manager()->check_fs) {
	fprintf(l_environment->get_debug_manager()->out_deb_file,"add_dist_to_fs %f %lx to %lx\n",l_environment->get_current_time().get_time(),(long)new_dist,(long)this);
    }

    new_dist->l_friction_system=this;
    //manage list of all dists
    {
	IVP_Contact_Point *first=this->first_friction_dist;
	new_dist->prev_dist_in_friction=NULL;
	new_dist->next_dist_in_friction=first;
	this->first_friction_dist=new_dist;
	if(first)
	{
	    first->prev_dist_in_friction=new_dist;
	}
    }

    //new_dist->number_in_friction = this->friction_dist_number; //dists are numbered the wrong way: last has number 0
    IVP_IF(l_environment->debug_information->debug_mindist){
	printf("added_dist %d\n",(int)friction_dist_number);
    }
    friction_dist_number++;
    
    //fr_solver.calc_calc_solver(this); //solver matrix must be greater and temporary results also
}

void IVP_Friction_System::add_core_to_system(IVP_Core *new_obj)
{
    IVP_IF(l_environment->get_debug_manager()->check_fs) {
	fprintf(l_environment->get_debug_manager()->out_deb_file,"add_core_to_fs %f %lx to %lx\n",l_environment->get_current_time().get_time(),(long)new_obj,(long)this);
    }

    cores_of_friction_system.add(new_obj);
    if(!new_obj->physical_unmoveable) {
        moveable_cores_of_friction_system.add(new_obj);
	new_obj->add_core_controller(&static_fs_handle);
        new_obj->add_core_controller(this);
	new_obj->add_core_controller(&energy_fs_handle);
	//new_obj->sim_unit_of_core->add_controller_of_core( new_obj, &this->static_fs_handle );
	//new_obj->sim_unit_of_core->add_controller_of_core( new_obj, this );
	//new_obj->sim_unit_of_core->add_controller_of_core( new_obj, &this->energy_fs_handle );
    }
    
    friction_obj_number++;
}
 
void IVP_Friction_System::remove_core_from_system(IVP_Core *old_obj)
{
    IVP_IF(l_environment->get_debug_manager()->check_fs) {
	fprintf(l_environment->get_debug_manager()->out_deb_file,"remove_core_from_fs %f %lx from %lx\n",l_environment->get_current_time().get_time(),(long)old_obj,(long)this);
    }

    if(!old_obj->physical_unmoveable) {
        moveable_cores_of_friction_system.remove(old_obj);
	old_obj->rem_core_controller(&this->energy_fs_handle);
        old_obj->rem_core_controller(this);
	old_obj->rem_core_controller(&static_fs_handle);
    }    
    cores_of_friction_system.remove(old_obj);

    friction_obj_number--;
}

void IVP_Environment::remove_revive_core(IVP_Core *c) {
    int index=this->core_revive_list.index_of(c);
    if(index>=0) {
	IVP_ASSERT(c->is_in_wakeup_vec==IVP_TRUE);
	this->core_revive_list.remove_at(index);
	c->is_in_wakeup_vec=IVP_FALSE;
    }
}

void IVP_Environment::add_revive_core(IVP_Core *c)
{
    if(c->is_in_wakeup_vec==IVP_TRUE) {
	return;
    }
    IVP_IF(1) {
	IVP_ASSERT(this->core_revive_list.index_of(c)<0);
    }
	
    core_revive_list.add(c);
    c->is_in_wakeup_vec=IVP_TRUE;
}

void IVP_Environment::revive_cores_PSI(){
    for(int i=core_revive_list.len()-1;i>=0;i--) {
	IVP_Core *my_core=core_revive_list.element_at(i);
	IVP_ASSERT(my_core->physical_unmoveable==IVP_FALSE);
	my_core->ensure_core_to_be_in_simulation();
	my_core->is_in_wakeup_vec=IVP_FALSE;
    }
    core_revive_list.clear();
}





void IVP_Universe_Manager::ensure_objects_in_environment(IVP_Real_Object * /*object*/,
							 IVP_U_Float_Point * /*sphere_center*/,
							 IVP_DOUBLE /*sphere_radius*/) {
    ;
}

void IVP_Universe_Manager::object_no_longer_needed(IVP_Real_Object *) {
    ;
}

void IVP_Universe_Manager::event_object_deleted(IVP_Real_Object *) {
    ;
}

const IVP_Universe_Manager_Settings *IVP_Universe_Manager::provide_universe_settings() {
    return NULL;
}


