// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_hull_manager.hxx>
#include <ivp_clustering_longrange.hxx>
#include <ivp_range_manager.hxx>

// assumes that core is freshly updated by calc_next_PSI_matrix()
void IVP_Synapse_OO::hull_limit_exceeded_event(IVP_Hull_Manager *,IVP_HTIME){
    watcher->hull_limit_exceeded_event();
}

void IVP_OO_Watcher::hull_limit_exceeded_event(){

    IVP_Real_Object *obj0 = get_synapse(0)->object;
    IVP_Real_Object *obj1 = get_synapse(1)->object;
    
    // build all mindists
    IVP_DOUBLE dist0,dist1;
    IVP_Environment *env = obj0->get_environment();
    env->get_statistic_manager()->range_intra_exceeded++;
    env->get_range_manager()->get_coll_range_intra_objects(obj0,obj1,&dist0,&dist1);

    env->get_mindist_manager()->create_exact_mindists(obj0,obj1,  dist0 + dist1, &mindists, NULL, NULL, NULL, NULL, this);
    IVP_Time t_now = env->get_current_time();
    obj0->get_hull_manager()->update_synapse( get_synapse(0), t_now, dist0);
    obj1->get_hull_manager()->update_synapse( get_synapse(1), t_now, dist1);    
}

void IVP_Synapse_OO::hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *){
    watcher->hull_manager_is_going_to_be_deleted_event();
}

IVP_Synapse_OO::~IVP_Synapse_OO(){
  object->get_hull_manager()->remove_synapse(this);
}

void IVP_Synapse_OO::init_synapse_oo(IVP_OO_Watcher *w, IVP_Real_Object *obj){
    IVP_Hull_Manager *hull_manager = obj->get_hull_manager();
    object = obj;
    hull_manager->insert_synapse(this,obj->get_environment()->get_current_time(), P_FLOAT_MAX);	// dummy time
    watcher = w;
}


void IVP_OO_Watcher::hull_manager_is_going_to_be_deleted_event(){
    delete this;
}

IVP_OO_Watcher::IVP_OO_Watcher(IVP_Collision_Delegator *del, IVP_Real_Object *obj0, IVP_Real_Object *obj1): IVP_Collision(del), mindists(8){
    get_synapse(0)->init_synapse_oo(this,obj0);
    get_synapse(1)->init_synapse_oo(this,obj1);    
    hull_limit_exceeded_event(  );
}

IVP_OO_Watcher::~IVP_OO_Watcher(){
    for (int i = mindists.len()-1; i>=0;i--){
	IVP_Collision *ma  = mindists.element_at(i);
	P_DELETE(ma);
    }
    delegator->collision_is_going_to_be_deleted_event(this);
}


void IVP_OO_Watcher::collision_is_going_to_be_deleted_event(IVP_Collision *c){
    mindists.remove_allow_resort(c);
}

void IVP_OO_Watcher::get_objects( IVP_Real_Object *objects_out[2] ){
    objects_out[0] = get_synapse(0)->object;
    objects_out[1] = get_synapse(1)->object;
}

void IVP_OO_Watcher::get_ledges( const IVP_Compact_Ledge *ledges_out[2] ){
    ledges_out[0] = NULL;
    ledges_out[1] = NULL;
}



