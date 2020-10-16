// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_range_manager.hxx>


IVP_Range_Manager::IVP_Range_Manager(IVP_Environment *env, IVP_BOOL delete_this_on_env_delete){
    bound_to_environment = delete_this_on_env_delete;
    environment = env;
    look_ahead_time_world = 1.0f; 		// seconds
    look_ahead_max_radius_world = 5.0f;   // factor of object radius
    look_ahead_min_distance_world = 0.5f;	// [m]
    look_ahead_max_distance_world = 15.0f;	// [m]
    look_ahead_min_seconds_world = 0.1f; // [s] this is real min assert(look_ahead_min_seconds > 2*environment->get_delta_PSI_time())

    look_ahead_time_intra = 0.5f; 		 // seconds
    look_ahead_max_radius_intra = 0.9f;	 // additional radius as factor of object radius
    look_ahead_min_distance_intra = 0.8f;// [m]
    look_ahead_max_distance_intra = 10.0f;	 // [m]
    look_ahead_min_seconds_intra = 0.1f; // [s] this is real min assert(look_ahead_min_seconds > 2*environment->get_delta_PSI_time())
}

void IVP_Range_Manager::environment_will_be_deleted(IVP_Environment *){
    if (bound_to_environment){
	P_DELETE_THIS(this);
    }
}

void IVP_Range_Manager::get_coll_range_intra_objects( const IVP_Real_Object *obj0,const IVP_Real_Object *obj1,
						      IVP_DOUBLE *range0, IVP_DOUBLE *range1 ){
    
    IVP_Core *core0 = obj0->get_core();
    IVP_Core *core1 = obj1->get_core();

    // split max_dist to both hull_managers
    IVP_DOUBLE speed0 = core0->current_speed + core0->max_surface_rot_speed + P_DOUBLE_EPS; // 
    IVP_DOUBLE speed1 = core1->current_speed + core1->max_surface_rot_speed + P_DOUBLE_EPS;

    IVP_DOUBLE sum_speed = speed0 + speed1;

    IVP_DOUBLE min_rad;
    if ( core0->upper_limit_radius < core1->upper_limit_radius){
	min_rad = core0->upper_limit_radius;
    }else{
	min_rad = core1->upper_limit_radius;
    }
    IVP_DOUBLE max_dist = sum_speed * look_ahead_time_intra;

    // take minimul of range and radius
    if ( min_rad * look_ahead_max_radius_intra < max_dist){
	max_dist = min_rad * look_ahead_max_radius_intra;
    }
    if (max_dist < look_ahead_min_distance_intra){
	max_dist = look_ahead_min_distance_intra;
    }

    if (max_dist > look_ahead_max_distance_intra){
	max_dist = look_ahead_max_distance_intra;
    }

    max_dist -= sum_speed * environment->get_delta_PSI_time();

    if ( max_dist < look_ahead_min_seconds_intra * sum_speed){
	max_dist  = look_ahead_min_seconds_intra * sum_speed;
    }
    
    
    speed0 += 0.2f * speed1;
    speed1 += 0.18f * speed0;
    
    IVP_DOUBLE sum = speed0 + speed1;
    IVP_DOUBLE inv_sum = 1.0f / sum;

    *range0 = max_dist * speed0 * inv_sum;
    *range1 = max_dist * speed1 * inv_sum;
}


IVP_DOUBLE IVP_Range_Manager::get_coll_range_in_world( const IVP_Real_Object *obj0 ){
    
    IVP_Core *core0 = obj0->get_core();

    // split max_dist to both hull_managers
    IVP_DOUBLE speed0 = core0->current_speed + core0->max_surface_rot_speed + P_DOUBLE_EPS; // 

    IVP_DOUBLE min_rad = core0->upper_limit_radius;

    IVP_DOUBLE max_dist = speed0 * look_ahead_time_world;

    // take minimul of range and radius
    if ( min_rad * look_ahead_max_radius_world < max_dist){
	max_dist = min_rad * look_ahead_max_radius_world;
    }
    if (max_dist < look_ahead_min_distance_world){
	max_dist = look_ahead_min_distance_world;
    }

    if (max_dist > look_ahead_max_distance_world){
	max_dist = look_ahead_max_distance_world;
    }

    max_dist -= speed0 * environment->get_delta_PSI_time();

    if ( max_dist < core0->upper_limit_radius + look_ahead_min_seconds_world * speed0){
	max_dist  = look_ahead_min_seconds_world * speed0 + core0->upper_limit_radius;
    }
        
    return max_dist;
}

