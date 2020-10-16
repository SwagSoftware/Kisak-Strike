// Copyright (C) Ipion Software GmbH 2000. All rights reserved.

// IVP_EXPORT_PUBLIC


#ifndef WIN32
#	pragma implementation "ivp_anomaly_manager.hxx"
#endif

#include <ivp_physics.hxx>

#include <ivp_anomaly_manager.hxx>

#include <ivp_mindist.hxx>
#include <ivp_mindist_intern.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_compact_ledge.hxx>

IVP_Anomaly_Limits::IVP_Anomaly_Limits(IVP_BOOL delete_this_if_env_is_deleted_in){
    delete_this_if_env_is_deleted = delete_this_if_env_is_deleted_in;
    max_velocity = 2000.0f;
    max_angular_velocity_per_psi = IVP_FLOAT(IVP_PI * 0.5f);
    max_collisions_per_psi = 70000;
}


IVP_Anomaly_Limits::~IVP_Anomaly_Limits(){
}

void IVP_Anomaly_Limits::environment_will_be_deleted(IVP_Environment *){
    if (delete_this_if_env_is_deleted){
	P_DELETE_THIS(this);
    }
}



IVP_Anomaly_Manager::IVP_Anomaly_Manager(IVP_BOOL delete_this_if_env_is_deleted_in){
    delete_this_if_env_is_deleted = delete_this_if_env_is_deleted_in;
}

IVP_Anomaly_Manager::~IVP_Anomaly_Manager(){
}

void IVP_Anomaly_Manager::environment_will_be_deleted(IVP_Environment *){
    if (delete_this_if_env_is_deleted){
	P_DELETE_THIS(this);
    }
}

void IVP_Anomaly_Manager::max_velocity_exceeded(IVP_Anomaly_Limits *al, IVP_Core *, IVP_U_Float_Point *velocity_in_out){
    IVP_DOUBLE square_speed = velocity_in_out->quad_length();
    IVP_DOUBLE max_speed = al->get_max_velocity();
    velocity_in_out->mult(0.99f * max_speed / IVP_Inline_Math::ivp_sqrtf(square_speed));
}

void IVP_Anomaly_Manager::max_angular_velocity_exceeded(IVP_Anomaly_Limits *al, IVP_Core *core, IVP_U_Float_Point *angular_velocity_in_out){
    IVP_DOUBLE square_rot_speed = angular_velocity_in_out->quad_length();
    IVP_DOUBLE max_rot_speed = al->get_max_angular_velocity_per_psi() * core->environment->get_inv_delta_PSI_time();
    angular_velocity_in_out->mult(0.9f * max_rot_speed / IVP_Inline_Math::ivp_sqrtf(square_rot_speed));
}

void IVP_Anomaly_Manager::solve_inter_penetration_simple( IVP_Real_Object *obj0, IVP_Real_Object *obj1){
        IVP_Environment *env = obj0->get_environment();
    IVP_DOUBLE d_time = env->get_delta_PSI_time();
    IVP_Core *core0 = obj0->get_core();
    IVP_Core *core1 = obj1->get_core();

    const IVP_U_Matrix *m_world_f_core0 = core0->get_m_world_f_core_PSI();
    const IVP_U_Matrix *m_world_f_core1 = core1->get_m_world_f_core_PSI();

    IVP_U_Float_Point vec01;
    vec01.subtract( &m_world_f_core1->vv, &m_world_f_core0->vv);
    float deltaLength = vec01.real_length_plus_normize();
	if ( deltaLength <= 0.01 )
	{
		// if the objects are exactly on top of each other just push down X the axis
		vec01.set( 1, 0, 0 );
	}

    IVP_DOUBLE speed_change = get_push_speed_penetration( obj0, obj1);

    if(IVP_MTIS_SIMULATED(core0->movement_state) && !core0->pinned){
	IVP_U_Float_Point p0;
	p0.set_multiple(&vec01, -speed_change * core0->get_mass());
	obj0->async_push_object_ws(&m_world_f_core0->vv, &p0);
	IVP_U_Float_Point rot_speed_object(-d_time,0,0);
	obj0->async_add_rot_speed_object_cs( &rot_speed_object);
    }
		
    if(IVP_MTIS_SIMULATED(core1->movement_state) && !core1->pinned){
	IVP_U_Float_Point p1;
	p1.set_multiple(&vec01, speed_change * core1->get_mass());
	obj1->async_push_object_ws(&m_world_f_core0->vv, &p1);
	IVP_U_Float_Point rot_speed_object(d_time,0,0);
	obj1->async_add_rot_speed_object_cs( &rot_speed_object);
    }

    IVP_IF(1){
	const char *name0 = obj0->get_name();
	if (!name0) name0 = "(null)";
	const char *name1 = obj1->get_name();
	if (!name1) name1 = "(null)";
	printf("Mindist rescue push between '%s' and '%s' at %f\n",
	       name0,name1,
	       env->get_current_time().get_time());
    }
}

void IVP_Anomaly_Manager::inter_penetration(IVP_Mindist *mindist, IVP_Real_Object *obj0, IVP_Real_Object *obj1){
	IVP_Real_Object *my_objects[2];
	my_objects[0]=obj0;
	my_objects[1]=obj1;
	int swapped=0;

	if( obj1->get_core()->physical_unmoveable ) {
		swapped=1;
	}

	if( !my_objects[swapped]->get_core()->physical_unmoveable ) {
		solve_inter_penetration_simple( obj0,obj1 );
		return;
	}
 
	//solve_inter_penetration_simple( obj0, obj1 );
	//return;

#if 1
	if( mindist->synapse[swapped].get_status() > IVP_ST_TRIANGLE ) {
	    solve_inter_penetration_simple( obj0, obj1 );
		return;
	}

	// now my_objects[swapped] is unmoveable and has a polygonal ledge
	IVP_Real_Object *moving_obj=my_objects[1-swapped];
	IVP_U_Point world_pos_moving;
	const IVP_U_Matrix *world_mat;
	world_mat=moving_obj->get_core()->get_m_world_f_core_PSI();
	world_pos_moving.set( &world_mat->vv );
	
	IVP_Cache_Object *unmovable_cache=my_objects[swapped]->get_cache_object_no_lock();
	IVP_U_Point obj_pos_moving;
	unmovable_cache->m_world_f_object.vimult4( &world_pos_moving, &obj_pos_moving );

	const IVP_Compact_Ledge *my_compact_ledge=mindist->synapse[swapped].get_ledge();
	const IVP_Compact_Triangle *my_compact_triangle=my_compact_ledge->get_first_triangle();

    IVP_U_Point push_direction;   //direction to push movable object
	push_direction.set(1.0f,0.0f,0.0f); 
	IVP_DOUBLE biggest_dist= -P_DOUBLE_MAX;

	for(int i=0;i<my_compact_ledge->get_n_triangles();i++) {
		IVP_U_Point three_points[3];
		three_points[0].set( my_compact_triangle->get_edge(0)->get_start_point( my_compact_ledge ) );
		three_points[1].set( my_compact_triangle->get_edge(1)->get_start_point( my_compact_ledge ) );
		three_points[2].set( my_compact_triangle->get_edge(2)->get_start_point( my_compact_ledge ) );

		IVP_U_Hesse hesse_plane;
		hesse_plane.calc_hesse( &three_points[0], &three_points[2], &three_points[1] );
		hesse_plane.normize();

		IVP_DOUBLE distance=hesse_plane.get_dist(&obj_pos_moving); //negative distance means surface is looking away from me (backside)
		if(distance>biggest_dist) {
			biggest_dist=distance;
			push_direction.set( &hesse_plane );
		}

		my_compact_triangle=my_compact_triangle->get_next_tri();
	}

	IVP_U_Point world_push_dir;
	unmovable_cache->m_world_f_object.vmult3( &push_direction, &world_push_dir );

    IVP_U_Float_Point world_push_vec;
	world_push_vec.set(&world_push_dir);

    IVP_DOUBLE speed_change = get_push_speed_penetration( obj0, obj1);

	IVP_Core *moving_core=moving_obj->get_core();
    if(IVP_MTIS_SIMULATED(moving_core->movement_state)){
		world_push_vec.mult( speed_change );
		moving_obj->async_add_speed_object_ws( &world_push_vec );
    }

#endif
}

IVP_FLOAT IVP_Anomaly_Manager::get_push_speed_penetration( IVP_Real_Object *obj0, IVP_Real_Object *obj1) {
	IVP_DOUBLE d_time=obj0->get_environment()->get_delta_PSI_time();
    IVP_DOUBLE grav = obj0->get_environment()->get_gravity()->real_length();
    IVP_DOUBLE speed_change = (grav*2.0f) * d_time;
	return speed_change;
}

IVP_BOOL IVP_Anomaly_Manager::max_collisions_exceeded_check_freezing(IVP_Anomaly_Limits *, IVP_Core *core){
    return IVP_TRUE;
}
