// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivp_contact_situation.hxx"
#endif

#include <ivp_physics.hxx>
#include <ivu_memory.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>

#include <ivp_physic_private.hxx>
#include <ivp_sim_unit.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_material.hxx>
#include <ivp_impact.hxx> 
#include <ivp_mindist_intern.hxx>
//#include <ivp_friction.hxx>
#include <ivp_hull_manager.hxx>
#include "ivp_mindist_macros.hxx"
#include <ivp_listener_collision.hxx>
#include <ivp_calc_next_psi_solver.hxx>
#include <ivp_debug_manager.hxx>
#include <ivp_anomaly_manager.hxx>

#define IVP_SAFETY_FACTOR_FOR_DELAY 1.2f
#define IVP_INV_SAFETY_FACTOR_FOR_DELAY 1.0f/IVP_SAFETY_FACTOR_FOR_DELAY
#define IVP_ROT_MAX_UNCERTAINTY 0.5f // max rot degree speed uncertainty for rescue_speed
#define IVP_COLL_DETECT_MIN_INV_TIME 200.0f // inverse time we have to make sure no collision occurs
#define IVP_COLL_DETECT_MIN_TIME 1.0f/IVP_COLL_DETECT_MIN_INV_TIME
#define IVP_SPEED_ADDON_SYSTEM_IMPACT 0.01f
#define IVP_INV_HALF_CONSERVATION_STEPS 0.5f // elasticity is raised with number of impacts in system, at 1.0f/IVP_INV_HALF_CONSERVATION_STEPS the elasticity is at least 0.5
#define MAXIMUM_SYSTEM_PUSH_NUMBER 5000
#define MINIMAL_IMPACT_VELOCITY_FIXPOINT -11.552f
#define IMPACT_EPS 10E-5f

//#define EASEONIMPACT //doesnt work with balls // #+# kill and remove dependent functions
#define SYSTEM_IMPACT_ENABLED 

// generates friction mindist from mindist when no friction mindist exists and makes sure that friction mindist (generated or not) is up to date
// sim_unit_not_destroy is the one that has to remain after fusion

IVP_Contact_Point *IVP_Mindist::try_to_generate_managed_friction(IVP_Friction_System **associated_fs,IVP_BOOL *having_new,IVP_Simulation_Unit *sim_unit_not_destroy,IVP_BOOL call_recalc_svals){
    
    IVP_Mindist *my_dist = this;
    IVP_Real_Object *obj0 = get_sorted_synapse(0)->l_obj;
    IVP_Real_Object *obj1 = get_sorted_synapse(1)->l_obj;


    IVP_IF(obj0->get_environment()->debug_information->debug_friction){
	printf("new_fri_mindist\n");
    }

    IVP_Friction_System *fr_sys0,*fr_sys1;
    IVP_Friction_Info_For_Core *fr_info0,*fr_info1;

    IVP_Core *core0 = obj0->friction_core;
    IVP_Core *core1 = obj1->friction_core;
    
    if(core0->physical_unmoveable){
	IVP_Core *temp_core;
	temp_core=core0;
	core0=core1;
	core1=temp_core;
    }
    
    //core0 is now movable
    
    fr_info0=core0->moveable_core_has_friction_info();
    
    IVP_Environment *my_env = core0->environment;
    IVP_Contact_Point *friction_dist;
    IVP_BOOL gen_success;
    
    friction_dist = IVP_Friction_Manager::generate_contact_point(my_dist,&gen_success);
    {
	if(gen_success!=IVP_TRUE) {
	    *associated_fs=fr_info0->l_friction_system;
	    *having_new=IVP_FALSE;
	    if(call_recalc_svals) {
		friction_dist->recalc_friction_s_vals();
		friction_dist->read_materials_for_contact_situation(friction_dist->tmp_contact_info);		
	    }
	    return friction_dist;
	}
    }
    if(call_recalc_svals) {
	friction_dist->recalc_friction_s_vals(); // this is overhead, because nearly the same function was called for the associated mindist
	friction_dist->read_materials_for_contact_situation(friction_dist->tmp_contact_info);
    }

    {
	IVP_Event_Friction event_friction;
	IVP_Environment *env = obj0->get_environment();
	event_friction.environment = env;
	event_friction.contact_situation= friction_dist->tmp_contact_info; //is inherited
	event_friction.friction_handle = friction_dist;
	env->fire_event_friction_created(&event_friction);
	{
	    if (obj0->flags.collision_listener_exists){
		IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
		clus_man->fire_event_friction_created(obj0, &event_friction);
	    }
	    if (obj1->flags.collision_listener_exists){
		IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
		clus_man->fire_event_friction_created(obj1, &event_friction);
	    }
	}
    }
    
    // IVP_ASSERT(my_env->get_mindist_manager()->debug_test_having_fr_mindist(friction_dist)==IVP_TRUE);
    
    *having_new=IVP_TRUE;
    
    IVP_Friction_System *affected_fs;
    
    if(fr_info0)
	{
	    fr_sys0=fr_info0->l_friction_system;
	    fr_info1=core1->get_friction_info(fr_sys0);
	    if(fr_info1)
		{
		    //both objects have same friction_system
		    fr_sys0->add_dist_to_system(friction_dist);
		    fr_sys0->dist_added_update_pair_info(friction_dist);
		    //fr_sys0->fr_solver.calc_calc_solver(fr_sys0);
		    affected_fs=fr_sys0;
		} else {
		    if(core1->physical_unmoveable)
			{
				// obj1 may have other friction systems. Doesnt matter systems act independently.
			    goto add_second_to_first;
			} else {
			    fr_info1=core1->moveable_core_has_friction_info(); //obj1 is movable and so has only one fr_sys
			    if(fr_info1)
				{
				    //movable object has already a different system, fusion systems
				    //core0->environment->get_friction_manager()->fusion_friction_systems(fr_sys0,fr_info1->l_friction_system);
				    fr_sys0->fusion_friction_systems(fr_info1->l_friction_system);
				    goto add_second_to_first_dist_only;
				} else {
				add_second_to_first:		    
				    // add second core to system of first core
				    fr_info1=new IVP_Friction_Info_For_Core();
				    fr_info1->l_friction_system=fr_sys0;
				    core1->add_friction_info(fr_info1);
				    fr_sys0->add_core_to_system(core1);
		    
				add_second_to_first_dist_only:		    
				    fr_sys0->add_dist_to_system(friction_dist);
				    fr_sys0->dist_added_update_pair_info(friction_dist);
				    //fr_sys0->fr_solver.calc_calc_solver(fr_sys0);
				    affected_fs=fr_sys0;
				}
			}
		}
	} else {
	    //first obj has no friction system, first make info-block
	    fr_info0=new IVP_Friction_Info_For_Core();
	    //core0->add_friction_info(fr_info0);		    

	    if(core1->physical_unmoveable)
		{
		    // obj may have other systems that act independently. Make new system
		    goto new_system_for_both_objects;
		} else {
		    fr_info1=core1->moveable_core_has_friction_info();
		    if(fr_info1)
			{
				// add obj0 to system of obj1
			    fr_sys1=fr_info1->l_friction_system;
			    fr_info0->l_friction_system=fr_sys1;
			    core0->add_friction_info(fr_info0);
			    fr_sys1->add_core_to_system(core0);
			    fr_sys1->add_dist_to_system(friction_dist);
			    fr_sys1->dist_added_update_pair_info(friction_dist);
				//fr_sys1->fr_solver.calc_calc_solver(fr_sys1);
			    affected_fs=fr_sys1;
		
			} else {
				// make a new system for both objects
			new_system_for_both_objects:
			    IVP_Friction_System *new_system;
			    new_system=new IVP_Friction_System(my_env);
			    fr_info1=new IVP_Friction_Info_For_Core();
			    fr_info0->l_friction_system=new_system;
			    fr_info1->l_friction_system=new_system;
			    core1->add_friction_info(fr_info1);		    
			    core0->add_friction_info(fr_info0);		    
		
				//my_env->get_friction_manager()->add_friction_system(new_system);
			    new_system->add_dist_to_system(friction_dist);
			    new_system->dist_added_update_pair_info(friction_dist);
				//new_system->fr_solver.calc_calc_solver(new_system);		
			    new_system->add_core_to_system(core0);
			    new_system->add_core_to_system(core1);
			    affected_fs=new_system;
			}
		}
	}
    
    //allow to go from object to its distances
    fr_info0->friction_info_insert_friction_dist(friction_dist);
    fr_info1->friction_info_insert_friction_dist(friction_dist);
#ifdef EASEONIMPACT
    fr_info0->set_all_dists_of_obj_neutral();
    fr_info1->set_all_dists_of_obj_neutral();
#endif    

    //printf("add_fri_dist %lx to sys %lx\n",(long)friction_dist,(long)affected_fs); UFTEST

    *associated_fs=affected_fs;
    //friction_dist->l_friction_system=affected_fs;
#ifdef DEBUG
    IVP_IF(0) {
	IVP_Friction_System *fs0,*fs1;
	fs0=NULL;
	fs1=NULL;
	if(!core0->physical_unmoveable) {
	    IVP_Friction_Info_For_Core *info_friction=core0->moveable_core_has_friction_info();
	    if(info_friction) {
		fs0=info_friction->l_friction_system;
	    }
	}
	if(!core1->physical_unmoveable) {
	    IVP_Friction_Info_For_Core *info_for_core=core1->moveable_core_has_friction_info();
	    if(info_for_core) {
		fs1=info_for_core->l_friction_system;
	    }
	}
	printf("\nmaking_new_frdist for cores %lx %d  %lx %d  fss %lx %lx  with sim_units: %lx %lx\n",
	       (long)core0,core0->physical_unmoveable,
	       (long)core1,core1->physical_unmoveable,
	       (long)fs0,(long)fs1,(long)core0->sim_unit_of_core,(long)core1->sim_unit_of_core);
	core0->sim_unit_of_core->sim_unit_debug_out();
	core1->sim_unit_of_core->sim_unit_debug_out();
    }
#endif        
    friction_dist->calc_virtual_mass_of_mindist();
    
    IVP_IF(obj0->get_environment()->get_debug_manager()->check_fs) {
	affected_fs->test_hole_fr_system_data();
    }
    
    IVP_Simulation_Unit *sim0,*sim1;
    sim0=core0->sim_unit_of_core;
    sim1=core1->sim_unit_of_core;

    if( !(core1->physical_unmoveable | core0->physical_unmoveable) ) {
	if(sim0!=sim1) {
	    if(sim1==sim_unit_not_destroy) {
		sim1->fusion_simulation_unities(sim0);
		P_DELETE(sim0);
	    } else {
		sim0->fusion_simulation_unities(sim1);
		P_DELETE(sim1);
	    }
	}
    }
#ifdef DEBUG    
    IVP_IF(1) {
	if(!core0->physical_unmoveable) {
	    core0->sim_unit_of_core->sim_unit_debug_consistency();
	}
	if(!core1->physical_unmoveable) {
	    core1->sim_unit_of_core->sim_unit_debug_consistency();
	}
    }
#endif    
    return friction_dist;
}

IVP_DOUBLE IVP_Impact_Solver::get_total_energy(){
    // energy of both objects doing impact
    IVP_DOUBLE ener;
    ener =  core[0]->get_energy_on_test(&trans_speed[0],&rot_speed[0])
	+   core[1]->get_energy_on_test(&trans_speed[1],&rot_speed[1]);
    IVP_IF(core[0]->environment->debug_information->debug_impact)	{
	    IVP_DOUBLE trans_e=trans_speed[1].quad_length() * core[1]->get_mass();
	    IVP_U_Float_Point hp;
	    hp.set_pairwise_mult( &rot_speed[1], &rot_speed[1]);
	    IVP_DOUBLE rot_e = hp.dot_product(core[1]->get_rot_inertia());
	
	    printf("total_energies trans %.3f rot %.3f sum %.3f\n",0.5f*trans_e,0.5f*rot_e,ener);
	}
    return ener;
}

//get relative world speed seen from obj 0
void IVP_Impact_Solver::get_relative_speed_vector()
{
    IVP_U_Float_Point world_speed0,world_speed1;
    core[0]->get_surface_speed_on_test(obj_point[0],&trans_speed[0],&rot_speed[0],&world_speed0);
    core[1]->get_surface_speed_on_test(obj_point[1],&trans_speed[1],&rot_speed[1],&world_speed1);
    relative_world_speed.subtract(&world_speed1,&world_speed0);
    //relative_world_speed is now relative speed seen from now still object 0
}

// calculates push direction vector. length of vector is 1.0
// if vector isnt steep enough for friction, vector is adjusted (made steepeer).
// function is used by 'do_push' and before 'estimate_push_impulse'
void IVP_Impact_Solver::get_world_push_direction()
{
    //given : relative_world_speed;
    //wanted : vector world_push_direction giving direction of push (normized)
    //world_push_direction differs from relative_world_speed if allowed friction is exceeded. then length is also smaller than 1.0f 

    IVP_U_Float_Point part_in_direction_surf; //decomposition of velo vec in direction normal and rest (this is the rest)

    world_push_direction.set(&relative_world_speed);
    world_push_direction.fast_normize();

    IVP_DOUBLE angle=world_push_direction.dot_product(surf_normal); // is negativ. the greater (absolut) the steeper is angle
    //angle is cos alpha . alpha seen from surface normal

    IVP_IF(core[0]->environment->debug_information->debug_impact)
    {
      printf("wpd %.2f %.2f %.2f  ",world_push_direction.k[0],world_push_direction.k[1],world_push_direction.k[2]);
    }
    if((angle>0.0f))
    {
	//turnaround_next_time: objects may still be closing, but is only rest speed that can be neglected, as lowest energy point of impact was already traversed
	IVP_IF(core[0]->environment->debug_information->debug_impact)
	{
	    printf(" impact_speed_now_reversed ");
	}
	//world_push_direction.set(surf_normal);
	//world_push_direction.mult(-1.0f);
	world_push_direction.set(&integral_pushes_world);
	world_push_direction.fast_normize();
	//angle=-angle;
	//world_push_direction.subtract(surf_normal); // impact direction is mirrored relative to surface
	return;
    } else {
      IVP_IF(core[0]->environment->debug_information->debug_impact){
	  printf(" not_reversed ");
	}
    }

    if(two_friction_values==IVP_FALSE) {

#if 0	
	percent_deformation=1.0f;
#endif	
	if((angle>-cos_friction)) //negativ value
	    {
		//angle less cos_friction degree -> no more friction
		//devide world_push_direction in parts in direction of surface and surface normal
		//direction in surface normal is kept. direction in surface is shortened in a way the angle is border-angle
#if 0
//not used at the moment -> needed by single_impact
		percent_deformation=angle/-cos_friction;
#endif
		IVP_DOUBLE mangle = - angle;
		part_in_direction_surf.add_multiple(&world_push_direction,surf_normal,mangle);
		part_in_direction_surf.normize();
		part_in_direction_surf.mult(sin_friction); //part in direction of surface is shortend to be length of sin
	
		//part_in_direction_norm.set_multiple(surf_normal,-cos_friction);
		world_push_direction.add_multiple(&part_in_direction_surf,surf_normal, -cos_friction); //recombine; vector has exactly length 1.0

		//printf("recomb2 %f %f %f\n",world_push_direction.k[0],world_push_direction.k[1],world_push_direction.k[2]);
	    }
    } else {
	get_world_push_direction_two_friction(angle);
    }
}

// when friction values are low, the part in direction surface of world push direction is not allowed to be big
// we calculate the length of the part in direction surface of world push direction
// then we calculate the allowed length
// therefore we calculate the angle in 2D (spanned by the two friction vectors)
// and the allowed length is a position on a 2D ellipse
// when allowed length is shorter, we have to take the shorter value
void IVP_Impact_Solver::get_world_push_direction_two_friction(IVP_DOUBLE part_direction_surf_normal) {
        IVP_U_Float_Point part_in_direction_surf; //decomposition of velo vec in direction normal and rest (this is the rest)
#if 0	
	percent_deformation=1.0f;
#endif	
	IVP_DOUBLE mangle = - part_direction_surf_normal;
	part_in_direction_surf.add_multiple(&world_push_direction,surf_normal,mangle);
	//existing length
	IVP_FLOAT length_direction_surf=part_in_direction_surf.real_length_plus_normize(); 

	//now calculate allowed length
	//we have an ellipse with first radius sin_friction and second radius sin_second_friction
	//second axis is world_direction_second_friction
	IVP_DOUBLE part_second_friction=part_in_direction_surf.dot_product(&world_direction_second_friction);
	
	IVP_FLOAT abs_second_part=IVP_Inline_Math::fabsd(part_second_friction);
	IVP_FLOAT angle_2d = IVP_Inline_Math::asind(abs_second_part);
	IVP_FLOAT axis_first_friction = IVP_Inline_Math::approx5_cos(angle_2d);
	IVP_FLOAT axis_second_friction=abs_second_part; //equals to sin(angle_2d)
	IVP_FLOAT quad_len_allowed=axis_first_friction*axis_first_friction*sin_friction*sin_friction + axis_second_friction*axis_second_friction*sin_second_friction*sin_second_friction;
	
	if( length_direction_surf*length_direction_surf > quad_len_allowed ) {
	    //the combined friction values are too low to allow existing push direction
	    IVP_FLOAT allowed_len = IVP_Inline_Math::ivp_sqrtf(quad_len_allowed);

#if 0	    
	    IVP_FLOAT factor_length;
	    if(length_direction_surf>P_DOUBLE_EPS) {
		factor_length=allowed_len/length_direction_surf;
	    } else {
		factor_length=0.0f;
	    }
	    percent_deformation=factor_length;
#endif
	    
	    IVP_FLOAT length_angle = IVP_Inline_Math::asind(allowed_len);
	    IVP_FLOAT cos_part= IVP_Inline_Math::approx5_cos(length_angle);
	    IVP_U_Float_Point world_push_dir_fl;
	    world_push_dir_fl.set_multiple(surf_normal,-cos_part);
	    world_push_direction.set(&world_push_dir_fl);
	    world_push_direction.add_multiple(&part_in_direction_surf,allowed_len); //length is now exactly 1.0
	}
	    
	    //IVP_DOUBLE part_second_friction=world_push_direction.dot_product(&world_direction_second_friction);
}

void IVP_Impact_Solver::confirm_impact(int core_nr)
{
    // transfer values to object core
    // if objects are unmovale, values are zero
    IVP_Core *c = core[core_nr];
    c->speed.set(&trans_speed[core_nr]);
    c->rot_speed.set(&rot_speed[core_nr]);
    IVP_Anomaly_Limits *al = c->environment->get_anomaly_limits();

    if( c->impacts_since_last_PSI > al->get_max_collisions_per_psi() ) {
	IVP_Anomaly_Manager *am = c->environment->get_anomaly_manager();
	c->temporarily_unmovable =  am->max_collisions_exceeded_check_freezing(al,  c );
    }
}

void IVP_Impact_Solver::undo_push()
{
    // make last push undone
    IVP_IF(core[0]->environment->debug_information->debug_impact)
    {
	printf("impact_undo_push\n");
    }
    if(!core[0]->physical_unmoveable)
    {
	rot_speed[0].subtract(&rot_speed_change[0]);
	trans_speed[0].subtract(&trans_speed_change[0]);
    }
    if(!core[1]->physical_unmoveable)
    {
	rot_speed[1].subtract(&rot_speed_change[1]);    
	trans_speed[1].subtract(&trans_speed_change[1]);
	IVP_IF(core[0]->environment->debug_information->debug_impact)
	{
	    printf("undoing trans %.3f %.3f %.3f\n",trans_speed_change[1].k[0],trans_speed_change[1].k[1],trans_speed_change[1].k[2]);
	}
    }
}

//push_dir_norm: normized push pointer, points to first core
//rescue push gives rotation free pushes when needed, to assure that objects are moving away from from each other
void IVP_Impact_Solver::do_rescue_push(IVP_U_Float_Point *push_dir_norm,IVP_BOOL panic_mode)
{
    //printf("rescuepush\n\n");
        IVP_U_Point world_point_obj0,world_point_obj1;
	m_world_f_core[0]->vmult4(obj_point[0],&world_point_obj0);
	m_world_f_core[1]->vmult4(obj_point[1],&world_point_obj1);
	
	IVP_U_Float_Point diff_vec_world; //vector of position difference. points from first synapse to second
	//diff_vec_world.subtract(&world_point_obj1,&world_point_obj0);
//	IVP_DOUBLE dist_len_now=diff_vec_world.fast_real_length();
	
	//diff_vec_world.normize();
	diff_vec_world.set_multiple(push_dir_norm,-1.0f); //diff_vec_world must point from first to second

	IVP_U_Float_Point translation,rotation,world_speed0,world_speed1;
	
	//rotation.set(&core0->rot_speed);
	rotation.set(&rot_speed[0]);
	//translation.set(&core0->speed);
	translation.set(&trans_speed[0]);
	core[0]->get_surface_speed_on_test(obj_point[0],&translation,&rotation,&world_speed0);

	//rotation.set(&core1->rot_speed);
	rotation.set(&rot_speed[1]);
	//translation.set(&core1->speed);
	translation.set(&trans_speed[1]);
	core[1]->get_surface_speed_on_test(obj_point[1],&translation,&rotation,&world_speed1);

	IVP_U_Float_Point rel_speed_world; //speed of second synapse against first, in world coords
	rel_speed_world.subtract(&world_speed1,&world_speed0);

	IVP_DOUBLE dist_velocity=rel_speed_world.dot_product(&diff_vec_world); //velocity of distance change. negative means getting closer.
	if((panic_mode==IVP_TRUE)) {
	    if(dist_velocity>0.0f) {
		dist_velocity=0.0f; //assure that a positive push is really done
	    }
	}
	
//	IVP_Environment *env = core0->environment;
	//if (dist_velocity > 0) dist_velocity = 0;	// 
	IVP_DOUBLE desired_velo_change= //(ivp_mindist_settings.coll_dist + 2 * ivp_mindist_settings.mindist_change_force_dist - dist_len_now) / env->get_delta_PSI_time() +
	    this->rescue_speed_impact_solver - dist_velocity; 

	if(desired_velo_change<0.0f) {
	    return;
	}
#if 0
	//new version with translation only -> seems to be no good
	IVP_IF(1) {
	    if(panic_mode) {
		core[0]->environment->impact_hard_rescue_counter++;
	    } else {
		core[0]->environment->impact_rescue_after_counter++;
	    }
	}
	
	IVP_DOUBLE speed_change_scalar=0.0f;
	if(!core[0]->physical_unmoveable) {
	    speed_change_scalar+=core[0]->inv_mass;
	}
	if(!core[1]->physical_unmoveable) {
	    speed_change_scalar+=core[1]->inv_mass;
	}
	
	IVP_DOUBLE resulting_push=desired_velo_change/speed_change_scalar;
	
	if(!core[0]->physical_unmoveable) {
	    trans_speed_change[0].set_multiple(push_dir_norm,resulting_push*core[0]->inv_mass);
	    trans_speed[0].add(&trans_speed_change[0]); // if push has to be undone, values are subtracted later
	}	    

	if(!core[1]->physical_unmoveable) {
	    trans_speed_change[1].set_multiple(push_dir_norm,-resulting_push*core[1]->inv_mass);
	    trans_speed[1].add(&trans_speed_change[1]); // if push has to be undone, values are subtracted later
	}	    
	
	
#else //old version: translation and rotation	
	IVP_U_Float_Point push_vec_obj;
	IVP_U_Float_Point push_vec_world;
	IVP_U_Float_Point rotation_vec;
	IVP_U_Float_Point translation_vec;
	IVP_U_Float_Point speed_change_vec;
	IVP_DOUBLE speed_change_scalar=0.0f;
	
	
	push_vec_world.set(&diff_vec_world);
	IVP_U_Matrix *mat;
	
	if(!core[1]->physical_unmoveable)
	{
	    //give second obj a test push and look how distance reacts
	    mat=m_world_f_core[1];
	    mat->vimult3(&push_vec_world,&push_vec_obj);

      	    core[1]->test_push_core(obj_point[1],&push_vec_obj,&push_vec_world,&translation_vec,&rotation_vec);	    
	    core[1]->get_surface_speed_on_test(obj_point[1],&translation_vec,&rotation_vec,&speed_change_vec);
	    speed_change_scalar+=push_vec_world.dot_product(&speed_change_vec);
	}

	if(!core[0]->physical_unmoveable)
	{
	    //give first obj a test push and look how distance reacts
	    mat=m_world_f_core[0];
	    push_vec_world.mult(-1.0f);
	    mat->vimult3(&push_vec_world,&push_vec_obj);

	    core[0]->test_push_core(obj_point[0],&push_vec_obj,&push_vec_world,&translation_vec,&rotation_vec);
	    core[0]->get_surface_speed_on_test(obj_point[0],&translation_vec,&rotation_vec,&speed_change_vec);
	    speed_change_scalar+=push_vec_world.dot_product(&speed_change_vec);
	}

	IVP_DOUBLE resulting_push=desired_velo_change/(speed_change_scalar + 1e-15f);
	if(resulting_push<0.0f) {
	    return;
	}

	IVP_IF(1) {
	    if(panic_mode) {
		core[0]->environment->get_statistic_manager()->impact_hard_rescue_counter++;
	    } else {
		core[0]->environment->get_statistic_manager()->impact_rescue_after_counter++;
	    }
	}
	
	IVP_IF(core[0]->environment->debug_information->debug_impact||0)
	    {
		printf("doing_res_push %f dvc %f\n",resulting_push,desired_velo_change);
	    }
		
	IVP_U_Float_Point obj_push, world_push;
	
	if(!core[0]->physical_unmoveable)
	{
	    mat=m_world_f_core[0];
	    world_push.set_multiple(&diff_vec_world, -resulting_push); //negativ: dist_vector_world points to second core
	    mat->vimult3(&world_push,&obj_push);
	    //core0->push_core(obj_point0,&obj_push,&world_push);
	    //optimization, but debugging impossible, instead:
	    core[0]->test_push_core(obj_point[0],&obj_push,&world_push,&trans_speed_change[0],&rot_speed_change[0]);
	    rot_speed[0].add(&rot_speed_change[0]); // ...speed0 has sum of pushes and beginning speed before impact
	    trans_speed[0].add(&trans_speed_change[0]); // if push has to be undone, values are subtracted later
	    
	}
	
	if(!core[1]->physical_unmoveable)
	{
	    mat=m_world_f_core[1];
	    world_push.set_multiple(&diff_vec_world, resulting_push); //negativ: dist_vector_world points to second core
	    mat->vimult3(&world_push,&obj_push);
	    //core1->push_core(obj_point1,&obj_push,&world_push); 
	    //optimization, but debugging impossible, instead:
	    core[1]->test_push_core(obj_point[1],&obj_push,&world_push,&trans_speed_change[1],&rot_speed_change[1]);
	    rot_speed[1].add(&rot_speed_change[1]); // ...speed0 has sum of pushes and beginning speed before impact
	    trans_speed[1].add(&trans_speed_change[1]); // if push has to be undone, values are subtracted later
	}
#endif	
}

void IVP_Impact_Solver::do_push_on_core(IVP_U_Float_Point *push_vec_world,int num_core) {
    IVP_U_Float_Point push_vec_obj;
    IVP_U_Matrix *mat=m_world_f_core[num_core];
    mat->vimult3(push_vec_world,&push_vec_obj);

    core[num_core]->test_push_core(obj_point[num_core],&push_vec_obj,push_vec_world,&trans_speed_change[num_core],&rot_speed_change[num_core]);
    rot_speed[num_core].add(&rot_speed_change[num_core]); // ...speed0 has sum of pushes and beginning speed before impact
    trans_speed[num_core].add(&trans_speed_change[num_core]); // if push has to be undone, values are subtracted later
}


// first_push and second_push are pointers to cores that are pushed instantly (if not set to NULL)
// when allow_delaying is TRUE, one core may rest untouched and is changed in next PSI
void IVP_Impact_Solver::do_impact(IVP_Core *pushed_cores[2],IVP_BOOL allow_delaying,int pushes_while_system,IVP_FLOAT rescue_speed_addon)
{
    // impact of surface (obj0) and point (obj1)
    // impulse is estimated, but problem cant be solved in one step -> iteration of pushes
    // there are done several pushes with push direction changing, when desired energy level interval is reached : Stop
    // there may be situations where energy level cant be reached ( very flat impact angle). Then do a single short push to avoid (hopefully) collision: do_rescue_push

    //printf("\n\n\nimpacting\n\n\n");
    IVP_DOUBLE virtual_speed,give_back_speed = 0.0f,speed_before;
    IVP_DOUBLE relative_trans_speed_before;
    IVP_DOUBLE used_conservation;

    IVP_IF(1) {
	core[0]->environment->get_debug_manager()->all_time_impacts += 1.0f;
    }

    used_conservation = 1.0f - (1.0f/(pushes_while_system*IVP_INV_HALF_CONSERVATION_STEPS+1.0f))
	* (1.0f-percent_energy_conservation);
    
    this->rescue_speed_impact_solver= (IVP_FLOAT)(ivp_mindist_settings.min_vertical_speed_at_collision + rescue_speed_addon) * IVP_SAFETY_FACTOR_FOR_DELAY;
    
    delaying_is_allowed=allow_delaying;
    //delaying_is_allowed=IVP_FALSE;
    pushed_cores[0]=core[0];
    pushed_cores[1]=core[1];
     
    core[0]->environment->get_statistic_manager()->impact_counter++;
        
    m_world_f_core[0]	=	&core[0]->m_world_f_core_last_psi;
    m_world_f_core[1] =	&core[1]->m_world_f_core_last_psi;

    IVP_DOUBLE mass0,mass1;
    mass0 = core[0]->get_mass();
    mass1 = core[1]->get_mass();
    if(core[0]->physical_unmoveable || core[0]->pinned)    { //@@CBPIN
	mass0=mass1 * 10000000.0f;
    }

    if(core[1]->physical_unmoveable || core[1]->pinned)    { //@@CBPIN
	mass1=mass0 * 10000000.0f;
    }

    IVP_IF(0){
	rot_speed[0]  .set( &core[0]->rot_speed );
	trans_speed[0].set( &core[0]->speed);
	rot_speed[1]  .set( &core[1]->rot_speed);
	trans_speed[1].set( &core[1]->speed);

	this->get_relative_speed_vector();
	IVP_DOUBLE speed_parallel_mindist = relative_world_speed.dot_product(surf_normal); //normally negative value
	printf("impact %.20G\n",speed_parallel_mindist); //@@CB
    }
    
    rot_speed[0]  .add( &core[0]->rot_speed, &core[0]->rot_speed_change);
    trans_speed[0].add( &core[0]->speed,     &core[0]->speed_change);
    rot_speed[1]  .add( &core[1]->rot_speed, &core[1]->rot_speed_change);
    trans_speed[1].add( &core[1]->speed,     &core[1]->speed_change);
    
    IVP_IF(1) {
	IVP_U_Float_Point trans_vec;
	trans_vec.subtract( &trans_speed[0],&trans_speed[1] );
	relative_trans_speed_before=trans_vec.real_length();
    }
    
    //IVP_DOUBLE full_energy = this->get_total_energy();
    
    energy_deformation= 0.0f;
    
    //target_energy_high = full_energy;
    //target_energy_low  = target_energy_high;

    this->get_relative_speed_vector();
    speed->set(&relative_world_speed); //return value
    
    int push_counter=0;

    IVP_DOUBLE impulse=this->estimate_push_impulse() * 0.1f; //do only 10 percent pushes

    IVP_DOUBLE speed_parallel_mindist = relative_world_speed.dot_product(surf_normal); //normally negative value

    if(speed_parallel_mindist > -IMPACT_EPS)
    {
	// something went wrong, do a rescue push
        //printf("bad_rescue_push\n");
        IVP_U_Float_Point push_dir;	push_dir.set_negative(surf_normal);
	do_rescue_push(&push_dir,IVP_TRUE); //panic
	//confirm_impact(0);
	//confirm_impact(1);
	//clear_change_values_cores();
	
	//return IVP_OK;
	if(pushes_while_system>10) {
	    // I think I am jammed 
	    delaying_is_allowed=IVP_FALSE;
	}
	goto delaying_test;
    }

    //maybe do an extra rescue_push when speed is very low, this time with friction

    IVP_IF(core[0]->environment->debug_information->debug_impact)    {
	printf("speed %.4f %.4f %.4f rot %.4f %.4f %.4f\n",trans_speed[1].k[0],trans_speed[1].k[1],trans_speed[1].k[2],rot_speed[1].k[0],rot_speed[1].k[1],rot_speed[1].k[2]);
    }
    
    this->get_world_push_direction();

    //this->calc_virt_masses_impact_solver(&world_push_direction);
    
    //last_energy=full_energy;

    integral_pushes_world.set_to_zero();
    virtual_speed=-surf_normal->dot_product(&relative_world_speed); // alternative: world_push_direction->dot_pro..
    speed_before=virtual_speed;
    IVP_ASSERT(virtual_speed>0.0f);

    give_back_speed = virtual_speed*IVP_Inline_Math::ivp_sqrtf(used_conservation);
    give_back_speed+=IVP_SPEED_ADDON_SYSTEM_IMPACT * IVP_Inline_Math::ivp_sqrtf((IVP_DOUBLE)pushes_while_system);
    
    while((virtual_speed > 0.0f)&&(push_counter<100)) {
	push_counter++;
	do_push(impulse);
	//integral_pushes_world.add(&world_push_direction);
	get_relative_speed_vector();
	virtual_speed=-surf_normal->dot_product(&relative_world_speed);
	get_world_push_direction();
    }

    give_back_speed += virtual_speed; //virtual_speed is negative, and that speed is already given back
    world_push_direction.set_multiple(surf_normal,-1.0f);
    //world_push_direction.set(&integral_pushes_world);
    //world_push_direction.normize();
    if((give_back_speed > 0.0f)&&(push_counter!=100)) {
	do_push(1); //test push @@CB
	get_relative_speed_vector();
	IVP_DOUBLE speed_test=surf_normal->dot_product(&relative_world_speed);
	IVP_ASSERT(( speed_test > -virtual_speed));
	IVP_DOUBLE speed_diff=speed_test+virtual_speed;
                IVP_DOUBLE target_push;
				// was fabs, which was a sml call
                if( IVP_Inline_Math::fabsd(speed_diff) > IMPACT_EPS) { 
		target_push=give_back_speed/(speed_diff);
                } else {
		target_push=0.0f; //@@CB double to float
                }
	undo_push();
	do_push(target_push);
    }

    //energy_at_end=this->get_total_energy();
    //factor=energy_at_end / full_energy;
    //printf("energy_change_factor_impact %f\n",factor);
      
    do_rescue_push(&world_push_direction,IVP_FALSE); //to be sure that minimal velocity is reached, not panic

    IVP_IF(1==1)
    {
	IVP_U_Float_Point rel_speed_now;
	IVP_Core::get_diff_surface_speed_of_two_cores_on_test(core[0], core[1],obj_point[0],obj_point[1],&trans_speed[0],&rot_speed[0],&trans_speed[1],&rot_speed[1],&rel_speed_now);
	//printf("world_speed_after_imp %.3f %.3f %.3f  ",rel_speed_now.k[0],rel_speed_now.k[1],rel_speed_now.k[2]);
	//printf("rel_sppeed %.4f\n",rel_speed_now.dot_product(surf_normal));
	if(rel_speed_now.dot_product(surf_normal)>0.0f)
	{
	    IVP_IF(1) { printf("impact_objects_have_illegal_speed_afterwards\n"); }
	    // CORE;
	}
    }
    
delaying_test:
    
    IVP_IF(0) {
    	get_relative_speed_vector();
	IVP_DOUBLE testtest=surf_normal->dot_product(&relative_world_speed);
	IVP_USE(testtest);
	IVP_DOUBLE speed_gain=testtest-speed_before; //speed before not initialized when hard rescue
	if(speed_gain>0.0f) {
	    if(speed_gain > core[0]->environment->get_statistic_manager()->max_speed_gain) {
		core[0]->environment->get_statistic_manager()->max_speed_gain=speed_gain;
	    }
	}
    }

    core[0]->clip_velocity(&trans_speed[0], &rot_speed[0]);
    core[1]->clip_velocity(&trans_speed[1], &rot_speed[1]);
    
    //printf("speed_after_imp %f %f  %f %f\n",trans_speed[0].real_length(),rot_speed[0].real_length(),trans_speed[1].real_length(),rot_speed[1].real_length());

    delay_decision(pushed_cores);

    if( core[0]->temporarily_unmovable | core[1]->temporarily_unmovable ) {
	IVP_IF(0) {
	    printf("switched_to_temporarily_unmov %lx %lx %f\n",(long)core[0],(long)core[1],core[0]->environment->get_current_time().get_time());
	}
	core[0]->environment->get_statistic_manager()->impact_unmov++;

	{
	    IVP_Core *c = core[0];
	    c->temporarily_unmovable=(IVP_BOOL)(1-(int)c->physical_unmoveable);
	    c->speed_change.add(&c->speed);
	    c->rot_speed_change.add(&c->rot_speed);
	    c->speed.set_to_zero();
	    c->rot_speed.set_to_zero();
	    pushed_cores[0]=c;
	}
	{
	    IVP_Core *c = core[1];
	    c->temporarily_unmovable=(IVP_BOOL)(1-(int)c->physical_unmoveable);
	    c->speed_change.add(&c->speed);
	    c->rot_speed_change.add(&c->rot_speed);
	    c->speed.set_to_zero();
	    c->rot_speed.set_to_zero();
	    pushed_cores[1]=c;
	}
    }
    
    //this->confirm_impact();
    return;
}

void IVP_Impact_Solver::delay_decision(IVP_Core *pushed_cores[2]) {
    //for movables increase impact counter 
    core[0]->impacts_since_last_PSI+=(1-(int)core[0]->physical_unmoveable);
    core[1]->impacts_since_last_PSI+=(1-(int)core[1]->physical_unmoveable);
    
    if(delaying_is_allowed) {
		
	int try_to_delay_core;
	int other_core;

	if(virt_mass[0]>virt_mass[1]) {
	    try_to_delay_core=0;
	    other_core = 1;
	} else {
	    try_to_delay_core=1;
	    other_core=0;
	}

	if(pushed_cores[try_to_delay_core]->physical_unmoveable) {
	    // no delaying on unmoveable cores
	    goto no_delaying;
	}
	
	//if(pushed_cores[try_to_delay_core]->get_current_sim_man_slot() > 0) {
	    // this is a core that is not simulated every psi -> not any longer
	//goto no_delaying;
	    //}

	IVP_U_Float_Point delayer_trans_speed, delayer_rot_speed;
	IVP_U_Float_Point   other_trans_speed, other_rot_speed;
	
	other_rot_speed.set   (&rot_speed  [other_core]);	// use speeds after impact
	other_trans_speed.set (&trans_speed[other_core]);
	delayer_rot_speed.set (&core [try_to_delay_core]->rot_speed);	// use original speeds
	delayer_trans_speed.set(&core[try_to_delay_core]->speed);
	
	IVP_DOUBLE closing_speed;
	{
	    IVP_U_Float_Point rel_world_speed;
	    IVP_Core::get_diff_surface_speed_of_two_cores_on_test(core[try_to_delay_core], core[other_core],
							      obj_point[try_to_delay_core],obj_point[other_core],
							      &delayer_trans_speed, &delayer_rot_speed,
							      &other_trans_speed,   &other_rot_speed,
							      &rel_world_speed);
	    closing_speed = rel_world_speed.dot_product(surf_normal);
	    if(try_to_delay_core) {
		closing_speed *= -1.0f;
	    }
	}

	if(closing_speed< -this->rescue_speed_impact_solver * IVP_INV_SAFETY_FACTOR_FOR_DELAY) {
	    core[0]->environment->get_statistic_manager()->impact_delayed_counter++;
	    pushed_cores[try_to_delay_core]=NULL;
	    clear_change_values_cores();
	    confirm_impact(1-try_to_delay_core);
	    //printf("delaying_a_push\n");
	    delay_of_impact(try_to_delay_core);
	    core[try_to_delay_core]->impacts_since_last_PSI--;
	} else {
	    goto no_delaying;
	}
    } else {
no_delaying:
	clear_change_values_cores();
	confirm_impact(0);
	confirm_impact(1);
    }
}

void IVP_Impact_Solver::delay_of_impact(int delayed_core_nr) {
    //calc needed change values
    IVP_Core *c = core[delayed_core_nr];
    c->speed_change.subtract (   &trans_speed[delayed_core_nr], &c->speed);
    c->rot_speed_change.subtract (&rot_speed[delayed_core_nr],  &c->rot_speed);
    //c->environment->add_delayed_push_core(c);
}

void IVP_Impact_Solver::clear_change_values_cores()
{
    core[0]->rot_speed_change.set_to_zero();
    core[0]->speed_change.set_to_zero();
    core[1]->rot_speed_change.set_to_zero();
    core[1]->speed_change.set_to_zero();    
}

// gives impulse push on both objects
// direction (pointing to second obj) is world_push_direction (is normized)
// push quantity is impulse_val
void IVP_Impact_Solver::do_push(IVP_DOUBLE impulse_val)
{
    IVP_U_Float_Point obj_push;
    IVP_U_Float_Point world_push;

    world_push.set(&world_push_direction);

    IVP_IF(core[0]->environment->debug_information->debug_impact)    {
	IVP_U_Float_Point surf_dir;
	surf_dir.set(0.0f,1.0f,0.0f);
	IVP_DOUBLE surf_cos=surf_dir.dot_product(&world_push_direction);
	printf("push_dir_cos %f\n",surf_cos);
    }
    
    world_push.mult(impulse_val); // vector gets longer with impulse_val

    if(!core[0]->physical_unmoveable)    {
        IVP_IF(core[0]->environment->debug_information->debug_impact)
	{
	    //printf("p %.2f %.2f %.2f ",world_push_direction.k[0],world_push_direction.k[1],world_push_direction.k[2]);
	}
	m_world_f_core[0]->inline_vimult3(&world_push,&obj_push); // obj_push now in object coords
	core[0]->test_push_core(obj_point[0],&obj_push,&world_push,&trans_speed_change[0],&rot_speed_change[0]);
	rot_speed[0].add(&rot_speed_change[0]); // ...speed0 has sum of pushes and beginning speed before impact
	trans_speed[0].add(&trans_speed_change[0]); // if push has to be undone, values are subtracted later
    }

    if(!core[1]->physical_unmoveable)    {
	world_push.mult(-1.0f);
	IVP_IF(core[0]->environment->debug_information->debug_impact)
	{
	    //printf("p %.2f %.2f %.2f ",world_push_direction.k[0],world_push_direction.k[1],world_push_direction.k[2]);
	}
	m_world_f_core[1]->inline_vimult3(&world_push,&obj_push);
	core[1]->test_push_core(obj_point[1],&obj_push,&world_push,&trans_speed_change[1],&rot_speed_change[1]);
	IVP_IF(core[0]->environment->debug_information->debug_impact)
	{
	    //printf("did_push trans %.3f %.3f %.3f add %.3f %.3f %.3f\n",trans_speed1.k[0],trans_speed1.k[1],trans_speed1.k[2],trans_speed1_change.k[0],trans_speed1_change.k[1],trans_speed1_change.k[2]);
	}
	rot_speed[1].add(&rot_speed_change[1]);
	trans_speed[1].add(&trans_speed_change[1]);
    } 
}

void IVP_Impact_Solver::calc_virt_masses_impact_solver(const IVP_U_Float_Point *world_direction_normal) {
    int i;
    IVP_U_Float_Point direction_world;
    direction_world.set(world_direction_normal);
    for(i=1;i>=0;i--) {
	IVP_U_Float_Point direction_obj;
	m_world_f_core[i]->vimult3(&direction_world,&direction_obj);
	this->virt_mass[i]=core[i]->calc_correct_virt_mass(obj_point[i],(IVP_Vec_PCore *)&direction_obj,&direction_world);
	direction_world.mult(-1.0f);
    }
    
    if(core[0]->physical_unmoveable)    {
	virt_mass[0]=virt_mass[1]*100000.0f; //@@CBPIN
    }
    if(core[1]->physical_unmoveable)    {
	virt_mass[1]=virt_mass[0]*100000.0f; //@@CBPIN
    }
}

IVP_DOUBLE IVP_Impact_Solver::estimate_push_impulse()
{
    // estimates impulse, impulse is always > 0
    // virtual masses are along relative_world_speed , alternative would be to change relative_world_speed according (not existing) friction and then calc virtual mass
    // no longer!

    // get virtual masses along surface normal
    // get speed in direction of surface normal
    // impulse is mass * velocity

    calc_virt_masses_impact_solver(surf_normal);
      
    IVP_DOUBLE push_v0,push_v1;
    push_v0=0.0f;

    //printf("absolute_velocity %.2f  ",relative_world_speed.real_length());
    
    push_v1= - relative_world_speed.dot_product(surf_normal);

    IVP_DOUBLE u0,u1; //speed results after push
    IVP_DOUBLE div_masses=1.0f/(virt_mass[0]+virt_mass[1]);

    //not maximal fast
    u0=(2.0f*virt_mass[1]*push_v1+push_v0*(virt_mass[0]-virt_mass[1]));
    u1=(2.0f*virt_mass[0]*push_v0+push_v1*(virt_mass[1]-virt_mass[0]));
    u0*=div_masses;
    u1*=div_masses;

    IVP_DOUBLE diff_v0=u0-push_v0;
    //IVP_DOUBLE diff_v1=u1-push_v1; //difference of velocity before/after impact

    IVP_IF(core[0]->environment->debug_information->debug_impact)
    {
	printf("start_impulse %.2f\n",diff_v0*virt_mass[0]);	
    }
    return(diff_v0*virt_mass[0]);    
}

// #+# simplify this function a lot
void IVP_Impact_Solver_Long_Term::do_impact_of_two_objects(IVP_Mindist *mindist,  IVP_Real_Object *obj0, IVP_Real_Object *obj1 )
{
	// coll_point in object_coordinates
	// surface_normal in world_coords

	// environment->debug_information->psi_synchrone=1;

	// mindist->synapse[0]->l_obj->to_real()->revive_object_for_simulation();
	// mindist->synapse[1]->l_obj->to_real()->revive_object_for_simulation();

	IVP_Friction_System* affected_friction_system;
	IVP_BOOL having_new_dist;

	IVP_Core *core0 = obj0->get_core();
	IVP_Core *core1 = obj1->get_core();
	IVP_Simulation_Unit *sim_u;
	if( obj0->get_movement_state() >= IVP_MT_NOT_SIM ) {
		sim_u = core0->sim_unit_of_core;
	} else {
		sim_u = core1->sim_unit_of_core;
	}

	IVP_Contact_Point* my_fr_dist = mindist->try_to_generate_managed_friction(&affected_friction_system,&having_new_dist,sim_u,IVP_TRUE); //with recalc_friction_s_vals
	IVP_Impact_Solver_Long_Term* imp_solve = my_fr_dist->tmp_contact_info;

	IVP_Time now_time, last_time;
	IVP_Friction_Core_Pair* pair = affected_friction_system->find_pair_of_cores(core0,core1);
	now_time = mindist->get_environment()->get_current_time();
	last_time = pair->last_impact_time_pair;
	pair->last_impact_time_pair = now_time;
	IVP_Environment* env = mindist->get_environment();

	// printf("impactt\n");
	IVP_Core *impacteers[2];

	IVP_FLOAT rescue_val_addon = my_fr_dist->get_rescue_speed_impact(obj0->get_environment());

	///////////////////////////////////////////////////////////////////////////
	//@@VALVE
	///////////////////////////////////////////////////////////////////////////
	IVP_Event_Collision event_collision;
	event_collision.d_time_since_last_collision = now_time - last_time;
	event_collision.environment = env;
	event_collision.contact_situation = imp_solve; //is inherited
	env->fire_event_pre_collision( &event_collision );
	{
		if (obj0->flags.collision_listener_exists) 
		{
			IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
			clus_man->fire_event_pre_collision(obj0, &event_collision);
		}

		if (obj1->flags.collision_listener_exists) 
		{
			IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
			clus_man->fire_event_pre_collision(obj1, &event_collision);
		}
	}

	imp_solve->do_impact_long_term(impacteers,rescue_val_addon,my_fr_dist);

	IVP_U_Float_Point speed_for_callback;
	speed_for_callback.set(&imp_solve->speed);
	if( core1->physical_unmoveable ) {
		speed_for_callback.mult(-1.0f); //IVP_Impact_Solver_Long_Term changes order when core1 is unmovable @@CB fptodp
	}

//	if((having_new_dist==IVP_FALSE)||1) 
	{
		IVP_Impact_System imp_sys;
		imp_sys.init_and_solve_impact_system(mindist, affected_friction_system, pair, my_fr_dist);
	}

	// copy changed values
	event_collision.contact_situation = imp_solve; //is inherited
	imp_solve->speed.set(&speed_for_callback);
	env->fire_event_post_collision(&event_collision);

	{
		if (obj0->flags.collision_listener_exists) 
		{
			IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
			clus_man->fire_event_post_collision(obj0, &event_collision);
		}

		if (obj1->flags.collision_listener_exists) 
		{
			IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
			clus_man->fire_event_post_collision(obj1, &event_collision);
		}
	}
}


#if 0
    if(0) { //merge example!!
        IVP_Core *core_to_merge=mindist->get_synapse(0)->get_object()->physical_core;
	IVP_Core *other_core0=mindist->get_synapse(1)->get_object()->physical_core;

	if(core_to_merge->physical_unmoveable){
	    IVP_Core *h = core_to_merge; core_to_merge = other_core0; other_core0 = h;
	}
	IVP_ASSERT(core_to_merge->physical_unmoveable == IVP_FALSE);

	IVP_Core *other_core1;
	other_core1=IVP_Impact_Solver_Long_Term::find_second_critical_impact_core(core_to_merge,other_core0);
	
	if(other_core1!=NULL){
	    IVP_Core *best_other_core = IVP_Impact_Solver_Long_Term::get_best_merge_core(core_to_merge,other_core0,other_core1);
	    core_to_merge->create_collision_merged_core_with(best_other_core);
	}
    }
#endif

//lwss add
void IVP_Contact_Point::recompute_friction()
{
    IVP_Environment *environment = this->l_friction_system->l_environment;
    IVP_U_Memory *memory = environment->short_term_mem;
    memory->start_memory_transaction();
    this->recalc_friction_s_vals(); // this doesn't take an IVP_Environment* ??
    memory->end_memory_transaction();
}
//lwss end
void IVP_Contact_Point::get_material_info(IVP_Material *mtl[2]) {
    int k;
    for(k=0;k<2;k++) {
    	int mat_index = this->get_synapse(k)->get_material_index();
	if (mat_index == 0){
	    mtl[k] = get_synapse(k)->get_object()->l_default_material;
	}else{
	    IVP_Environment *env = get_synapse(0)->get_object()->get_environment();
	  mtl[k] = env->get_material_manager()->get_material_by_index(0,mat_index);  // @@@@@
	}
	IVP_ASSERT(mtl[k]);
    }
}

void IVP_Impact_Solver::get_cos_sin_for_impact(IVP_FLOAT friction_val,IVP_FLOAT percent_energy_conservation,IVP_FLOAT *cos_val,IVP_FLOAT *sin_val) {
    IVP_DOUBLE impact_fri_fact=friction_val * (1.0f + IVP_Inline_Math::ivp_sqrtf(percent_energy_conservation));
    IVP_DOUBLE fri_angle = IVP_Inline_Math::atand(impact_fri_fact);  // #+# bitter, kill, 
    IVP_DOUBLE c = IVP_Inline_Math::approx5_cos(fri_angle);
    *cos_val = c;
    *sin_val = c * impact_fri_fact;
}


void IVP_Contact_Point::read_materials_for_contact_situation(IVP_Impact_Solver_Long_Term *info) {

    
    IVP_Real_Object *obj0,*obj1;
    obj0 = get_synapse(0)->get_object(); //syn0->get_ivp_polygon();
    obj1 = get_synapse(1)->get_object(); //syn1->get_ivp_polygon();

    get_material_info(info->materials);
    
    IVP_Environment *env = obj0->get_environment();

    //////// IVP_Contact_Situation
    info->objects[0] = obj0;	// IVP_Contact_Situation
    info->objects[1] = obj1;

    info->compact_edges[0] = get_synapse(0)->edge;
    info->compact_edges[1] = get_synapse(1)->edge;


    IVP_ASSERT(obj0 != obj1);    
    IVP_Material_Manager *mtlm = env->get_material_manager();
    
    info->impact.percent_energy_conservation  = mtlm->get_elasticity(info);

    this->real_friction_factor = mtlm->get_friction_factor(info);
}


extern IVP_Mindist *g_pCurrentMindist;
extern bool g_fDeferDeleteMindist;
// assert:	recalc_mindist 	called
// assert:	mindist of EXACT
// actions perform the impact of two real (!) objects with all !!! side effects
// #+# simplify, move synchronize and revive to do_impact_of_two_objects
void IVP_Mindist::do_impact(){
	g_pCurrentMindist = this;
    IVP_Environment *env;
    env=get_environment();
    
    IVP_ASSERT( mindist_status == IVP_MD_EXACT);

    // move variables too impact system
    IVP_Real_Object *objects[2];
    
    for (int i = 0;i<2;i++){
      IVP_Synapse_Real *syn = this->get_synapse(i);
	IVP_Real_Object  *obj= syn->l_obj;
	objects[i] = obj;
	obj->revive_object_for_simulation();
    }

	g_pCurrentMindist = NULL;
	if (g_fDeferDeleteMindist)
	{
		// BUGBUG: someone changed a collision filter and didn't tell us!
		IVP_ASSERT(0);
		delete this;
		return;
	}

    //revive_object_core is calling start_memory_transaction and end_memory_transaction, so we cannot start memory transaction earlier
    env->sim_unit_mem->start_memory_transaction();

    for(int j=0;j<2;j++) {
	IVP_Core *core=objects[j]->get_core();
	if ( IVP_MTIS_SIMULATED(core->movement_state) && !core->pinned){ //@@CB
	  core->synchronize_with_rot_z();
	}	
    }    
    
    env->mindist_event_timestamp_reference++;

    IVP_Impact_Solver_Long_Term::do_impact_of_two_objects(this, objects[0], objects[1]   );
    env->sim_unit_mem->end_memory_transaction();
}

//find second critical core for core0 that is not core1
IVP_Core *IVP_Impact_Solver_Long_Term::find_second_critical_impact_core(IVP_Core *core0,IVP_Core *core1)
{
    
    for(int c= core0->objects.len()-1;c>=0; c--){
	IVP_Real_Object *r_obj=core0->objects.element_at(c);
	for(IVP_Synapse_Real *synr=r_obj->get_first_exact_synapse();synr;synr=synr->get_next())
	{
	    IVP_Mindist *mdist=synr->get_mindist();
	    //if(mdist->coll_watcher.nr_collisions_per_psi>4) {
	    if(1) { //change that!
		for(int i=0;i<2;i++) {
		    IVP_Synapse_Real *syn;
		    syn=mdist->get_synapse(i);
		   IVP_Real_Object *obj=syn->get_object();
		   if((obj->physical_core!=core0)&&(obj->physical_core!=core1)) {
		       return obj->physical_core;
		   }
		}
	    }
	}	
    }
    return NULL;
}

//merge (movable) core0 with core1 or core2. return best alternative
IVP_Core *IVP_Impact_Solver_Long_Term::get_best_merge_core(IVP_Core *core0,IVP_Core *core1,IVP_Core *core2)
{
    IVP_Friction_Info_For_Core *fr_i=core0->moveable_core_has_friction_info();
    if(!fr_i) {
	return core1; //friction is disabled 
    }
    IVP_Friction_System *fr_sys=fr_i->l_friction_system;

    int anz_1=0;
    int anz_2=0;
    for (int i = fr_sys->fr_pairs_of_objs.len()-1; i>=0;i--){
	IVP_Friction_Core_Pair *my_pair = fr_sys->fr_pairs_of_objs.element_at(i);
	int my_1_nr;
	if(my_pair->objs[0]==core0){
	    my_1_nr=1;
	} else {
	    my_1_nr=0;
	}
	if(my_pair->objs[my_1_nr]==core1){
	    anz_1=my_pair->number_of_pair_dists();
	}
	if(my_pair->objs[my_1_nr]==core2)	{
	    anz_2=my_pair->number_of_pair_dists();
	}
    }
    if(anz_2>anz_1) {
	return core2;
    } else {
	return core1;
    }
}


void IVP_Impact_Solver_Long_Term::do_impact_long_term(IVP_Core *pushed_cores[2],IVP_FLOAT rescue_speed_val,IVP_Contact_Point *cp)
{
    IVP_Impact_Solver imp_solver_stack;
    IVP_U_Float_Point surf_normal_stack; 
    if(!contact_core[1]) {

	imp_solver_stack.core[0] = objects[1]->get_core();
	imp_solver_stack.core[1] = contact_core[0];
	
	imp_solver_stack.obj_point[0]=&contact_point_cs[1];
	imp_solver_stack.obj_point[1]=&contact_point_cs[0];
	surf_normal_stack.set_negative(&surf_normal);
	imp_solver_stack.surf_normal=&surf_normal_stack;
    } else {
	imp_solver_stack.core[0]=contact_core[0];
	if (!contact_core[0]){
	    imp_solver_stack.core[0] = objects[0]->get_core();
	}
	imp_solver_stack.core[1]=contact_core[1];
	imp_solver_stack.obj_point[0]=&contact_point_cs[0];
	imp_solver_stack.obj_point[1]=&contact_point_cs[1];
	imp_solver_stack.surf_normal=&surf_normal;
    }
    imp_solver_stack.speed=&speed; //return value
    imp_solver_stack.percent_energy_conservation = this->impact.percent_energy_conservation;
    imp_solver_stack.two_friction_values=IVP_FALSE;

	if ( imp_solver_stack.core[0]->car_wheel || imp_solver_stack.core[1]->car_wheel ) {
		imp_solver_stack.get_cos_sin_for_impact(0.0f, this->impact.percent_energy_conservation,&imp_solver_stack.cos_friction,&imp_solver_stack.sin_friction);
	}else{
		imp_solver_stack.get_cos_sin_for_impact(cp->real_friction_factor, this->impact.percent_energy_conservation,&imp_solver_stack.cos_friction,&imp_solver_stack.sin_friction);
		if(cp->two_friction_values) {
		imp_solver_stack.get_world_direction_second_friction(cp);//imp_solver_stack.two_friction_values=cp->get_world_direction_second_friction(&imp_solver_stack.world_direction_second_friction,&imp_solver_stack.sin_second_friction);
		}
	}
    IVP_IF(1) {
	if (contact_core[0]) contact_core[0]->core_plausible_check();
	if (contact_core[1]) contact_core[1]->core_plausible_check();
    }
    imp_solver_stack.do_impact(pushed_cores,IVP_TRUE,impacts_while_system,rescue_speed_val);
    IVP_IF(1) {
	if (contact_core[0]) contact_core[0]->core_plausible_check();
	if (contact_core[1]) contact_core[1]->core_plausible_check();
    }
}

//call recalc_friction_s_vals for all fr_dists except the initial one
//is nearly same as check_all_fr_mindists_to_be_valid
//->make redesign !!!!!
void IVP_Impact_System::impact_system_check_start_pair(IVP_Friction_Core_Pair *start_pair,IVP_Contact_Point *mdfr) {
    
    for (int i = start_pair->fr_dists.len()-1; i>=0;i--){
	IVP_Contact_Point *my_dist = start_pair->fr_dists.element_at(i);
	if( mdfr != my_dist ) { // warning old version checked mdfr !=  next_dist
	    my_dist->recalc_friction_s_vals();
	    IVP_Impact_Solver_Long_Term *info=my_dist->tmp_contact_info;
	    my_dist->read_materials_for_contact_situation(info);
	    //printf("impact_sys_update_contact_vals %lx\n",(long)my_dist);
	    if(info->friction_is_broken==IVP_TRUE) {
	        associated_fs_system->delete_friction_distance(my_dist);
	    }
	}
    }
    
}

// optimize: first (initial) friction mindist is checked although it is known that it is valid

// Impact System:
// An Impact System is oriented to the Friction System. Friction Mindists are checked if 
// they are still valid and then if they may collide in the near future. If they do, the 
// impact is done instantly for speedup. 
//
// There are three lists. One list with interesting Mindists. One list with Cores that are 
// known (means they are synchronized for that point in time) and one list with Cores that 
// have already impacted (they are synchronized and their speed has changed, meaning their 
// Exact_Mindists and their Next_PSI_Matrix have to be updated. More speedup is gained with 
// delaying technique. A core that is pushed only very weak doesnt change its speed. The 
// speed is added in next PSI. Delayed cores need no update in Exact_Mindists and 
// Next_PSI_Matrix. But the synchronize has to be undone. Delayed cores are found in list 
// with known cores, but not necessarily in list with pushed cores. When a push is done in 
// Impact System, the pushed cores (not a delayed one) are taken and their Mindists are 
// rechecked (calc_coll_distance). A Core that was pushed is put into pushed list. A delayed 
// push means that Mindists dont need call to calc_coll_distance. When everything is over, 
// the cores beeing in known list and NOT beeing in pushed list are unsynchronized. All 
// Cores in pushed list are updated in Exact_Mindists and Next_PSI_Matrix.

// #+# these things with start_pair and except_pair are weird and not necessary any longer: 
// remove
void IVP_Impact_System::init_and_solve_impact_system(IVP_Mindist *mindist, IVP_Friction_System *fs_system,IVP_Friction_Core_Pair *start_pair,IVP_Contact_Point *start_fr_dist)
{
    IVP_Core *start_core0=start_fr_dist->get_synapse(0)->get_object()->get_core();
    IVP_Core *start_core1=start_fr_dist->get_synapse(1)->get_object()->get_core();
  
    sum_of_pushes=0;
    associated_fs_system=fs_system;
    l_environment=start_pair->objs[0]->environment;

    IVP_Core *core0,*core1;
    
    if(start_core0 && !start_core0->physical_unmoveable && !start_core0->pinned) { //@@CBPIN
	add_pushed_core_with_pairs_except(start_core0,start_pair);
	//invalidate_impact_mindists(start_core0);
    }

    core0 = start_pair->objs[0];//start_mindist->synapse[0]->get_core();
    if(!core0->physical_unmoveable  && !core0->pinned) { //@@CBPIN
	add_known_core_to_impact_system(core0);
    }
       
    if(start_core1 && !start_core1->physical_unmoveable && !start_core1->pinned) { //@@CBPIN
	add_pushed_core_with_pairs_except(start_core1,start_pair);
	//invalidate_impact_mindists(start_core1);
    }
    
    core1 = start_pair->objs[1];//start_mindist->synapse[1]->get_core();    
    if(!core1->physical_unmoveable && !core1->pinned) { //@@CBPIN
	add_known_core_to_impact_system(core1);
    }

    add_pair_to_impact_system(start_pair);
    impact_system_check_start_pair(start_pair,start_fr_dist);
    
    IVP_IF(1) {
        l_environment->get_debug_manager()->debug_imp_sys=IVP_FALSE;
    }
    
    IVP_IF(l_environment->get_debug_manager()->file_out_impacts) {
        if(l_environment->get_current_time().get_time() > 10000.86f) {
	    printf("critical_imp_sys\n");
	    l_environment->get_debug_manager()->debug_imp_sys=IVP_TRUE;
	}
        fprintf(l_environment->get_debug_manager()->out_deb_file,"starting_impact_system at time %f\n",l_environment->get_current_time().get_time());
    }

    int impact_sys_counter = 0;
    while(test_loop_all_pairs()==IVP_TRUE)    {
	impact_sys_counter ++;
	sum_of_pushes++;
	if(sum_of_pushes > MAXIMUM_SYSTEM_PUSH_NUMBER) {
	    IVP_IF(1) {
		printf("system_push_number exceeded\n");
	    }
	    P_DELETE(mindist);
	    break;
	    // put some mindists into hull!
	}
    }
    l_environment->get_statistic_manager()->impact_sum_sys+= impact_sys_counter+1;

    IVP_IF(sum_of_pushes>1) {
	//printf("did_impact_sys %d pushes at %f\n",sum_of_pushes,l_environment->get_current_time());
#if 0
	for(IVP_Friction_Core_Pair *my_pair=get_first_impact_pair();my_pair;my_pair=get_next_impact_pair())
	{
	    printf(" paair %lx ",(long)my_pair&0x0000ffff);
	    for(IVP_Contact_Point *my_fr=my_pair->get_first_fr_dist_obj_pairs();my_fr;my_fr=my_pair->get_next_fr_dist_obj_pairs())
	    {
		printf("md %lx ",(long)my_fr&0x0000ffff);
	    }
	}
	printf("\n");
#endif
    }
    IVP_IF(1) {
        debug_check_all_dists_at_end();
    }
    
    recalc_all_affected_cores();
    IVP_IF(1) {
	associated_fs_system->debug_clean_tmp_info();
    }
    IVP_IF(l_environment->get_debug_manager()->file_out_impacts) {
	l_environment->get_debug_manager()->debug_imp_sys=IVP_FALSE;	
        fprintf(l_environment->get_debug_manager()->out_deb_file,"\n");
    }
}

void IVP_Impact_System::add_pair_to_impact_system(IVP_Friction_Core_Pair *new_pair)
{
    i_s_pairs.add(new_pair);
#if 0 /* this is done now at recalc_friction_s_vals */   
    for (int i = new_pair->fr_dists.len()-1; i>=0; i--){
	IVP_Contact_Point *my_dist = new_pair->fr_dists.element_at(i);
	my_dist->tmp_contact_info->rescue_factor=1.0f;
	my_dist->tmp_contact_info->impacts_while_system=0;
	my_dist->tmp_contact_info->coll_time_is_valid=IVP_FALSE;
    }
#endif    
}

void IVP_Impact_System::synchronize_core_for_impact_system(IVP_Core *new_core) {
    if (IVP_MTIS_SIMULATED( new_core->movement_state )){
	new_core->synchronize_with_rot_z();
    }
}

void IVP_Impact_System::add_pushed_core_with_pairs_except(IVP_Core *new_core,IVP_Friction_Core_Pair *start_pair)
{
    add_pushed_core_to_impact_system(new_core);
    
    // connection of core to all its pairs is missing -> loop is quite big, but add_core is not too often done
    for ( int i = associated_fs_system->fr_pairs_of_objs.len()-1; i>=0; i--){
	IVP_Friction_Core_Pair *fr_pair = associated_fs_system->fr_pairs_of_objs.element_at(i);
	if((fr_pair->objs[0] == new_core)  ||   (fr_pair->objs[1] == new_core))	{
	    if(fr_pair!=start_pair) {
		if(pair_is_already_in_system(fr_pair)==IVP_FALSE) {
		    if(fr_pair->check_all_fr_mindists_to_be_valid(this->associated_fs_system) > 0) {
			//warning: at this point fr_pair could be deleted
			add_pair_to_impact_system(fr_pair);
		    }
		}
	    }
	}
    }
}



// speedup: put flag in pairs
IVP_BOOL IVP_Impact_System::pair_is_already_in_system(IVP_Friction_Core_Pair *test_pair) {
    for (int i = i_s_pairs.len()-1; i>=0; i--){
	IVP_Friction_Core_Pair *my_pair = i_s_pairs.element_at(i);
	if(my_pair==test_pair) {
	    return IVP_TRUE;
	}
    }
    return IVP_FALSE;
}

void IVP_Friction_System::debug_clean_tmp_info() {
    IVP_Contact_Point *my_dist;
    for(my_dist=this->get_first_friction_dist();my_dist;my_dist=this->get_next_friction_dist(my_dist)) {
	my_dist->tmp_contact_info=NULL;
    }
}

void IVP_Impact_System::debug_check_all_dists_at_end() {
    IVP_Contact_Point *my_dist;
    for(my_dist=associated_fs_system->get_first_friction_dist();my_dist;my_dist=associated_fs_system->get_next_friction_dist(my_dist)) {
	my_dist->recalc_friction_s_vals();
	my_dist->calc_coll_distance();
	my_dist->tmp_contact_info->coll_time_is_valid=IVP_FALSE;
	if(my_dist->tmp_contact_info->impact.distance_reached_in_time < ivp_mindist_settings.minimum_friction_dist) {
	    IVP_Core *core0,*core1;
	    core0=my_dist->get_synapse(0)->get_object()->physical_core;
	    core1=my_dist->get_synapse(1)->get_object()->physical_core;

	    int unmov0,unmov1;
	    unmov0=(int)core0->physical_unmoveable | (int)core0->temporarily_unmovable;
	    unmov1=(int)core1->physical_unmoveable | (int)core1->temporarily_unmovable;
	    if(unmov0&unmov1) {
		continue;
	    }
	    if(core_is_already_known_to_system(core0) && core_is_already_known_to_system(core1)) {
		printf("error_in_impact_system_missed_impact\n");
	    }
	}
    }
}

IVP_BOOL IVP_Impact_System::test_loop_all_pairs()
{
#ifndef SYSTEM_IMPACT_ENABLED
    return IVP_FALSE;
#endif
    
    IVP_Contact_Point *worst_impact_dist=NULL;
    IVP_Friction_Core_Pair *associated_pair = NULL;
    IVP_DOUBLE smallest_distance =  ivp_mindist_settings.minimum_friction_dist;
    IVP_DOUBLE rescue_speed_addon=0.0f;
    IVP_IF(l_environment->get_debug_manager()->file_out_impacts) {
        fprintf(l_environment->get_debug_manager()->out_deb_file,"calc_new_coll_distances\n");
    }
    for (int i = i_s_pairs.len()-1; i >= 0 ; i--){
	IVP_Friction_Core_Pair *my_pair = i_s_pairs.element_at(i);
	int unmov0,unmov1;
	unmov0=(int)my_pair->objs[0]->physical_unmoveable | (int)my_pair->objs[0]->temporarily_unmovable; //@@CBPIN
	unmov1=(int)my_pair->objs[1]->physical_unmoveable | (int)my_pair->objs[1]->temporarily_unmovable; //@@CBPIN
	if(unmov0 & unmov1) {
	    continue;
	}
	for (int k = my_pair->fr_dists.len()-1; k>=0; k--){
	    IVP_Contact_Point *my_fr = my_pair->fr_dists.element_at(k);
	    if(my_fr->tmp_contact_info->coll_time_is_valid==IVP_TRUE) {
	      IVP_IF(l_environment->get_debug_manager()->debug_imp_sys) {
		printf("did_not_test %lx\n",0x0000ffff&(long)my_fr);
	      }
	    } else {
		l_environment->get_statistic_manager()->impact_coll_checks++;
		my_fr->calc_coll_distance();
	    }
	    if(my_fr->tmp_contact_info->impact.distance_reached_in_time< smallest_distance ) {
		rescue_speed_addon=my_fr->tmp_contact_info->impact.rescue_speed_addon;
		smallest_distance=my_fr->tmp_contact_info->impact.distance_reached_in_time;
		worst_impact_dist=my_fr;
		associated_pair=my_pair;
	    }
	}
    }
    IVP_IF(l_environment->get_debug_manager()->file_out_impacts) {
        fprintf(l_environment->get_debug_manager()->out_deb_file,"\n");
    }
    
    if(worst_impact_dist){
	IVP_ASSERT(worst_impact_dist->tmp_contact_info->impact.distance_reached_in_time < ivp_mindist_settings.minimum_friction_dist);
	IVP_Core *core;
	for(int i=1;i>=0;i--) {
	    core=worst_impact_dist->tmp_contact_info->contact_core[i];
	    if(core) {
		if(core_is_already_known_to_system(core)==IVP_FALSE) {
		// Mac port was: if (i_s_known_cores.index_of(core) == -1) {
		    add_known_core_to_impact_system(core);
		    synchronize_core_for_impact_system(core);
		}
	    }
	}
	    
	IVP_Core *impacting_cores[2];
	worst_impact_dist->tmp_contact_info->impacts_while_system++;
       
	worst_impact_dist->tmp_contact_info->do_impact_long_term(impacting_cores,worst_impact_dist->tmp_contact_info->impact.rescue_speed_addon,worst_impact_dist);
	    
	//printf("did_sys_imp %lx\n",(long)(&worst_impact_dist->long_term_impact_info)&0x0000ffff);
	    
	{
	    for(int i=1;i>=0;i--) {
		core = impacting_cores[i];
		if(core && !core->physical_unmoveable && !core->pinned) { //@@CBPIN
		    if(core_was_already_pushed_in_system(core)==IVP_FALSE) {
			add_pushed_core_with_pairs_except(core,associated_pair);
		    } else {
			invalidate_impact_mindists(core);
		    }
		}
	    }
	}
	
	return IVP_TRUE;
    } else {
	return IVP_FALSE;
    }
}

void IVP_Impact_System::invalidate_impact_mindists(IVP_Core *my_core)
{    //there is a cheaper way: via synapses
    IVP_Friction_Info_For_Core *my_info = my_core->get_friction_info(associated_fs_system);
    for (int i = my_info->friction_springs.len()-1; i>=0; i--){
	IVP_Contact_Point *my_dist = my_info->friction_springs.element_at(i);
	my_dist->tmp_contact_info->coll_time_is_valid=IVP_FALSE;
    }
}
//direction of second friction lies orthogonal to surf_normal
//returns TRUE when second friction value is used
// &imp_solver_stack.world_direction_second_friction,&imp_solver_stack.sin_second_friction
void  IVP_Impact_Solver::get_world_direction_second_friction(IVP_Contact_Point *cp) { //IVP_U_Float_Point *ret_second_friction_vec,IVP_FLOAT *sin_second_friction) {
    IVP_Material *mtl[2];
    cp->get_material_info(mtl);

    IVP_U_Float_Point *ret_second_friction_vec=&world_direction_second_friction;
    
    IVP_BOOL use_second_friction=IVP_FALSE;

    int i;
    for(i=0;i<2;i++) {
        if(mtl[i]->second_friction_x_enabled) {
#if 1	    
	      //following is copy-paste from function 'two_values_friction'
	      //-> maybe make common structure and fill it
		IVP_U_Float_Point x_direction;
		IVP_Core *i_core=cp->get_synapse(i)->l_obj->friction_core;
		IVP_U_Matrix *mat=&i_core->m_world_f_core_last_psi;
		mat->get_col(IVP_INDEX_X,&x_direction);
		IVP_U_Float_Point *surf_normal=&cp->tmp_contact_info->surf_normal;

		IVP_U_Float_Point x_in_surface;
		x_in_surface.set_orthogonal_part(&x_direction,surf_normal);
	    
		IVP_DOUBLE relevance_factor=x_in_surface.real_length();
		IVP_DOUBLE second_friction_val=mtl[i]->get_second_friction_factor();
		IVP_DOUBLE other_factor=mtl[1-i]->get_friction_factor();
		IVP_DOUBLE using_friction=other_factor*second_friction_val;
		IVP_DOUBLE difference_in_friction=cp->real_friction_factor - using_friction;
		IVP_FLOAT effective_friction_second=cp->real_friction_factor - difference_in_friction * relevance_factor;
#endif	    

	    if(relevance_factor<P_DOUBLE_EPS) {
	        continue;
	    }
	    use_second_friction=IVP_TRUE;
	    x_in_surface.normize();
	    ret_second_friction_vec->set(&x_in_surface);

	    IVP_FLOAT dummy_cos;
	    get_cos_sin_for_impact(effective_friction_second,(IVP_FLOAT)percent_energy_conservation,&dummy_cos,&sin_second_friction);
	}
    }
    two_friction_values=use_second_friction;
}


IVP_FLOAT IVP_Contact_Point::get_rot_speed_uncertainty() {
  IVP_FLOAT rot_uncertainty=0.0f;
  int i;
  for(i=0;i<2;i++) {
      IVP_Core *my_core=tmp_contact_info->contact_core[i];
      if(my_core) {
          IVP_Synapse_Friction *my_syn=this->get_synapse(i);
          if(my_syn->get_status() != IVP_ST_BALL) {
	      //IVP_DOUBLE rota_way=my_core->rot_speed.real_length() * IVP_COLL_DETECT_MIN_TIME;
	      //if( rota_way > IVP_PI_2) { printf("warning_pi\n"); }

	      //IVP_DOUBLE tan_quad = tan ( rota_way );
	      //tan_quad*=tan_quad;
	          IVP_DOUBLE tan_quad = my_core->rot_speed.quad_length() * IVP_COLL_DETECT_MIN_TIME * IVP_COLL_DETECT_MIN_TIME; //approximate tan quad alpha with alpha*alpha

		  IVP_DOUBLE max_tan_quad = IVP_ROT_MAX_UNCERTAINTY * IVP_ROT_MAX_UNCERTAINTY;
		  if(tan_quad > max_tan_quad) {
		      //printf("clippingrottan %f to %f\n",sqrt(tan_quad),sqrt(max_tan_quad));
		      tan_quad = max_tan_quad;
		  }
#if 0		  
		  IVP_DOUBLE under_root = 1.0f + tan_quad;
		  under_root = sqrtf( under_root );

		  under_root = under_root - 1.0f;
		  IVP_DOUBLE uncertain_way = under_root * my_core->upper_limit_radius; //this->long_term_impact_info.contact_point_cs[i].real_length();
#endif
		  IVP_DOUBLE alpha = IVP_Inline_Math::ivp_sqrtf(tan_quad);
		  IVP_DOUBLE uncertain_way= (1.0f - IVP_Inline_Math::cosd(alpha))*my_core->upper_limit_radius;
		  
		  //test only
		  //IVP_DOUBLE alpha = my_core->rot_speed.real_length() * IVP_COLL_DETECT_MIN_TIME;
		  //IVP_DOUBLE sina = sin(alpha);
		  //IVP_ASSERT(alpha >= sina);
		  //IVP_DOUBLE uncertain_way2 = (alpha - sina) * this->long_term_impact_info.contact_point_cs[i].real_length();

		  IVP_DOUBLE uncertain_speed = uncertain_way * my_core->environment->get_inv_delta_PSI_time(); //later speed is transformed back to way with get_delta_PSI_time

		  //IVP_FLOAT rot_speed_quad=my_core->get_quad_surface_speed_orth_to_normal(&this->long_term_impact_info.contact_point_cs[i],&this->get_lt()->surf_normal);
		  //IVP_FLOAT upper_bound=IVP_ROT_MAX_UNCERTAINTY*IVP_ROT_MAX_UNCERTAINTY*my_core->environment->get_inv_delta_PSI_time()*my_core->environment->get_inv_delta_PSI_time();
		  //if(rot_speed_quad > upper_bound ) {
		  //  rot_speed_quad = upper_bound;
		  //}
		  // rotate surface and take worst case way passed -> get uncertainty speed
		  // in formula tan alpha is approximated through alpha
		  //IVP_FLOAT speed_uncertainty=1.0f+rot_speed_quad;
		  //speed_uncertainty=sqrt( speed_uncertainty ) - 1.0f;

		  //printf("speed_unctnty %f\n",speed_uncertainty);
		  //speed_uncertainty=0.0f;
		  //IVP_FLOAT obj_length_factor=long_term_impact_info.contact_point_cs[i].real_length();
		  //IVP_ASSERT( (obj_length_factor < my_core->upper_limit_radius + P_DOUBLE_EPS) );

		  rot_uncertainty = rot_uncertainty + uncertain_speed;
          }
      }
  }
  return rot_uncertainty;
}

IVP_FLOAT IVP_Contact_Point::get_rescue_speed_impact(IVP_Environment *env) {
    IVP_DOUBLE resc_val=0.0f;
    IVP_DOUBLE penetration_len=get_gap_length();
    if(penetration_len < ivp_mindist_settings.min_coll_dists) {
       resc_val = (ivp_mindist_settings.min_coll_dists - penetration_len) * env->get_inv_delta_PSI_time();
    } else {
	this->tmp_contact_info->impact.rescue_speed_addon = 0.0f;
    }
    IVP_DOUBLE rot_uncertainty=this->get_rot_speed_uncertainty();
    resc_val+=rot_uncertainty;

    IVP_ASSERT( resc_val < 1000.0f);
    IVP_IF(resc_val > env->get_statistic_manager()->max_rescue_speed) {
	env->get_statistic_manager()->max_rescue_speed = resc_val;
    }
    return 2.0f*resc_val; //2.0f: use half factor when testing afterwards to assure convergence
}

void IVP_Contact_Point::calc_coll_distance(){
    IVP_Impact_Solver_Long_Term *info = tmp_contact_info;

    info->coll_time_is_valid = IVP_TRUE;    

    if(get_gap_length() > ivp_mindist_settings.max_distance_for_impact_system) {
	info->impact.distance_reached_in_time=1E20f;
	return;
    }
    IVP_Environment *env = get_synapse(0)->get_object()->get_environment();
    
    info->impact.rescue_speed_addon = this->get_rescue_speed_impact(env);    
    IVP_DOUBLE closing_speed = info->get_closing_speed();
    info->impact.distance_reached_in_time = get_gap_length() - ( closing_speed + info->impact.rescue_speed_addon * 0.5f ) * env->get_delta_PSI_time();
    //rescue speed * 0.5f : at impact demand full rescue_speed; at testing, demand only half (to assure termination)

    IVP_IF(1){
	IVP_IF(env->get_debug_manager()->file_out_impacts) {
	    FILE *fp=env->get_debug_manager()->out_deb_file;
	    IVP_Core *core0,*core1;
	    core0=get_synapse(0)->l_obj->friction_core;
	    core1=get_synapse(1)->l_obj->friction_core;
	    fprintf(fp,"  %lx %lx-%lx: ",0x0000ffff&(long)this,0x0000ffff&(long)core0,0x0000ffff&(long)core1);
	    fprintf(fp,"di %.4f  ",get_gap_length());
	    IVP_DOUBLE debug_cs = closing_speed + info->impact.rescue_speed_addon*0.5f;
	    fprintf(fp,"cs %.4f  ",debug_cs);
	    fprintf(fp,"dr %.4f  ",info->impact.distance_reached_in_time);
	}
	IVP_IF(env->get_debug_manager()->debug_imp_sys) {
	    printf("tested_frdist %lx di %.4f cs %.4f dr %.4f\n",
		0x0000ffff&(long)this,
		get_gap_length(),
		closing_speed + info->impact.rescue_speed_addon*0.5f,
		info->impact.distance_reached_in_time);
	}
    }
};

class IVP_Vector_of_Hull_Managers_256: public IVP_U_Vector<IVP_Hull_Manager_Base> {
    void *elem_buffer[256];
public:
    IVP_Vector_of_Hull_Managers_256(): IVP_U_Vector<IVP_Hull_Manager_Base>( &elem_buffer[0],256 ){;};
};

void IVP_Impact_System::recalc_all_affected_cores()
{
    IVP_IF(1) {
	for (int k = i_s_pushed_cores.len()-1; k>=0; k--){
	    IVP_Core *my_core= i_s_pushed_cores.element_at(k);
	    //every pushed core has to be known
	    IVP_BOOL found=IVP_FALSE;
	    for (int j = i_s_known_cores.len()-1; j>=0; j--){
		IVP_Core *test_core= i_s_known_cores.element_at(j);
		if(my_core==test_core) {
		    found=IVP_TRUE;
		}
	    }
	    IVP_ASSERT(found==IVP_TRUE);
	    IVP_ASSERT(my_core->tmp_null.old_sync_info->was_pushed_during_i_s==IVP_TRUE);
	}
    }
    {
	for (int j = i_s_known_cores.len()-1; j>=0; j--){
	    IVP_Core *my_core2 = i_s_known_cores.element_at(j);
	    if(core_was_already_pushed_in_system(my_core2)==IVP_FALSE) {
		//this core has impacted, but was not pushed (delayed)
		my_core2->undo_synchronize_rot_z();
	    }
	    my_core2->tmp_null.old_sync_info=NULL; //clear
	}
    }
    {
	IVP_DOUBLE delta_time_till_next_PSI = l_environment->get_next_PSI_time() - l_environment->get_current_time();
	IVP_Vector_of_Hull_Managers_256 active_hull_managers;
	
	for (int k = i_s_pushed_cores.len()-1; k>=0; k--){
	    IVP_Core *my_core= i_s_pushed_cores.element_at(k);
	    IVP_ASSERT(!my_core->physical_unmoveable); //core merge?
	    if(!my_core->physical_unmoveable) {
		//printf("calcing_next_psi_matrix %lx %f\n",(long)my_core&0x0000ffff,l_environment->get_current_time());
	        //my_core->commit_all_async_pushes();
		IVP_Event_Sim es(l_environment, delta_time_till_next_PSI);
		IVP_Calc_Next_PSI_Solver nps(my_core);
		nps.calc_next_PSI_matrix(&es, &active_hull_managers);
		my_core->tmp_null.old_sync_info=NULL;
		invalidate_impact_mindists(my_core);
	    }
	}
	IVP_Calc_Next_PSI_Solver::commit_all_hull_managers( l_environment,  &active_hull_managers);
    }
    {
	for (int k = i_s_pushed_cores.len()-1; k>=0; k--){
	    IVP_Core *my_core= i_s_pushed_cores.element_at(k);
	    if(!my_core->physical_unmoveable) {
		//printf("calcing_exactmindists %lx\n",(long)my_core&0x0000ffff);
		my_core->update_exact_mindist_events_of_core();
	    }
	}
    }
}






