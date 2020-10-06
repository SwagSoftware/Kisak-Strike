// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#include <ivp_physics.hxx>

#if defined(LINUX) || defined(SUN) || (__MWERKS__ && __POWERPC__)
#	include <alloca.h>
#endif

// IVP_EXPORT_PRIVATE


#include <ivu_memory.hxx>
#include <ivp_great_matrix.hxx>
#include <ivp_sim_unit.hxx>
#include <ivp_physic_private.hxx>
#include <ivp_core_macros.hxx>
#include <ivp_material.hxx>
#include <ivp_impact.hxx>

#include <ivp_mindist_intern.hxx> // because of Mindist

#include <ivp_friction.hxx>
#include <ivp_friction_solver.hxx>
#include <ivp_debug_manager.hxx> // bvecause of debug psi_synchrone

#include <ivp_compact_ledge.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <ivp_listener_collision.hxx>

#include <ivp_hull_manager.hxx> //because of Mindist.
#include <ivp_mindist_macros.hxx> //because of Mindist
#include <ivp_solver_core_reaction.hxx>
#include <ivp_constraint_car.hxx>

void IVP_Synapse_Friction::remove_friction_synapse_from_object(){
    if (next) next->prev = prev;
    if (prev){
	prev->next = next;
    }else{
	l_obj->friction_synapses = get_next();
    }
}

int IVP_Friction_System::friction_global_counter=0;



void IVP_Synapse_Friction::init_synapse_friction( IVP_Contact_Point *friction,
						  IVP_Real_Object *object,
						  const IVP_Compact_Edge 	*edge_in,
						  IVP_SYNAPSE_POLYGON_STATUS status_in) {
    l_obj = object;
    set_contact_point(friction);
	next = object->friction_synapses;
	prev = NULL;
	if (next) next->prev = this;
	object->friction_synapses = this;

	status = status_in;
	edge = edge_in;
}




IVP_Contact_Point::IVP_Contact_Point( IVP_Mindist *md)
{
    IVP_Synapse_Real *syn0 = md->get_sorted_synapse(0);
    synapse[0].init_synapse_friction(this, syn0->get_object(), syn0->edge, syn0->get_status());
#ifdef IVP_USE_S_VALS_FOR_PRETENSION    
    s_coords[0] = s_coords[1] = 0.0f;
#endif
    IVP_Synapse_Real *syn1 = md->get_sorted_synapse(1);
    synapse[1].init_synapse_friction(this, syn1->get_object(), syn1->edge, syn1->get_status());

    IVP_Environment *env = md->get_environment();
    this->last_time_of_recalc_friction_s_vals = env->get_current_time();

    //this->l_environment = mindist->l_environment;
    IVP_IF(1) {
	IVP_IF(env->get_debug_manager()->check_fs) {
	    fprintf(env->get_debug_manager()->out_deb_file,"create_mindist %f %lx cores %lx %lx\n",
		    env->get_current_time().get_time(),
		    (long)this,
		    (long)syn0->l_obj->physical_core,
		    (long)syn0->l_obj->physical_core);
	}
    }
    if ( syn1->get_status() == IVP_ST_TRIANGLE){
	const IVP_Compact_Edge *e1 = syn1->edge;
	IVP_U_Point wHesse_vecF_Fos;
	IVP_CLS.calc_hesse_vec_object_not_normized(e1, e1->get_compact_ledge(), &wHesse_vecF_Fos);
	inv_triangle_det = 1.0f / wHesse_vecF_Fos.real_length();
    }

    this->old_energy_dynamic_fr=0.0f;
    this->now_friction_pressure=0.0f;
    this->integrated_destroyed_energy = 0.0f;
    this->cp_status = IVP_CPBS_NEEDS_RECHECK;
    //printf("initing_mindist_fr %lx\n",(long)this);

    this->span_friction_s[0]=0.0f;
    this->span_friction_s[1]=0.0f;

    this->last_gap_len=ivp_mindist_settings.friction_dist; //at first PSI-iteration there is no energy from complex //maybe drop it
    has_negative_pull_since=0;
    //force_push_complex_last_psi=0.0f; //evtl. nicht noetig    
    slowly_turn_on_keeper = IVP_SLOWLY_TURN_ON_KEEPER;
    l_friction_system=NULL;
    two_friction_values=IVP_FALSE;
    
}


// everything in world coords
IVP_DOUBLE IVP_Contact_Point::two_values_friction(IVP_U_Float_Point *real_world_friction_vec) {
    IVP_Material *mtl[2];
    get_material_info(mtl);
    IVP_Impact_Solver_Long_Term *info = tmp_contact_info;

    int i;
    for(i=0;i<2;i++) {
        if(mtl[i]->second_friction_x_enabled) {
	    IVP_U_Float_Point x_direction;
	    IVP_Core *core=get_synapse(i)->l_obj->friction_core;
	    IVP_U_Matrix *mat=&core->m_world_f_core_last_psi;
	    mat->get_col(IVP_INDEX_X,&x_direction);
	    IVP_U_Float_Point *surf_normal=&info->surf_normal;

	    IVP_IF(1) {
		IVP_U_Point base_p;
		base_p.set(&mat->vv);
		IVP_U_Float_Point vec_p;
		vec_p.set(&x_direction);
		char *out_text=p_make_string("x_vec");
		core->environment->add_draw_vector(&base_p,&vec_p,out_text,1);
		P_FREE(out_text);
		vec_p.set(0.0f,-1.0f,0.0f);
		core->environment->add_draw_vector(&base_p,&vec_p,"grav",3);
	    }
	    
	    IVP_U_Float_Point x_in_surface;
	    x_in_surface.set_orthogonal_part(&x_direction,surf_normal);
	    
	    IVP_DOUBLE relevance_factor=x_in_surface.real_length();
	    IVP_DOUBLE second_friction_val=mtl[i]->get_second_friction_factor();
	    IVP_DOUBLE other_factor=mtl[1-i]->get_friction_factor();
	    IVP_DOUBLE using_friction=other_factor*second_friction_val;
	    IVP_DOUBLE difference_in_friction=real_friction_factor - using_friction;
	    IVP_DOUBLE effective_friction=real_friction_factor - difference_in_friction * relevance_factor;

	    if(real_friction_factor<P_DOUBLE_EPS) {
	        return 0.0f;
	    }
	    IVP_DOUBLE max_len_in_x_direction = IVP_MINIMAL_REAL_FRICTION_LEN * effective_friction/real_friction_factor;
	    IVP_U_Float_Point friction_vec;

	    IVP_Impact_Solver_Long_Term *info_this = this->tmp_contact_info;
	    friction_vec.set_multiple(&info_this->span_friction_v[0],this->span_friction_s[0]);
	    friction_vec.add_multiple(&info_this->span_friction_v[1],this->span_friction_s[1]);

	    IVP_IF(1) {
		IVP_U_Point base_p;
		base_p.set(&tmp_contact_info->contact_point_ws);
		IVP_U_Float_Point vec_p;
		vec_p.set(&friction_vec);
	        vec_p.mult(10.0f);
		char *out_text=p_make_string("f_vec");
		core->environment->add_draw_vector(&base_p,&vec_p,out_text,1);
		P_FREE(out_text);
	    }
	    
	    x_in_surface.normize();
	    IVP_U_Float_Point x_in_surface_float;
	    x_in_surface_float.set(&x_in_surface);
	    IVP_DOUBLE len_direction_x=friction_vec.dot_product(&x_in_surface_float);
	    IVP_DOUBLE abs_len=IVP_Inline_Math::fabsd(len_direction_x);

	    if((abs_len>max_len_in_x_direction)&&1) {
	        //we have to shorten the friction
		
	        IVP_U_Float_Point vec_full_friction;
		vec_full_friction.set(&friction_vec);
		vec_full_friction.add_multiple(&x_in_surface,-len_direction_x);
		IVP_U_Float_Point vec_second_dir;
		vec_second_dir.set_multiple(&x_in_surface,max_len_in_x_direction);
		if(len_direction_x<0.0f) {
		    vec_second_dir.mult(-1.0f);
		}
		IVP_U_Float_Point resulting_vec;
		resulting_vec.set(&vec_full_friction);
		resulting_vec.add(&vec_second_dir);
		*real_world_friction_vec=resulting_vec;
		this->span_friction_s[0]=resulting_vec.dot_product(&info->span_friction_v[0]);
		this->span_friction_s[1]=resulting_vec.dot_product(&info->span_friction_v[1]);
	    }
	}
    }
    
    IVP_U_Float_Point friction_vec;

    friction_vec.set_multiple(&info->span_friction_v[0],this->span_friction_s[0]);
    friction_vec.add_multiple(&info->span_friction_v[1],this->span_friction_s[1]);
    real_world_friction_vec->set(&friction_vec);
    return real_world_friction_vec->real_length_plus_normize();
}

// calculates friction pointer, pointing away from first obj, also returns length of this pointer and afterwards does normization
// #+# kill . TL: ??
IVP_DOUBLE IVP_Contact_Point::get_and_set_real_friction_len(IVP_U_Float_Point *real_world_friction_vec) {
    IVP_Impact_Solver_Long_Term *info = this->tmp_contact_info;
    real_world_friction_vec->set_multiple(&info->span_friction_v[0],this->span_friction_s[0]);
    real_world_friction_vec->add_multiple(&info->span_friction_v[1],this->span_friction_s[1]);
    
    IVP_DOUBLE len_of_spring = real_world_friction_vec->real_length_plus_normize();

    if(this->two_friction_values==IVP_TRUE) {
	return two_values_friction(real_world_friction_vec);
    }
    
    return len_of_spring;
}


void IVP_Contact_Point::static_friction_single(const IVP_Event_Sim *es,IVP_FLOAT desired_gap,IVP_FLOAT speedup_factor) {
    IVP_Impact_Solver_Long_Term *info = this->tmp_contact_info;
    IVP_DOUBLE closing_speed = info->get_closing_speed();
    IVP_DOUBLE gap_diff = desired_gap - this->get_gap_length();
    IVP_DOUBLE a = IVP_Friction_Solver::calc_desired_gap_speed(closing_speed, gap_diff,speedup_factor);
    IVP_DOUBLE b = info->virtual_mass;
    IVP_DOUBLE impulse = a * b;

    if(impulse > 0.0f) {
	this->now_friction_pressure = impulse * es->i_delta_time;    
	IVP_Friction_Solver::apply_impulse( info, impulse );
    } else {
	this->now_friction_pressure=0.0f;
    }
}

bool IVP_Contact_Point::friction_force_local_constraint_2d_wheel( IVP_Core *core_a, IVP_Impact_Solver_Long_Term *info,
																  const IVP_Event_Sim *es, IVP_FLOAT &flEnergy )
{
	IVP_Constraint_Car_Object *wheel = core_a->car_wheel;
	IVP_Constraint_Solver_Car *solver = wheel->solver_car;

    IVP_DOUBLE maximum_impulse_force = this->now_friction_pressure * this->real_friction_factor * es->delta_time;

	IVP_U_Float_Point axis_bs; 
	wheel->target_position_bs.get_col( IVP_COORDINATE_INDEX( solver->x_idx ), &axis_bs );
	IVP_U_Float_Point axis_ws;
	const IVP_U_Matrix *m_world_f_core = solver->body_object->get_core()->get_m_world_f_core_PSI();
	m_world_f_core->vmult3( &axis_bs, &axis_ws );

	// search new span system for which one axis.dot(axis_ws) == 0
	IVP_U_Float_Point span_v_0; // span which is orthogonal to any car body influences
	IVP_U_Float_Point span_v_1; // span which is influences by car body weight

	span_v_0.calc_cross_product( &axis_ws, &info->surf_normal);
	if ( span_v_0.quad_length() < 0.001f )
		return false;

	span_v_0.normize();
	span_v_1.calc_cross_product( &span_v_0, &info->surf_normal );

	IVP_DOUBLE dot_old0_new0 = info->span_friction_v[0].dot_product( &span_v_0 );
	IVP_DOUBLE dot_old0_new1 = info->span_friction_v[0].dot_product( &span_v_1 );
	IVP_DOUBLE dot_old1_new0 = info->span_friction_v[1].dot_product( &span_v_0 );
	IVP_DOUBLE dot_old1_new1 = info->span_friction_v[1].dot_product( &span_v_1 );

	IVP_DOUBLE span_s_0 = span_friction_s[0] * dot_old0_new0 + span_friction_s[1] * dot_old1_new0;
	IVP_DOUBLE span_s_1 = span_friction_s[0] * dot_old0_new1 + span_friction_s[1] * dot_old1_new1;
	
	// calculate pushing behaviour between wheel and car in wheel axis direction
	IVP_DOUBLE p_wheel;
	IVP_DOUBLE p_body;
	
	IVP_DOUBLE wheel_vel;
	IVP_DOUBLE body_vel;

    IVP_Solver_Core_Reaction tcb;
	tcb.init_reaction_solver_translation_ws( core_a, NULL, info->contact_point_ws, &span_v_0, &span_v_1, NULL );
	{
		p_wheel = tcb.m_velocity_ds_f_impulse_ds.get_elem( 1, 1 );
		wheel_vel = tcb.delta_velocity_ds.k[1];
	}

	{
		IVP_Solver_Core_Reaction sc_body;
		sc_body.init_reaction_solver_translation_ws( solver->body_object->get_core(), NULL, info->contact_point_ws, &span_v_1, NULL, NULL );
		p_body = sc_body.m_velocity_ds_f_impulse_ds.get_elem( 0, 0 );
		body_vel = sc_body.delta_velocity_ds.k[0];
	}
	
	IVP_DOUBLE a = span_s_0 * es->i_delta_time - tcb.delta_velocity_ds.k[0];
	//IVP_DOUBLE b = span_s_1 * es->i_delta_time - tcb.delta_velocity_ds.k[1];
	IVP_DOUBLE b = span_s_1 * es->i_delta_time - 1.0f * ( wheel_vel * 1.0f + body_vel * 0.0f);
		
	IVP_U_Matrix3 &tpm = tcb.m_velocity_ds_f_impulse_ds;
	
	IVP_DOUBLE inv_mat2x2[4];
	
	const IVP_DOUBLE tpm00 = tpm.get_elem( 0, 0 );
	const IVP_DOUBLE tpm01 = tpm.get_elem( 0, 1 );
	const IVP_DOUBLE tpm10 = tpm.get_elem( 1, 0 );
	const IVP_DOUBLE tpm11 = tpm.get_elem( 1, 1 );
	
	IVP_RETURN_TYPE ret = IVP_Inline_Math::invert_2x2_matrix( tpm00, tpm01, tpm01, tpm11, &inv_mat2x2[0], &inv_mat2x2[1],&inv_mat2x2[2],&inv_mat2x2[3] );
	if ( ret != IVP_OK ) 
	{
		flEnergy = 0.0f;
		return true;
	}

    IVP_U_Float_Point impulses;
	impulses.k[0] = inv_mat2x2[0] * a + inv_mat2x2[1] * b;
	impulses.k[1] = inv_mat2x2[2] * a + inv_mat2x2[3] * b;
	
	IVP_DOUBLE project_span_v1 = IVP_Inline_Math::fabsd( span_v_1.dot_product( &axis_ws ) );

	// 
	IVP_DOUBLE relaxation_coefficient = 0.3f;  // ~1.0f tends to oscillate
	IVP_DOUBLE body_impulse_factor =  ( project_span_v1 * p_wheel/ p_body + 1.0f ) * relaxation_coefficient; 
	
	IVP_DOUBLE imp0_sqrd = impulses.k[0] * impulses.k[0];
	IVP_DOUBLE imp1_sqrd = impulses.k[1] * impulses.k[1];

	// check for sliding with body push
	core_a->car_wheel->last_contact_position_ws.set( &info->contact_point_ws );
	core_a->car_wheel->last_skid_time = es->environment->get_current_time();

	// handbrake !!!
    IVP_DOUBLE square_impulse;
	if ( core_a->car_wheel->fix_wheel_constraint )
	{
		square_impulse = ( imp0_sqrd + imp1_sqrd ) * body_impulse_factor * body_impulse_factor;
		
		if ( square_impulse > maximum_impulse_force * maximum_impulse_force ) // test for sliding
		{
			// sliding handbrake
			IVP_DOUBLE isum_impulse = IVP_Inline_Math::isqrt_float( square_impulse ); // body push
			IVP_DOUBLE f = maximum_impulse_force * isum_impulse * body_impulse_factor;
			impulses.k[0] *= f;
			impulses.k[1] *= f;
			
			f /= body_impulse_factor;
			
			span_friction_s[0] *= f; // update reference point
			span_friction_s[1] *= f;
			
			core_a->car_wheel->last_skid_value = ( 1.0f - f ) * this->now_friction_pressure * solver->body_object->get_core()->get_inv_mass();
			//ivp_message( "handbrake factor %f\n", f);
		}
		else
		{
			// still handbrake
			IVP_DOUBLE impulse_factor = .5f;// reduce real values to avoid jitter effects
			impulses.k[0] *= body_impulse_factor * impulse_factor; 
			impulses.k[1] *= body_impulse_factor * impulse_factor;
			core_a->car_wheel->last_skid_value = 0.0f;
		}
	}
	else
	{
		square_impulse = imp0_sqrd + imp1_sqrd * body_impulse_factor * body_impulse_factor;
		if ( 1 && ( square_impulse > maximum_impulse_force * maximum_impulse_force ) ) 
		{
			// check for sliding with no body push
			IVP_DOUBLE square2 = imp0_sqrd + imp1_sqrd;
			if ( 1 && ( square2 > maximum_impulse_force*maximum_impulse_force ) )
			{
				// clip impulse on a circle
				IVP_DOUBLE isum_impulse = IVP_Inline_Math::isqrt_float( square2 ); // no body push
				IVP_DOUBLE f = maximum_impulse_force * isum_impulse;
				impulses.k[0] *= f;
				impulses.k[1] *= f;
				
				span_friction_s[0] *= f; // update reference point
				span_friction_s[1] *= f;
				core_a->car_wheel->last_skid_value = ( 1.0f - f ) * this->now_friction_pressure * solver->body_object->get_core()->get_inv_mass();
				//ivp_message(" sliding %f factor %f\n", impulses.k[1], f );
			}
			else
			{
				// reduce body push until no sliding (clipping impulse on a elipse)
				IVP_DOUBLE y = IVP_Inline_Math::sqrtd( maximum_impulse_force * maximum_impulse_force - imp0_sqrd );
				IVP_DOUBLE old1 = impulses.k[1];
				if ( impulses.k[1] > 0)
				{
					impulses.k[1] = y;
				}
				else
				{
					impulses.k[1] = -y;
				}
				
				if ( 1 ) // update reference point
				{
					//span_s_1 *= 0.95f;
					static IVP_DOUBLE extra_factor = .9f;
					IVP_DOUBLE s_factor = extra_factor * impulses.k[1] / ( old1 * body_impulse_factor );
					core_a->car_wheel->last_skid_value = 0.3f * ( 1.0f - s_factor ) * this->now_friction_pressure * solver->body_object->get_core()->get_inv_mass();
					if ( s_factor < 1.0f )
					{
						span_s_1 *= s_factor;
						IVP_DOUBLE s0 = span_s_0 * dot_old0_new0 + span_s_1 * dot_old0_new1;
						IVP_DOUBLE s1 = span_s_0 * dot_old1_new0 + span_s_1 * dot_old1_new1;
						span_friction_s[0] = s0;
						span_friction_s[1] = s1;
					}
					//ivp_message(" reduced body push %f  s_factor %f\n", impulses.k[1], s_factor );
				}
			}

			square_impulse = maximum_impulse_force * maximum_impulse_force;
		}
		else // no sliding
		{
			//impulses.k[1] *= body_impulse_factor;
			//ivp_message("normal %f\n", impulses.k[1] );
		}
	}

	tcb.exert_impulse_dim2( core_a, /*core_b*/NULL, impulses );

	flEnergy = 0.0f; // current_energy
	return true;
}

IVP_FLOAT IVP_Contact_Point::friction_force_local_constraint_2d(const IVP_Event_Sim *es) 
{
    IVP_Impact_Solver_Long_Term *info = this->tmp_contact_info;
    IVP_DOUBLE maximum_impulse_force = this->now_friction_pressure * this->real_friction_factor * es->delta_time;
    if ( maximum_impulse_force < P_FLOAT_RES )
		return 0.0f;
    
    IVP_Core *core_a = info->contact_core[0];
    IVP_Core *core_b = info->contact_core[1];	
	
    if ( 1 && core_a && core_a->car_wheel && !core_b && ( next_dist_in_friction == NULL || next_dist_in_friction->now_friction_pressure == 0.0f ) )
	{
		IVP_FLOAT flEnergy = 0.0f;
		if ( friction_force_local_constraint_2d_wheel( core_a, info, es, flEnergy ) )
			return flEnergy;		
    }

    IVP_Solver_Core_Reaction tcb;
	tcb.init_reaction_solver_translation_ws( core_a, core_b, info->contact_point_ws, &info->span_friction_v[0], &info->span_friction_v[1], NULL );
	
	IVP_DOUBLE a = span_friction_s[0] * es->i_delta_time - tcb.delta_velocity_ds.k[0];
	IVP_DOUBLE b = span_friction_s[1] * es->i_delta_time - tcb.delta_velocity_ds.k[1];
	
	IVP_U_Matrix3 &tpm = tcb.m_velocity_ds_f_impulse_ds;
	
	IVP_DOUBLE inv_mat2x2[4];
	
	const IVP_DOUBLE tpm00 = tpm.get_elem( 0, 0 );
	const IVP_DOUBLE tpm01 = tpm.get_elem( 0, 1 );
	const IVP_DOUBLE tpm10 = tpm.get_elem( 1, 0 );
	const IVP_DOUBLE tpm11 = tpm.get_elem( 1, 1 );
	
	IVP_RETURN_TYPE ret = IVP_Inline_Math::invert_2x2_matrix( tpm00, tpm01, tpm01, tpm11, &inv_mat2x2[0], &inv_mat2x2[1],&inv_mat2x2[2],&inv_mat2x2[3] );
	if( ret != IVP_OK ) 
		return 0.0f;
	
    IVP_U_Float_Point impulses;
	impulses.k[0] = inv_mat2x2[0] * a + inv_mat2x2[1] * b;
	impulses.k[1] = inv_mat2x2[2] * a + inv_mat2x2[3] * b;
	
    IVP_DOUBLE square_impulse;
	square_impulse = impulses.k[0] * impulses.k[0] + impulses.k[1] * impulses.k[1];
	
	if( square_impulse > maximum_impulse_force*maximum_impulse_force ) 
	{
		IVP_DOUBLE isum_impulse = IVP_Inline_Math::isqrt_float(square_impulse);
		IVP_DOUBLE f = maximum_impulse_force * isum_impulse;
		impulses.k[0] *= f;
		impulses.k[1] *= f;
	}

	tcb.exert_impulse_dim2( core_a, core_b, impulses );

	// calc  current energy:
	IVP_FLOAT e = square_impulse * es->delta_time * es->delta_time;
	e *= ( span_friction_s[0] * span_friction_s[0] + span_friction_s[1] * span_friction_s[1] );
	e = 0.5f * IVP_Inline_Math::ivp_sqrtf( e );  // #+# sqrt ???
	IVP_FLOAT diff_e= e - this->old_energy_dynamic_fr;
	this->old_energy_dynamic_fr=e;
	return diff_e;
}

void IVP_Contact_Point::friction_force_local_constraint_1d(const IVP_Event_Sim *es) 
{
    IVP_U_Float_Point world_offset_contact;
    IVP_DOUBLE spring_len = get_and_set_real_friction_len(&world_offset_contact);

    IVP_Core *core[2];
    core[0]=this->get_synapse(0)->get_object()->friction_core;
    core[1]=this->get_synapse(1)->get_object()->friction_core;
    
    IVP_IF(0) {
      IVP_U_Float_Point debug_way_world;
      debug_way_world.set_multiple(&world_offset_contact,spring_len);
      //printf("lenofspring %f\n",spring_len);
      IVP_U_Point base_p;
      base_p.set(&tmp_contact_info->contact_point_ws);
      IVP_U_Float_Point vec_p;
      vec_p.set(&world_offset_contact);
      vec_p.mult(spring_len*10000.0f);
      char *out_text=p_make_string("f_vec");
      core[0]->environment->add_draw_vector(&base_p,&vec_p,out_text,3);
      P_FREE(out_text);
    }
    
    IVP_DOUBLE maximum_impulse_force = this->now_friction_pressure * this->real_friction_factor *
	es->delta_time;
    
    int i;
    IVP_U_Float_Point world_speed[2];
    for(i=0;i<2;i++) {
	IVP_Core *c = core[i];
	if(!c->physical_unmoveable){
	    c->get_surface_speed(&tmp_contact_info->contact_point_cs[i],&world_speed[i]);
	} else {
	    world_speed[i].set_to_zero();
	}
    }

    IVP_U_Float_Point rel_world_speed;
    rel_world_speed.subtract(&world_speed[0],&world_speed[1]);

    IVP_DOUBLE now_speed_direction=world_offset_contact.dot_product(&rel_world_speed);
    
    IVP_U_Float_Point translation_vec[2],rotation_vec[2],push_core_vec[2];
    IVP_DOUBLE speed_change_on_push=0.0f;
    IVP_DOUBLE sign=1.0f;
    
    for(i=0;i<2;i++) {
	IVP_Core *c = core[i];

	translation_vec[i].set_to_zero();
	rotation_vec[i].set_to_zero();
	
	if(!c->physical_unmoveable) {
	    IVP_U_Matrix *mat=&c->m_world_f_core_last_psi;
	   
	    IVP_U_Float_Point push_world_vec;
	    push_world_vec.set(&world_offset_contact);
	    push_world_vec.mult(sign);
	    mat->vimult3(&push_world_vec,&push_core_vec[i]);
	    IVP_U_Float_Point test_trans,test_rot;
	    c->test_push_core(&tmp_contact_info->contact_point_cs[i],&push_core_vec[i],&push_world_vec,&test_trans,&test_rot);
	    translation_vec[i].set(&test_trans);
	    rotation_vec[i].set(&test_rot);
	    IVP_U_Float_Point test_world;
	    c->get_surface_speed_on_test(&tmp_contact_info->contact_point_cs[i],&test_trans,&test_rot,&test_world);
	    speed_change_on_push+=test_world.dot_product(&push_world_vec);
	}	
	sign *= -1.0f;
    }

    IVP_DOUBLE desired_speed = spring_len * es->delta_time;
    IVP_DOUBLE diff_speed = desired_speed-now_speed_direction;

    IVP_DOUBLE push_val;
    if(speed_change_on_push>P_DOUBLE_EPS) {
	push_val=diff_speed/speed_change_on_push;
    } else {
	return;
    }
	
    if(IVP_Inline_Math::fabsd(push_val) > maximum_impulse_force) {
	push_val=maximum_impulse_force;
    }
					       
    for(i=0;i<2;i++) {
	IVP_Core *c = core[i];
	
	if(!c->physical_unmoveable) {
	    c->speed.add_multiple(&translation_vec[i],push_val);
	    c->rot_speed.add_multiple(&rotation_vec[i],push_val);
	}
    }

}


// get maximum (un)sharpness
IVP_FLOAT IVP_Contact_Point::get_possible_friction_slide_way(const IVP_Event_Sim *) {
    IVP_FLOAT way_passed_through_pressure;
    way_passed_through_pressure = now_friction_pressure * real_friction_factor * inv_virt_mass_mindist_no_dir;
    return way_passed_through_pressure;
}

IVP_FLOAT IVP_Friction_Core_Pair::get_sum_slide_way(const IVP_Event_Sim *es) {
    IVP_FLOAT sum=0.0f;
    for (int i = fr_dists.len()-1; i>=0; i--){
	IVP_Contact_Point *my_fr = fr_dists.element_at(i);
        sum += my_fr->get_possible_friction_slide_way(es);
    }
    return sum * es->delta_time * es->delta_time;
}


void IVP_Contact_Point::calc_pretension(IVP_FLOAT len)	{ // clip pretension
  //this->cp_status = IVP_CPBS_NEEDS_RECHECK;    
  IVP_DOUBLE qlen_of_spring = span_friction_s[0] * span_friction_s[0] + span_friction_s[1] * span_friction_s[1];
  if( qlen_of_spring > len * len + P_FLOAT_RES )    {
    IVP_DOUBLE ilen_of_spring = IVP_Inline_Math::isqrt_float(qlen_of_spring);
    this->integrated_destroyed_energy += (qlen_of_spring * ilen_of_spring - len) * real_friction_factor * now_friction_pressure;
    IVP_DOUBLE percent_of_old = len * ilen_of_spring;
    span_friction_s[0] *= percent_of_old;
    span_friction_s[1] *= percent_of_old;
    this->cp_status = IVP_CPBS_NEEDS_RECHECK;    
  }
}

// some comments about two friction values:
//   At the moment the direction of the friction spring is adjusted in a way it is
//   shortened in direction of the lower second value.
//   For the old friction_force_local_constraint this has not the desired effect.
//   The contact points are pushed in a 2D manner (friction_span_vectors to span 2D)
//   to reach the desired positions. This means more force in direction of the lower
//   friction is applied.
//   Therefore now the two friction case is handled with a 1D local constraint.
//   That means that no unallowed force toward second friction direction is applied
//   However, the 1D local constraint is inaccurate and looses contact over the time :-(
//   The best solution would be to make a 2D local constraint for the two value case
//   and span the 2D with the vector of second direction and an orthogonal one. Then
//   we have to clip the force done on the second direction.

void IVP_Friction_Core_Pair::pair_calc_friction_forces(const IVP_Event_Sim *es) {
    IVP_FLOAT max_local_spring_len = get_sum_slide_way(es);

    IVP_FLOAT sum_delta_energy=0.0f;
    
    for (int k = fr_dists.len()-1; k>=0; k--){
	IVP_Contact_Point *my_fr = fr_dists.element_at(k);
	my_fr->calc_pretension( max_local_spring_len );
	
	IVP_ASSERT(my_fr->tmp_contact_info->coll_time_is_valid == IVP_FALSE);
	if(my_fr->two_friction_values!=IVP_TRUE) {
	    sum_delta_energy += my_fr->friction_force_local_constraint_2d(es);
	} else {
	    my_fr->friction_force_local_constraint_1d(es);
	}
    }

    if((sum_delta_energy>0.0f)) {
	integrated_anti_energy+=sum_delta_energy;
    }
}

void IVP_Friction_System::calc_friction_forces(const IVP_Event_Sim *es) {
    for (int i = fr_pairs_of_objs.len()-1; i>=0; i--){
	IVP_Friction_Core_Pair *my_pair = fr_pairs_of_objs.element_at(i);
    //printf("having_core_pairs  ");
	IVP_IF(my_pair->fr_dists.len() == 0) {
	    printf("\n\n\n warngin: empty core pair\n");
	}
	my_pair->pair_calc_friction_forces(es);
    }
    //printf("\n");
}

void IVP_Contact_Point::set_friction_to_neutral(){
    span_friction_s[0] = 0.0f;
    span_friction_s[1] = 0.0f;    
}

void IVP_Friction_Info_For_Core::set_all_dists_of_obj_neutral()
{
    for (int i = friction_springs.len()-1; i>=0; i--){
	IVP_Contact_Point *minfr = friction_springs.element_at(i);
	minfr->set_friction_to_neutral();
    }
}


void IVP_Contact_Point::ease_the_friction_force(IVP_U_Float_Point *ease_diff_vec){
    IVP_Impact_Solver_Long_Term *info = tmp_contact_info;
    this->span_friction_s[0] += ease_diff_vec->dot_product(&info->span_friction_v[0]);
    this->span_friction_s[1] += ease_diff_vec->dot_product(&info->span_friction_v[1]);
    return;
}

void IVP_Friction_Solver::ease_test_two_mindists(IVP_Contact_Point *dist0,IVP_Contact_Point *dist1,IVP_U_Float_Point *world_surf_normal)
{
    IVP_USE(dist1);
    IVP_USE(dist0);
    IVP_USE(world_surf_normal);
#ifdef NOEASING
    return;
#endif
#if 0   
    IVP_Core *rev_core; 
    rev_core=dist0->synapse[0]->l_obj->to_real()->physical_core; //forces seen relative to this core
    
    IVP_real_friction_data fr_data0,fr_data1;
    dist0->get_world_friction_forces_dist(&fr_data0,0);

    dist1->get_world_friction_forces_dist(&fr_data1,0);
    if(rev_core!=dist1->synapse[0]->l_obj->to_real()->physical_core) {
	fr_data1.revert_friction_data();
    }
    
    IVP_U_Float_Point force0_vec,force1_vec;
    force0_vec.subtract(&fr_data0.world0,&fr_data1.world0);
    force1_vec.subtract(&fr_data0.world1,&fr_data1.world1);
    
    IVP_DOUBLE lendiff=IVP_Inline_Math::fabsd(force0_vec.real_length()-force1_vec.real_length());
    IVP_IF(1) {

	IVP_Synapse_Polygon *syn0,*syn1;
	syn0=dist0->synapse[0]->to_poly();
	syn1=dist1->synapse[0]->to_poly();
	IVP_SYNAPSE_POLYGON_STATUS ty0,ty1;
	ty0=syn0->status;
	ty1=syn1->status;
	IVP_Core *core00,*core01,*core10,*core11;
	core00=dist0->synapse[0]->l_obj->to_real()->physical_core;
	core01=dist0->synapse[1]->l_obj->to_real()->physical_core;
	core10=dist0->synapse[0]->l_obj->to_real()->physical_core;
	core11=dist1->synapse[1]->l_obj->to_real()->physical_core;
	//printf("tcores %lx %lx %lx %lx ",(long)core00,(long)core01,(long)core10,(long)core11);
	printf("test_ease %lx %lx  ",(long)dist0,(long)dist1);
	if(ty0==IVP_ST_EDGE) {
	  printf("edge-edge ");
	} else {
	  printf("point-sur ");
	}
	if(ty1==IVP_ST_EDGE) {
	  printf("edge-edge  ");
	} else {
	  printf("point-sur  ");
	}
	printf("diff %f l0 %f l1 %f\n",lendiff,force0_vec.real_length(),force1_vec.real_length());
	printf("   parallel_ease %f %f %f   %f %f %f\n",force0_vec.k[0],force0_vec.k[1],force0_vec.k[2],force1_vec.k[0],force1_vec.k[1],force1_vec.k[2]);
    }
#endif    
}

void IVP_Friction_Solver::ease_two_mindists(IVP_Contact_Point *dist0,IVP_Contact_Point *dist1,
					    IVP_U_Float_Point *ease_diff_vec0,IVP_U_Float_Point *ease_diff_vec1,
					    IVP_DOUBLE ease_factor) 
{    
    IVP_Core *rev_core=dist0->get_synapse(0)->l_obj->physical_core; //forces seen relative to this core

    int second_core_reversed;
    IVP_FLOAT reverse_factor;
    if(rev_core!=dist1->get_synapse(0)->l_obj->physical_core) {
	second_core_reversed=1;
	reverse_factor = -1.0f;
    } else {
	second_core_reversed=0;
	reverse_factor = 1.0f;
    }
    
    IVP_Impact_Solver_Long_Term *info0 = dist0->tmp_contact_info;
    IVP_Impact_Solver_Long_Term *info1 = dist1->tmp_contact_info;

    IVP_U_Float_Point world_vec0,world_vec1;
    world_vec0.set_multiple(&info0->span_friction_v[0],dist0->span_friction_s[0]);
    world_vec0.add_multiple(&info0->span_friction_v[1],dist0->span_friction_s[1]);

    world_vec1.set_multiple(&info1->span_friction_v[0],dist1->span_friction_s[0]);
    world_vec1.add_multiple(&info1->span_friction_v[1],dist1->span_friction_s[1]);

    IVP_U_Point *world_point[2];
    world_point[0] = &info0->contact_point_ws;
    world_point[1] = &info1->contact_point_ws;
    
    IVP_U_Float_Point world_connection;
    world_connection.subtract(world_point[0],world_point[1]);

    IVP_IF(0) {
	const char *out_text=p_make_string("ease");
	rev_core->environment->add_draw_vector(world_point[1],&world_connection,out_text,3);
	P_FREE(out_text);
    }

    world_connection.fast_normize();
    
    IVP_U_Float_Point ease_part0,ease_part1;
    IVP_DOUBLE part_val0 = world_connection.dot_product(&world_vec0);    
    IVP_DOUBLE part_val1 = world_connection.dot_product(&world_vec1);
    
    ease_part0.set_multiple(&world_connection,part_val0);
    ease_part1.set_multiple(&world_connection,part_val1 * reverse_factor);

    IVP_U_Float_Point optimal_vec;
    optimal_vec.add(&ease_part0,&ease_part1);
    optimal_vec.mult(0.5f);

    IVP_U_Float_Point diff_vec0,diff_vec1;
    diff_vec0.inline_subtract_and_mult(&optimal_vec,&ease_part0,ease_factor); //no full easing
    diff_vec1.inline_subtract_and_mult(&optimal_vec,&ease_part1,ease_factor * reverse_factor);

    ease_diff_vec0->add(&diff_vec0);
    ease_diff_vec1->add(&diff_vec1);
}

void IVP_Friction_Solver::ease_friction_pair(IVP_Friction_Core_Pair *my_pair,IVP_U_Memory *my_mem)
{
    //IVP_U_Float_Point average_v;
    //my_pair->get_average_friction_vector(&average_v);
    //my_pair->set_friction_vectors(&average_v);
    //return;
    
    int total_n = my_pair->number_of_pair_dists();
    IVP_Contact_Point **all_my_dists=(IVP_Contact_Point**)my_mem->get_mem(total_n*sizeof(IVP_Contact_Point*));

    IVP_DOUBLE easing_factor = 1.0f/((IVP_DOUBLE)total_n + P_DOUBLE_EPS);
    
#if defined(IVP_NO_ALLOCA)
    IVP_U_Float_Point *ease_diff_force_vec_stack = (IVP_U_Float_Point*)my_mem->get_mem(total_n*sizeof(IVP_U_Float_Point));
#else
    IVP_U_Float_Point *ease_diff_force_vec_stack=(IVP_U_Float_Point*)alloca(total_n*sizeof(IVP_U_Float_Point));
#endif    
    
    int i=0;
    for (i = my_pair->fr_dists.len()-1; i>=0;i--){
	IVP_Contact_Point *fr_dist = my_pair->fr_dists.element_at(i);
	all_my_dists[i]=fr_dist;
	ease_diff_force_vec_stack[i].set_to_zero();
    }
    
    IVP_IF(0)    {
	printf("ease_testt\n");
	for(i=0;i<total_n-1;i++)
	{
	    IVP_Contact_Point *fr_dist=all_my_dists[i];
	    if((fr_dist->now_friction_pressure>P_DOUBLE_EPS)||1)
	    {
		for(int j=i+1;j<total_n;j++)
		{
		    IVP_Contact_Point *fr_dist2;
		    fr_dist2=all_my_dists[j];
		    if((fr_dist2->now_friction_pressure>P_DOUBLE_EPS)||1)
		    {
			IVP_DOUBLE dir_conform=fr_dist->get_lt()->surf_normal.dot_product(&fr_dist2->get_lt()->surf_normal); //near -1.0f or near 1.0f
			dir_conform=IVP_Inline_Math::fabsd(dir_conform)-1.0f;
			dir_conform=IVP_Inline_Math::fabsd(dir_conform);
			//printf("dirconform %f\n",dir_conform);
			if((dir_conform<1E-3f)||0) //due to random stray in distances the vectors between point-surface and edge-edge are not really parallel
			{
			    //printf("teest ");
			    ease_test_two_mindists(fr_dist2,fr_dist,&fr_dist->get_lt()->surf_normal);
			    //ease_two_mindists(fr_dist2,fr_dist,&world_normal1);
			    //ease_two_mindists(fr_dist2,fr_dist,&world_normal1);
			    //printf("three_eases done \n");
			} else {
			    //printf("noeasingstarted\n");
			}
		    }
		}
	    }
	}
    }
    //printf("real_easing\n");

    IVP_IF(1) {
	my_pair->debug_store_vector_before_ease();
    }
    
    for(i=0;i<total_n-1;i++){
	IVP_Contact_Point *fr_dist=all_my_dists[i];
	if((fr_dist->now_friction_pressure>P_DOUBLE_EPS)||1){
	    for(int j=i+1;j<total_n;j++){
		//printf("epair %d %d\n",i,j);
		IVP_Contact_Point *fr_dist2 = all_my_dists[j];
		if((fr_dist2->now_friction_pressure>P_DOUBLE_EPS)||1){
		    IVP_DOUBLE dir_conform=fr_dist->get_lt()->surf_normal.dot_product(&fr_dist2->get_lt()->surf_normal); //near -1.0f or near 1.0f
		    
		    dir_conform=IVP_Inline_Math::fabsd(dir_conform)-1.0f;
		    dir_conform=IVP_Inline_Math::fabsd(dir_conform);
		    //printf("dirconform %f\n",dir_conform);
		    if((dir_conform<1E-3f)||0) //due to random stray in distances the vectors between point-surface and edge-edge are not really parallel
		    {
		        //printf("eaase ");
			ease_two_mindists(fr_dist2,fr_dist,&ease_diff_force_vec_stack[j],&ease_diff_force_vec_stack[i],easing_factor);
			//ease_two_mindists(fr_dist2,fr_dist,&world_normal1);
			//ease_two_mindists(fr_dist2,fr_dist,&world_normal1);
			//printf("three_eases done \n");
		    } else {
			//printf("noeasingstarted\n");
		    }
		}
	    }
	}
    }

    if(1)    {
	//printf("set_the_friction_force\n");
	for(i=0;i<total_n;i++)	{
	    IVP_Contact_Point *fr_dist=all_my_dists[i];
	    if((fr_dist->now_friction_pressure>P_DOUBLE_EPS)||1)	    {
		fr_dist->ease_the_friction_force(&ease_diff_force_vec_stack[i]);
	    }
	}
    }

    IVP_IF(1) {
	my_pair->debug_read_vector_after_ease();
    }
    
    IVP_IF(0)
    {    
	printf("ease_testt\n");
	for(i=0;i<total_n-1;i++){
	    IVP_Contact_Point *fr_dist=all_my_dists[i];
	    if((fr_dist->now_friction_pressure>P_DOUBLE_EPS)||1)
	    {
		for(int j=i+1;j<total_n;j++)
		{
		    IVP_Contact_Point *fr_dist2;
		    fr_dist2=all_my_dists[j];
		    if((fr_dist2->now_friction_pressure>P_DOUBLE_EPS)||1)
		    {
			IVP_DOUBLE dir_conform=fr_dist->get_lt()->surf_normal.dot_product(&fr_dist2->get_lt()->surf_normal); //near -1.0f or near 1.0f
			dir_conform=IVP_Inline_Math::fabsd(dir_conform)-1.0f;
			dir_conform=IVP_Inline_Math::fabsd(dir_conform);
			//printf("dirconform %f\n",dir_conform);
			if((dir_conform<1E-3f)||0) //due to random stray in distances the vectors between point-surface and edge-edge are not really parallel
			{
			    //printf("teest ");
			    IVP_Friction_Solver::ease_test_two_mindists(fr_dist2,fr_dist,&fr_dist->get_lt()->surf_normal);
			    //ease_two_mindists(fr_dist2,fr_dist,&world_normal1);
			    //ease_two_mindists(fr_dist2,fr_dist,&world_normal1);
			    //printf("three_eases done \n");
			} else {
			    //printf("noeasingstarted\n");
			}
		    }
		}
	    }
	}
    }
}



#if 0 /* not needed any more (snief), but function is correct */
	 //given rot and trans of a core and a surface normal. wanted virtual center p in surface where no translation exists (relative to surface  2D!) 
IVP_RETURN_TYPE IVP_Friction_Solver::calc_virtual_rotation_center(IVP_Core *core,IVP_U_Float_Point *rotation,IVP_U_Float_Point *translation,IVP_U_Float_Point *surf_normal,IVP_U_Float_Point *obj_p_out)
{
    IVP_U_Matrix mat_world_f_z;
    mat_world_f_z.init_normized(surf_normal);

    IVP_U_Float_Point rot_z,trans_z,out_p_z;
    mat_world_f_z.vmult3(rotation,&rot_z);

    IVP_U_Float_Point test_n;
    mat_world_f_z.vmult3(surf_normal,&test_n);
    printf("z_normal %f %f %f\n",test_n.k[0],test_n.k[1],test_n.k[2]);
    
    if(IVP_Inline_Math::fabsd(rot_z.k[2]) < DOUBLE_EPS) {
	return IVP_FAULT;
    }

    IVP_DOUBLE inv_z=1.0f/rot_z.k[2];
    
    mat_world_f_z.vmult3(translation,&trans_z);
    
    out_p_z.k[2]=0.0f;
    out_p_z.k[0]=-trans_z.k[1]*inv_z;
    
    out_p_z.k[1]=trans_z.k[0]*inv_z;

    mat_world_f_z.vimult3(&out_p_z,obj_p_out);

    return IVP_OK;
}
#endif

// move points of synapses in a way unnecessary forces are reduced
void IVP_Friction_System::ease_friction_forces()
{
    IVP_Friction_System *fs=this;
    for (int i = fr_pairs_of_objs.len()-1; i>=0; i--){
    IVP_Friction_Core_Pair *my_pairs = fr_pairs_of_objs.element_at(i);
	my_pairs->next_ease_nr_psi--;
	if(my_pairs->next_ease_nr_psi==0) {
	    //printf("easefr_pair %lx %lx\n",(long)my_pairs->objs[0]&0x0000ffff,(long)my_pairs->objs[1]&0x0000ffff);
	    IVP_Friction_Solver::ease_friction_pair(my_pairs,fs->l_environment->get_memory_manager());
	    my_pairs->next_ease_nr_psi=IVP_EASE_EVERY_NTH_PSI; //do only 5 times a second
	}
    }
}

IVP_Contact_Point::~IVP_Contact_Point(){
    IVP_Environment *env = get_synapse(0)->get_object()->get_environment();

    IVP_Synapse_Friction *syn0 = get_synapse(0);
    syn0->get_object()->get_surface_manager()->remove_reference_to_ledge(syn0->edge->get_compact_ledge());
    IVP_Synapse_Friction *syn1 = get_synapse(1);
    syn1->get_object()->get_surface_manager()->remove_reference_to_ledge(syn1->edge->get_compact_ledge());
    {
	IVP_Event_Friction event_friction;
        event_friction.environment= env;

	IVP_Real_Object *obj0 = get_synapse(0)->l_obj;
	IVP_Real_Object *obj1 = get_synapse(1)->l_obj;
	IVP_Contact_Situation contact_situation;
	contact_situation.objects[0] = obj0;
	contact_situation.objects[1] = obj1;

	contact_situation.compact_edges[0] = get_synapse(0)->edge;
	contact_situation.compact_edges[1] = get_synapse(1)->edge;

	event_friction.contact_situation=&contact_situation;
	
	event_friction.friction_handle = this;
	event_friction.environment->fire_event_friction_deleted(&event_friction);
	{
	    if (obj0->flags.collision_listener_exists){
		IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
		clus_man->fire_event_friction_deleted(obj0, &event_friction);
	    }
	    if (obj1->flags.collision_listener_exists){
		IVP_Cluster_Manager *clus_man = env->get_cluster_manager();
		clus_man->fire_event_friction_deleted(obj1, &event_friction);
	    }
	}
    }
   {
     IVP_Core *core0,*core1;
        IVP_IF( env->get_debug_manager()->check_fs ) {
	    core0=get_synapse(0)->l_obj->friction_core;
	    core1=get_synapse(1)->l_obj->friction_core;
	}
	
	get_synapse(0)->remove_friction_synapse_from_object();
	get_synapse(1)->remove_friction_synapse_from_object();

	IVP_IF( env->get_debug_manager()->check_fs ) {
	    if(core0->physical_unmoveable) {
	        core0->unmovable_core_debug_friction_hash();
	    }
	    if(core1->physical_unmoveable) {
	        core1->unmovable_core_debug_friction_hash();
	    }
	}
    }
}


IVP_FLOAT IVP_Contact_Point_API::get_eliminated_energy(IVP_Contact_Point *friction_handle){
    return friction_handle->integrated_destroyed_energy;
}

void  IVP_Contact_Point_API::reset_eliminated_energy(IVP_Contact_Point *friction_handle){
    friction_handle->integrated_destroyed_energy = 0.0f;
}


IVP_FLOAT IVP_Contact_Point_API::get_vert_force(IVP_Contact_Point *friction_handle){
    return friction_handle->now_friction_pressure;
};

void IVP_Friction_Info_For_Core::friction_info_insert_friction_dist(IVP_Contact_Point *dist)
{
    friction_springs.add(dist);
}

void IVP_Friction_Info_For_Core::friction_info_delete_friction_dist(IVP_Contact_Point *dist)     
{
    friction_springs.remove(dist);
}

int IVP_Friction_Info_For_Core::dist_number()
{
    return friction_springs.len();
}


IVP_Friction_Solver::IVP_Friction_Solver(IVP_Friction_System *fri_sys, const IVP_Event_Sim *es_in)
{
    l_environment=fri_sys->l_environment;
    es = es_in;
#ifdef DEBUG
                gauss_succed=0;
#endif

    IVP_U_Memory *my_mem = fri_sys->l_environment->get_memory_manager();
    dist_change_mat.columns = fri_sys->friction_dist_number - fri_sys->complex_not_necessary_number;    
    dist_change_mat.calc_aligned_row_len();

    dist_change_mat.matrix_values =(IVP_DOUBLE*)my_mem->get_mem((size_t)(dist_change_mat.aligned_row_len*dist_change_mat.columns+IVP_VECFPU_SIZE-1)*sizeof(IVP_DOUBLE));
    dist_change_mat.desired_vector=(IVP_DOUBLE*)my_mem->get_mem((size_t)dist_change_mat.aligned_row_len*sizeof(IVP_DOUBLE));
    dist_change_mat.result_vector =(IVP_DOUBLE*)my_mem->get_mem((size_t)dist_change_mat.aligned_row_len*sizeof(IVP_DOUBLE));
    dist_change_mat.align_matrix_values();
}



void IVP_Friction_System::delete_friction_distance(IVP_Contact_Point *old_dist) {
    //ensure objects have time to move a little bit (e.g. fall down) after contact lost (do not freeze them)
    IVP_Core *core0,*core1;
    core0=old_dist->synapse[0].l_obj->get_core();
    core1=old_dist->synapse[1].l_obj->get_core();
    
    core0->reset_freeze_check_values();
    core1->reset_freeze_check_values();

    this->remove_dist_from_system(old_dist);
    if(this->dist_removed_update_pair_info(old_dist)==IVP_TRUE)
    {
	this->union_find_necessary=IVP_TRUE;
    }
    IVP_Friction_Info_For_Core *fr_info0,*fr_info1;
    fr_info0=core0->get_friction_info(this);
    fr_info1=core1->get_friction_info(this);

    fr_info0->friction_info_delete_friction_dist(old_dist);
    if(fr_info0->dist_number()==0)
    {
	core0->delete_friction_info(fr_info0);
	this->remove_core_from_system(core0);
	core0->sim_unit_of_core->union_find_needed_for_sim_unit=IVP_TRUE;
    }
    fr_info1->friction_info_delete_friction_dist(old_dist);
    if(fr_info1->dist_number()==0)
    {
	core1->delete_friction_info(fr_info1);
	this->remove_core_from_system(core1);
	core1->sim_unit_of_core->union_find_needed_for_sim_unit=IVP_TRUE;
    }
    
    for(int i=0;i<0;i++) {
	IVP_Synapse_Friction *my_syn=old_dist->get_synapse(i);
	if(my_syn->prev) {
	    my_syn->prev->next=my_syn->next;
	} else {
	    my_syn->l_obj->friction_synapses = my_syn->get_next();
	}
	if(my_syn->next) {
	    my_syn->next->prev=my_syn->prev;
	}
    }

    //printf("deleting_frdist %lx o %lx c %lx  o %lx c %lx\n",(long)old_dist,(long)old_dist->synapse[0]->l_obj,(long)old_dist->synapse[0]->l_obj->to_real()->physical_core,(long)old_dist->synapse[1]->l_obj,(long)old_dist->synapse[1]->l_obj->to_real()->physical_core);
    P_DELETE(old_dist);
}



void IVP_Friction_System::apply_real_friction(const IVP_Event_Sim *es)
{
    //fr_solver.mem_friction.init_mem(); // is freed in do_friction_system
    //printf("whole_mindists %ld\n",this->friction_dist_number);
    
    this->calc_friction_forces(es);
    this->ease_friction_forces();

}


// throws second sys in first one
// Optimize: leave pair_info valid, don't process with with dist_added_update_pair_info
void IVP_Friction_System::fusion_friction_systems(IVP_Friction_System *second_sys)
{
    IVP_Friction_System *first_sys=this;
    
    IVP_Simulation_Unit *s1,*s2;
    s1=first_sys->cores_of_friction_system.element_at(0)->sim_unit_of_core;
    s2=second_sys->cores_of_friction_system.element_at(0)->sim_unit_of_core;
  
    IVP_IF(1) {
        IVP_Environment *env=first_sys->l_environment;
	IVP_IF(env->get_debug_manager()->check_fs) {
	    printf("fusion_fs %f %lx %lx  cores ",env->get_current_time().get_time(),(long)first_sys,(long)second_sys);
	    for (int k = first_sys->cores_of_friction_system.len()-1; k>=0; k--){
		IVP_Core *my_core = first_sys->cores_of_friction_system.element_at(k);
	        printf("%lx ",(long)my_core);
	    }
	    printf(" ");
	    for (int l = second_sys->cores_of_friction_system.len()-1; l>=0; l--){
		IVP_Core *my_core = second_sys->cores_of_friction_system.element_at(l);
	        printf("%lx ",(long)my_core);
	    }
	    printf("\n");	    
	}
    }
  
    IVP_Contact_Point *my_dist,*next_dist;
    my_dist=second_sys->get_first_friction_dist();
    //int counter=0;

    IVP_Core *a_core_from_second;
    
    while(my_dist)  {
        next_dist=second_sys->get_next_friction_dist(my_dist);
	second_sys->remove_dist_from_system(my_dist);
	second_sys->dist_removed_update_pair_info(my_dist); //is a little bit slow. remove all core pairs directly. But this way it is saver
        first_sys->add_dist_to_system(my_dist);
	first_sys->dist_added_update_pair_info(my_dist);
	//first_sys->fr_solver.calc_calc_solver(first_sys);	
      	my_dist=next_dist;
	//counter+=2;
    }
    //second_sys->first_friction_dist=NULL; //DIRTY, as this should be done with a loop of deletes, but this way faster

    // now transfer objects ({unmovable!} if necessary) from one system to another

    a_core_from_second=second_sys->moveable_cores_of_friction_system.element_at(0);

    for (int n = second_sys->cores_of_friction_system.len()-1; n>=0;n--){
	IVP_Core *obj1 = second_sys->cores_of_friction_system.element_at(n);
	IVP_Friction_Info_For_Core *fr_info0,*fr_info1;
	fr_info1 = obj1->get_friction_info(second_sys); // this info block must exist
	if( (fr_info0 = obj1->get_friction_info(first_sys)) )	{
	    // this (unmovable) object is part of both systems; transferr mindists and delete second Info-Blocks
	    fr_info1 = obj1->get_friction_info(second_sys);
	    for (int m = fr_info1->friction_springs.len()-1; m>=0; m--){
		my_dist = fr_info1->friction_springs.element_at(m);
		fr_info0->friction_info_insert_friction_dist(my_dist);
		fr_info1->friction_info_delete_friction_dist(my_dist);
	    }
	    obj1->delete_friction_info(fr_info1);
	} else {
	    //this object is not member of first sys, leave info block but change link to system
	    obj1->unlink_friction_info(fr_info1);  // l_friction_system is used as key in hash
	    fr_info1->l_friction_system=first_sys; // now it is save to change l_friction_system
	    obj1->add_friction_info(fr_info1);     // re-add
	    second_sys->remove_core_from_system(obj1);
	    first_sys->add_core_to_system(obj1);
	}
    }

    //remove_friction_system(second_sys);
    IVP_USE(a_core_from_second);

    P_DELETE(second_sys);
    //first_sys->fr_solver.calc_calc_solver(first_sys);
    IVP_IF(1) {
        first_sys->test_hole_fr_system_data();
    }
    //printf("did_fusion\n");
}


int IVP_Friction_Core_Pair::number_of_pair_dists()
{
    return fr_dists.len();
}


void IVP_Friction_Core_Pair::add_fr_dist_obj_pairs(IVP_Contact_Point *dist)
{
#if 0 /* TL: this code is for initializing span_friction_s, but why at this point :( */   
    int numbers = fr_dists.len();
    if( numbers > 0 ) {
	IVP_U_Float_Point friction_vec;
	get_average_friction_vector(&friction_vec);
	IVP_Core *core0 = dist->get_synapse(0)->get_object()->friction_core;
	if( core0 != objs[0] ) {
	    friction_vec.mult(-1.0f);
	}
	IVP_Impact_Solver_Long_Term *info = dist->tmp_contact_info;
	dist->span_friction_s[0]=friction_vec.dot_product(&info->span_friction_v[0]);
	dist->span_friction_s[1]=friction_vec.dot_product(&info->span_friction_v[1]);
    }
#endif    
    fr_dists.add(dist);
}

void IVP_Friction_Core_Pair::del_fr_dist_obj_pairs(IVP_Contact_Point *dist)
{
    fr_dists.remove(dist);
}


void IVP_Friction_System::add_fr_pair(IVP_Friction_Core_Pair *pair)
{
    fr_pairs_of_objs.add(pair);
	pair->objs[0]->get_environment()->fire_event_friction_pair_created( pair );
}

void IVP_Friction_System::del_fr_pair(IVP_Friction_Core_Pair *pair)
{
	pair->objs[0]->get_environment()->fire_event_friction_pair_deleted( pair );
    fr_pairs_of_objs.remove(pair);
}

IVP_Friction_Core_Pair *IVP_Friction_System::get_pair_info_for_objs(IVP_Core *core0,IVP_Core *core1){
    for (int i = fr_pairs_of_objs.len()-1; i>=0; i--){
	IVP_Friction_Core_Pair *my_pair= fr_pairs_of_objs.element_at(i);
	if( (my_pair->objs[0]==core0) || (my_pair->objs[1]==core0) )	{
	    if( (my_pair->objs[0]==core1) || (my_pair->objs[1]==core1) )
	    return my_pair;
	}
    }
    return NULL;
}


// all cores of a simulated friction system have to be simulated
void IVP_Friction_System::debug_check_system_consistency() {
    for (int i = cores_of_friction_system.len()-1; i>=0; i--){
	IVP_Core *core = cores_of_friction_system.element_at(i);
	if(!core->physical_unmoveable)	{
	    IVP_ASSERT(IVP_MTIS_SIMULATED (core->movement_state) );
	}
    }    
}


// make inline
IVP_Core *IVP_Core::union_find_get_father()
{
    IVP_Core *obj=this;
    IVP_Core *on_top=obj;
    while(obj)
    {
	on_top=obj;
	obj=obj->tmp.union_find_father;
    }
    return on_top;
}

// a connection between two objects in fr sys no longer exists, test if fr sys can be splitted in two parts
// fixed objs dont block the splitting
IVP_Core *IVP_Friction_System::union_find_fr_sys()
{
    IVP_Friction_System *fr_sys=this;

    for (int k = fr_sys->cores_of_friction_system.len()-1; k>=0; k--){
	IVP_Core *obje = fr_sys->cores_of_friction_system.element_at(k);
	obje->tmp.union_find_father=NULL;
    }

    for (int l = fr_sys->fr_pairs_of_objs.len()-1; l>=0; l--){
	IVP_Friction_Core_Pair *fr_pair = fr_sys->fr_pairs_of_objs.element_at(l);
	IVP_Core *obj0,*obj1;
	obj0=fr_pair->objs[0];
	obj1=fr_pair->objs[1];
	if(obj0->physical_unmoveable || obj1->physical_unmoveable) {
	    continue; //ignore fixed objs. they cannot connect objects (fixed objs can have more than one friction system)
	}
	//printf("doinguf %lx %lx   ",(long)obj0&0x0000ffff,(long)obj1&0x0000ffff); UFTEST
	obj0=obj0->union_find_get_father();
	obj1=obj1->union_find_get_father();
	//printf("fathers are %lx %lx   ",(long)obj0&0x0000ffff,(long)obj1&0x0000ffff); UFTEST
	if(obj0!=obj1) {
	    obj1->tmp.union_find_father=obj0;
	    //printf("newfather %lx : %lx\n",(long)obj1&0x0000ffff,(long)obj0&0x0000ffff); UFTEST
	}
    }

    IVP_IF(0)	{
	for (int k = fr_sys->cores_of_friction_system.len()-1; k>=0; k--){
	    IVP_Core *objj = fr_sys->cores_of_friction_system.element_at(k);
	    IVP_Core *of=objj->union_find_get_father();
	    printf("uff of %lx : %lx\n",(long)objj&0x0000ffff,(long)of&0x0000ffff);
	}
    }

    
    IVP_Core *first_father=NULL,*second_father=NULL;

    // find representative obj for second system (must not be a fixed obj)
    for (int i1 = fr_sys->cores_of_friction_system.len()-1; i1>=0; i1--){
	IVP_Core *obj = fr_sys->cores_of_friction_system.element_at(i1);
	if(!obj->physical_unmoveable)	{
	    first_father=obj->union_find_get_father();
	}
    }
    IVP_ASSERT(first_father);
    
    for (int i2 = fr_sys->cores_of_friction_system.len()-1; i2>=0; i2--){
	IVP_Core *obj = fr_sys->cores_of_friction_system.element_at(i2);
	if(obj->physical_unmoveable) {
	    continue; //ignore fixed objs
	}
	IVP_Core *test_father=obj->union_find_get_father();
	if(test_father!=first_father)	{
	    second_father=test_father;
	}
    }
    
    return second_father;
}

// split in two parts; every object, that has 'split_father' as union-find father is put into a new system 
void IVP_Friction_System::split_friction_system(IVP_Core *split_father)
{
    IVP_Friction_System *fr_sys=this;
    
    IVP_Friction_System *new_fr_sys=new IVP_Friction_System(l_environment);

     IVP_IF( fr_sys->l_environment->get_debug_manager()->check_fs ) {
        printf("split_fs %f %lx %lx  cores ",fr_sys->l_environment->get_current_time().get_time(),(long)fr_sys,(long)new_fr_sys);
	for (int i = fr_sys->cores_of_friction_system.len()-1; i>=0;i--){
	    IVP_Core *my_core = fr_sys->cores_of_friction_system.element_at(i);
	    printf("%lx ",(long)my_core);
	}
	printf("\n");
     }

    //printf("splitting_now father %lx\n",(long)split_father&0x0000ffff);
    //fr_sys->debug_fs_out_ascii();
    {
	//first transfer objs
	for (int j = fr_sys->cores_of_friction_system.len()-1; j>=0; j--){
	    IVP_Core *obj=fr_sys->cores_of_friction_system.element_at(j);
	  //printf("   %lx hasfather %lx ",(long)obj&0x0000ffff,(long)union_find_get_father(obj)&0x0000ffff);  
	    if(obj->physical_unmoveable)    {
	        //printf("isumv");
		// fixed objs first belong to both systems and get two IVP_Friction_Info_For_Core, later deleted if necessary
		IVP_Friction_Info_For_Core *fr_i=new IVP_Friction_Info_For_Core();
		fr_i->l_friction_system=new_fr_sys;
		new_fr_sys->add_core_to_system(obj);
		obj->add_friction_info(fr_i);
	    } else {
		if(obj->union_find_get_father()==split_father)
		{
		    fr_sys->remove_core_from_system(obj);
		    new_fr_sys->add_core_to_system(obj);
		    IVP_Friction_Info_For_Core *fr_i=obj->get_friction_info(fr_sys);
		    fr_i->l_friction_system=new_fr_sys;
		}
	    }
	}
	//printf("\n");
    }

    {
	// transfer mindists (take pairs as orientation)
	for (int n = fr_sys->fr_pairs_of_objs.len()-1; n>=0; n--){
	IVP_Friction_Core_Pair *fr_pair = fr_sys->fr_pairs_of_objs.element_at(n);
	    IVP_Core *robj=fr_pair->objs[0]; //is used to identify fr sys my pair should belong to
	    IVP_Core *robj2=fr_pair->objs[1]; // used when fixed obj is involved 
	    IVP_Friction_Info_For_Core *fr_i_old = NULL,*fr_i_new = NULL; // when fixed obj exists, dists have to be transfered from fr_i_old to fr_i_new
	    if(robj->physical_unmoveable)   {
	        IVP_Core *ct;
		ct=robj2;
		robj2=robj;
		robj=ct;
		goto found_fixed;
	    } else {
		if(robj2->physical_unmoveable){
		found_fixed:    
		    fr_i_old=robj2->get_friction_info(fr_sys);
		    fr_i_new=robj2->get_friction_info(new_fr_sys);
		}
	    }

      	    //printf("reassignpair %lx infoc %lx",(long)fr_pair&0x0000ffff,(long)robj&0x0000ffff);
	    if(robj->union_find_get_father()==split_father)	    {
	      //printf(" to_new");
		fr_sys->del_fr_pair(fr_pair);
		new_fr_sys->add_fr_pair(fr_pair);
		for (int k = fr_pair->fr_dists.len()-1; k>=0; k--){
		    IVP_Contact_Point *fr_dist=fr_pair->fr_dists.element_at(k);
		    int found_mine=0;
		    IVP_Contact_Point *mindist;
		    for(mindist=fr_sys->get_first_friction_dist();mindist;mindist=fr_sys->get_next_friction_dist(mindist))   {
			if(mindist==fr_dist) {
			    found_mine=1;
			}
		    }
		    if(!found_mine)
		    {
			IVP_IF(1) { printf("removing_dist that doesnt belong to sys\n"); }
			CORE;
		    }
		    fr_sys->remove_dist_from_system(fr_dist);
		    new_fr_sys->add_dist_to_system(fr_dist);
		    //printf("add_to_new %lx fs_num_now %ld\n",(long)fr_dist,new_fr_sys->friction_dist_number); UFTEST
		    if(fr_i_old)
		    {
			// a fixed obj is involved. fixed objs have friction_info in BOTH systems
			fr_i_old->friction_info_delete_friction_dist(fr_dist);
			fr_i_new->friction_info_insert_friction_dist(fr_dist);
		    }
		}
	    }
	    //printf("jjj\n");
	}
    }

    //printf("\nbefore_removal\n");
    //fr_sys->debug_fs_out_ascii();
    //new_fr_sys->debug_fs_out_ascii();
    {
	// test if fixed objs can be removed from old system
	for (int l = fr_sys->cores_of_friction_system.len()-1; l>=0; l--){
	    IVP_Core *obj=fr_sys->cores_of_friction_system.element_at(l);
	    if(obj->physical_unmoveable)    {
		// fixed objs belong to both systems and have two IVP_Friction_Info_For_Obj
		IVP_Friction_Info_For_Core *fr_i_new=obj->get_friction_info(new_fr_sys);
		IVP_Friction_Info_For_Core *fr_i_old=obj->get_friction_info(fr_sys);
		if(fr_i_new->dist_number()==0) {
		  //printf("uf_rem %lx from new %lx\n",(long)obj&0x0000ffff,(long)new_fr_sys&0x0000ffff);
		    obj->delete_friction_info(fr_i_new);
		    new_fr_sys->remove_core_from_system(obj);
		}
		if(fr_i_old->dist_number()==0) {
		  //printf("uf_rem %lx from old %lx\n",(long)obj&0x0000ffff,(long)fr_sys&0x0000ffff);
		    obj->delete_friction_info(fr_i_old);
		    fr_sys->remove_core_from_system(obj);
		}
	    } 
	}
    } 
    
    if(new_fr_sys->friction_obj_number<2) {
      //printf("split_only_one_new\n"); //UFTEST
	IVP_Core *obj=new_fr_sys->cores_of_friction_system.element_at(0);
	IVP_Friction_Info_For_Core *fr_i = obj->get_friction_info(new_fr_sys);
	obj->delete_friction_info(fr_i);
	P_DELETE(fr_i);
	P_DELETE(new_fr_sys);
	return ;
    }
    if(fr_sys->friction_obj_number<2) {
      //printf("split_only_one_old\n"); //UFTEST
	IVP_Core *obj=fr_sys->cores_of_friction_system.element_at(0);
	IVP_Friction_Info_For_Core *fr_i=obj->get_friction_info(fr_sys);
	obj->delete_friction_info(fr_i);
	P_DELETE(fr_i);
	P_DELETE(fr_sys);
	return ;
    }
    //printf("splitted_frs %lx\n",(long)fr_sys); //UFTEST
    //new_fr_sys->fr_solver.calc_calc_solver(new_fr_sys);
    //fr_sys->fr_solver.calc_calc_solver(fr_sys);
    //this->add_friction_system(new_fr_sys);
    IVP_IF(1==1) //expensive but important
    {
      //printf("split_first_result\n");
      //new_fr_sys->debug_fs_out_ascii();
      //printf("split_second_result\n");
      //fr_sys->debug_fs_out_ascii();
      new_fr_sys->test_hole_fr_system_data();
      fr_sys->test_hole_fr_system_data();
    }
    {
	// it may happen, that a system can be split in more than two parts (very seldom)
	IVP_Core *obj=fr_sys->union_find_fr_sys();
	if(obj) {
	    fr_sys->split_friction_system(obj); //split recorsively if necessary
	}
    }
}

void IVP_Friction_System::print_all_dists()
{
    IVP_IF(1) {
    printf("fs %lx  ",(long)this&0x0000ffff);
		for(IVP_Contact_Point *mindist=this->get_first_friction_dist();mindist;mindist=this->get_next_friction_dist(mindist))
		{
		    printf("%lx ",(long)mindist&0x0000ffff);
		}    
    printf("\n");
    printf("      ");
    for (int i = fr_pairs_of_objs.len()-1; i>=0; i--){
	IVP_Friction_Core_Pair *fr_pair = fr_pairs_of_objs.element_at(i);
	printf("p %lx %lx  ",(long)fr_pair->objs[0]&0x0000ffff,(long)fr_pair->objs[1]&0x0000ffff);
	for (int c = fr_pair->fr_dists.len()-1;c>=0; c--){
	    IVP_Contact_Point *fr_dist=fr_pair->fr_dists.element_at(c);
	    printf("%lx ",(long)fr_dist&0x0000ffff);
	}
    }
    printf("\n");
    }
}

void IVP_Friction_System::reset_time(IVP_Time offset){
    for (int i = fr_pairs_of_objs.len()-1; i>=0; i--){
	IVP_Friction_Core_Pair *fr_pair = fr_pairs_of_objs.element_at(i);
	for (int c = fr_pair->fr_dists.len()-1;c>=0; c--){
	    IVP_Contact_Point *my_dist = fr_pair->fr_dists.element_at(c);
	    my_dist->reset_time(offset);
	}
    }
}

// return zero when pair no longer exists
int IVP_Friction_Core_Pair::check_all_fr_mindists_to_be_valid(IVP_Friction_System *my_fs) {
    int total_number_remaining = this->fr_dists.len();
    for (int k = fr_dists.len()-1; k>=0; k--){
	IVP_Contact_Point *my_dist=this->fr_dists.element_at(k);
	my_dist->recalc_friction_s_vals();
	IVP_Impact_Solver_Long_Term *info=my_dist->tmp_contact_info;
	my_dist->read_materials_for_contact_situation(info);
	
	//printf("impact_sys_update_contact_vals %lx\n",(long)my_dist);
	if( info->friction_is_broken == IVP_TRUE) {
	    total_number_remaining--;
	    my_fs->delete_friction_distance(my_dist);
	}
    }
    //warning: at this point 'this' might have been deleted

    return total_number_remaining;
}


void IVP_Friction_System::remove_energy_gained_by_real_friction()
{
    for (int i = fr_pairs_of_objs.len()-1; i>=0; i--){
	IVP_Friction_Core_Pair *my_pair = fr_pairs_of_objs.element_at(i);
	my_pair->remove_energy_gained_by_real_friction();
    }
}

void IVP_Friction_System::clear_integrated_anti_energy()
{
    for (int k = fr_pairs_of_objs.len()-1; k>=0; k--){
	IVP_Friction_Core_Pair *my_pair = fr_pairs_of_objs.element_at(k);
	my_pair->integrated_anti_energy=0.0f;
    }
}


void IVP_Friction_Core_Pair::remove_energy_gained_by_real_friction()
{
    integrated_anti_energy *= objs[0]->environment->get_integrated_energy_damp(); //to avoid accumulation to neverending
    if(integrated_anti_energy<0.0f){ 
	return;
    }
        //printf("destroyyenergy %.8f\n",integrated_anti_energy);
#ifdef IVP_FAST_WHEELS_ENABLED
    if (objs[0]->car_wheel || objs[1]->car_wheel){
	return;
    }
#endif

#ifndef NO_MUTUAL_ENERGYDESTROY
    IVP_DOUBLE amount_energy_destr;
    amount_energy_destr=destroy_mutual_energy(integrated_anti_energy);
    //printf("destr_eee %f\n",amount_energy_destr);
    //printf("destroyed %f\n",amount_energy_destr);
//	IVP_IF(1) {
	objs[0]->environment->get_statistic_manager()->sum_energy_destr+=amount_energy_destr;
//	}
    integrated_anti_energy-=amount_energy_destr;
#endif
}

IVP_DOUBLE IVP_Mutual_Energizer::calc_impulse_to_reduce_energy_level(IVP_DOUBLE speed_pot,IVP_DOUBLE inv_mass0,IVP_DOUBLE inv_mass1,IVP_DOUBLE delta_e)
{
    delta_e*=2.0f;
    IVP_DOUBLE divisor=1.0f/(inv_mass0+inv_mass1);
    IVP_DOUBLE root=(speed_pot*speed_pot-(inv_mass0+inv_mass1)*delta_e); ///(mass1*speed_pot*speed_pot-target_e));
    root = IVP_Inline_Math::fabsd(root);
    root = IVP_Inline_Math::ivp_sqrtf(root);
    IVP_DOUBLE x=(speed_pot-root)*divisor;
    return x;
}

IVP_DOUBLE IVP_Mutual_Energizer::calc_energy_potential(IVP_DOUBLE speed_pot,IVP_DOUBLE mass0,IVP_DOUBLE mass1,IVP_DOUBLE inv_mass0,IVP_DOUBLE inv_mass1)
{
    IVP_DOUBLE x; //impulse done on objects to destroy potential, first obj is still
    //Energy E1 of obj1: mass1 * (speed_pot - x*inv_mass1)^2
    //Energy E0 of obj0: mass0 * (x*inv_mass0)^2
    //point of minimal Energy: E1'(x) = -E0'(x)

    IVP_DOUBLE energy_now=mass1*speed_pot*speed_pot;
    
    x=speed_pot/(inv_mass0+inv_mass1); //solution of E1'(x) = -E0'(x) -> speed of both objects get identical, speed_pot gets zero

    IVP_DOUBLE E0,E1;
    IVP_DOUBLE speed;
    speed=speed_pot - x*inv_mass1;
    E1=mass1*speed*speed;

    speed=x*inv_mass0;
    E0=mass0*speed*speed;

    IVP_DOUBLE tmp = energy_now+P_DOUBLE_EPS - (E0+E1);
	// VALVE: For very large energies (e.g. 3e15) this can be slightly off due to precision errors :(
	// Clamp instead
	// IVP_ASSERT(tmp>=0);
	if ( tmp < 0 )
	{
		tmp = 0;
	}

    return 0.5f*(energy_now-E0-E1);
}

// rot moment in a direction
// rot_vec_obj is normized
void IVP_Mutual_Energizer::get_rot_inertia(IVP_Core *core,IVP_U_Float_Point *rot_vec_obj,IVP_DOUBLE *rot_inertia,IVP_DOUBLE *inv_rot_inertia)
{
    IVP_U_Float_Point rot_vec_abs;
    rot_vec_abs.set_pairwise_mult(rot_vec_obj,core->get_rot_inertia());
    *rot_inertia=rot_vec_abs.real_length();
    if(*rot_inertia<P_DOUBLE_EPS) {
	*rot_inertia=1.0f;
	*inv_rot_inertia=1.0f;
    } else {
	*inv_rot_inertia=1.0f/(*rot_inertia);
    }
}

void IVP_Mutual_Energizer::init_mutual_energizer(IVP_Core *core0,IVP_Core *core1) {
    if(core1->physical_unmoveable || core1->pinned) { //@@CBPIN
	this->core[0]=core1;
	this->core[1]=core0;
    } else {
	this->core[1]=core1;
	this->core[0]=core0;
    }
    //now core1 is moveable

    {
	trans_vec_world.subtract(&core[1]->speed,&core[0]->speed);
	trans_speed_potential=trans_vec_world.real_length_plus_normize();
    }
    {
	IVP_U_Float_Point rot0_world;   	core[0]->m_world_f_core_last_psi.vmult3(&core[0]->rot_speed,&rot0_world);
	IVP_U_Float_Point rot1_world;   	core[1]->m_world_f_core_last_psi.vmult3(&core[1]->rot_speed,&rot1_world);
	IVP_U_Float_Point rot_vec_world; 	rot_vec_world.subtract(&rot1_world,&rot0_world);
	
	rot_speed_potential = rot_vec_world.real_length_plus_normize();
	core[0]->m_world_f_core_last_psi.vimult3(&rot_vec_world,&rot_vec_obj[0]);
	rot_vec_world.mult(-1.0f);
	core[1]->m_world_f_core_last_psi.vimult3(&rot_vec_world,&rot_vec_obj[1]);
	
	get_rot_inertia(core[0],&rot_vec_obj[0],&rot_inertia[0],&inv_rot_inertia[0]);
	get_rot_inertia(core[1],&rot_vec_obj[1],&rot_inertia[1],&inv_rot_inertia[1]);
    }
    {
	trans_inertia[1] = core[1]->get_mass();
	inv_trans_inertia[1]=core[1]->get_inv_mass();
	if(core[0]->physical_unmoveable || core[0]->pinned) { //@@CBPIN
	    trans_inertia[0]=10000*trans_inertia[1];
	    inv_trans_inertia[0]=inv_trans_inertia[1]*(1.0f/10000.0f);
	    rot_inertia[0]=10000*rot_inertia[1];
	    inv_rot_inertia[0]=inv_rot_inertia[1]*(1.0f/10000.0f);
	} else {
	    trans_inertia[0]=core[0]->get_mass();
	    inv_trans_inertia[0]=core[0]->get_inv_mass();
	}
    }
}

void IVP_Mutual_Energizer::calc_energy_potential() {   
	rot_energy_potential = calc_energy_potential(rot_speed_potential,rot_inertia[0],rot_inertia[1],inv_rot_inertia[0],inv_rot_inertia[1]);
	trans_energy_potential = calc_energy_potential(trans_speed_potential,trans_inertia[0],trans_inertia[1],inv_trans_inertia[0],inv_trans_inertia[1]);
	whole_mutual_energy = trans_energy_potential+rot_energy_potential;
}

void IVP_Mutual_Energizer::destroy_percent_energy(IVP_DOUBLE percent_energy_to_destroy)
{
    IVP_DOUBLE rot_impulse=calc_impulse_to_reduce_energy_level(rot_speed_potential,inv_rot_inertia[0],inv_rot_inertia[1],percent_energy_to_destroy*rot_energy_potential);
    IVP_DOUBLE trans_impulse=calc_impulse_to_reduce_energy_level(trans_speed_potential,inv_trans_inertia[0],inv_trans_inertia[1],percent_energy_to_destroy*trans_energy_potential);
    if(!core[0]->physical_unmoveable) {
	core[0]->speed_change.add_multiple(&trans_vec_world,inv_trans_inertia[0]*trans_impulse);
	core[0]->rot_speed_change.add_multiple(&rot_vec_obj[0],inv_rot_inertia[0]*rot_impulse);
    }
    trans_vec_world.mult(-1.0f);
    //printf("muttd %f %f\n",trans_impulse,rot_impulse);
    core[1]->speed_change.add_multiple(&trans_vec_world,inv_trans_inertia[1]*trans_impulse);
    core[1]->rot_speed_change.add_multiple(&rot_vec_obj[1],inv_rot_inertia[1]*rot_impulse);
}

IVP_DOUBLE IVP_Friction_Core_Pair::destroy_mutual_energy(IVP_DOUBLE d_e){
    IVP_Mutual_Energizer mutual_energizer_stack;
    mutual_energizer_stack.init_mutual_energizer(objs[0],objs[1]);

    mutual_energizer_stack.calc_energy_potential();
    IVP_DOUBLE max_energy_to_destroy = MAX_ENERGY_DESTROY * mutual_energizer_stack.whole_mutual_energy;
    if(d_e > max_energy_to_destroy) {
	d_e=max_energy_to_destroy;
	//d_e=whole_mutual_energy;
    }
    if(mutual_energizer_stack.whole_mutual_energy < P_DOUBLE_EPS) {
	return 0.0f;
    }
    //printf("destroyy %f\n",d_e);
    IVP_DOUBLE percent_energy_to_destroy = d_e / mutual_energizer_stack.whole_mutual_energy;

    mutual_energizer_stack.destroy_percent_energy(percent_energy_to_destroy);    
    return d_e;
}

IVP_Friction_Core_Pair::~IVP_Friction_Core_Pair() {
    //printf("deleteing_core_pair %lx\n",0x0000ffff&(long)this);
}

IVP_Friction_Core_Pair::IVP_Friction_Core_Pair()
{
    last_impact_time_pair=-1000.0f; //negative time
    integrated_anti_energy=0.0f;
    next_ease_nr_psi=1;
}

void IVP_Friction_Core_Pair::set_friction_vectors(IVP_U_Float_Point *average_friction) {
    for (int i = fr_dists.len()-1; i>=0; i--){
	IVP_Contact_Point *fr_dist = fr_dists.element_at(i);
	    IVP_DOUBLE sign;
	    if( fr_dist->get_synapse(0)->get_object()->physical_core == objs[0] ) {
		sign = 1.0f;
	    } else {
		sign =-1.0f;
	    }
	    IVP_Impact_Solver_Long_Term *info = fr_dist->tmp_contact_info;
	    fr_dist->span_friction_s[0]=average_friction->dot_product(&info->span_friction_v[0])*sign;
	    fr_dist->span_friction_s[1]=average_friction->dot_product(&info->span_friction_v[1])*sign;
	}
}

void IVP_Friction_Core_Pair::get_average_friction_vector(IVP_U_Float_Point *average_friction) {
    average_friction->set_to_zero();
    int numbers = fr_dists.len();
    if( numbers > 0 ) {
	IVP_DOUBLE factor = 1.0f/numbers;
	for (int i = fr_dists.len()-1; i>=0; i--){
	    IVP_Contact_Point *fr_dist = fr_dists.element_at(i);
	    IVP_U_Float_Point temp_v;
	    temp_v.set_to_zero();
	    IVP_Impact_Solver_Long_Term *info = fr_dist->tmp_contact_info;
	    temp_v.add_multiple(&info->span_friction_v[0],fr_dist->span_friction_s[0]);
	    temp_v.add_multiple(&info->span_friction_v[1],fr_dist->span_friction_s[1]);
	    IVP_DOUBLE sign;
	    if( fr_dist->get_synapse(0)->get_object()->physical_core == objs[0] ) {
		sign = 1.0f;
	    } else {
		sign =-1.0f;
	    }

	    average_friction->add_multiple(&temp_v,sign);
	}
	average_friction->mult(factor);
    }
}

void IVP_Friction_Core_Pair::debug_printf_pair() {
    IVP_IF(1) {
	IVP_Real_Object *obj1,*obj2;
	obj1=objs[0]->objects.element_at(0);
	obj2=objs[1]->objects.element_at(0);
	printf("frpair_names %s %s ",obj1->get_name(),obj2->get_name());
    }
}

void IVP_Friction_Core_Pair::debug_store_vector_before_ease() {
    span_vector_sum.set_to_zero();
    get_average_friction_vector(&span_vector_sum);
}

void IVP_Friction_Core_Pair::debug_read_vector_after_ease() {
    IVP_U_Float_Point test_vec;
    test_vec.set_to_zero();
    get_average_friction_vector(&test_vec);
#if 0    
    IVP_Contact_Point *fr_dist;
    for(fr_dist=this->get_first_fr_dist_obj_pairs(); fr_dist; fr_dist=this->get_next_fr_dist_obj_pairs()) {
	IVP_U_Float_Point temp_v;
	temp_v.set_to_zero();
    IVP_Impact_Solver_Long_Term *info0 = &dist0.long_term_impact_info;
	temp_v.add_multiple(&fr_dist->span_friction_v[0],fr_dist->span_friction_s[0]);
	temp_v.add_multiple(&fr_dist->span_friction_v[1],fr_dist->span_friction_s[1]);
	IVP_DOUBLE sign;
	if( fr_dist->synapse[0]->l_obj->physical_core == objs[0] ) {
	    sign = 1.0f;
	} else {
	    sign =-1.0f;
	}

	test_vec.add_multiple(&temp_v,sign);
    }
#endif
    IVP_U_Float_Point diff;
    diff.subtract(&test_vec,&span_vector_sum);
    if( diff.real_length() > 0.01f ) {
	printf("easingerror: %f %f %f should equal %f %f %f\n",test_vec.k[0],test_vec.k[1],test_vec.k[2],span_vector_sum.k[0],span_vector_sum.k[1],span_vector_sum.k[2]);
    }
}


void IVP_Friction_System::debug_fs_out_ascii()
{
    IVP_IF(1) {
    printf("fs %lx  ",(long)this&0x0000ffff);
		for(IVP_Contact_Point *mindist=this->get_first_friction_dist();mindist;mindist=this->get_next_friction_dist(mindist))
		{
		    printf("%lx ",(long)mindist&0x0000ffff);
		}    
    printf("\n");
    for (int k = cores_of_friction_system.len()-1; k>=0; k--){
	IVP_Core *my_core = cores_of_friction_system.element_at(k);
        printf("    core %lx  ",(long)my_core&0x0000ffff);
	IVP_Friction_Info_For_Core *inf=my_core->get_friction_info(this);
	printf("lfs %lx  ",(long)inf->l_friction_system&0x0000ffff);

	for (int i = inf->friction_springs.len()-1; i>=0; i--){
	    IVP_Contact_Point *mindist = inf->friction_springs.element_at(i);
	    printf("%lx  ",(long)mindist&0x0000ffff);
	}
    }
    printf("\n");
    for (int m = fr_pairs_of_objs.len()-1; m>=0;m--){
	IVP_Friction_Core_Pair *fr_pair = fr_pairs_of_objs.element_at(m);
	printf("    p %lx %lx  ",(long)fr_pair->objs[0]&0x0000ffff,(long)fr_pair->objs[1]&0x0000ffff);
	for (int c = fr_pair->fr_dists.len()-1; c>=0; c--){
	    IVP_Contact_Point *fr_dist= fr_pair->fr_dists.element_at(c);
	    printf("%lx ",(long)fr_dist&0x0000ffff);
	}
    }
    printf("\n");
    }
}

IVP_BOOL IVP_Friction_System::core_is_found_in_pairs(IVP_Core *test_core)
{
    for (int m = fr_pairs_of_objs.len()-1; m>=0;m--){
	IVP_Friction_Core_Pair *fr_pair = fr_pairs_of_objs.element_at(m);
        if( (fr_pair->objs[0]==test_core) || (fr_pair->objs[1]==test_core) )
	{
	    return IVP_TRUE;
	}
    }
    return IVP_FALSE;
}

IVP_Friction_Core_Pair *IVP_Friction_System::find_pair_of_cores(IVP_Core *core0,IVP_Core *core1)
{
    for (int m = fr_pairs_of_objs.len()-1; m>=0;m--){
	IVP_Friction_Core_Pair *my_pair = fr_pairs_of_objs.element_at(m);
	if( ((my_pair->objs[0]==core0)&&(my_pair->objs[1]==core1)) || ((my_pair->objs[0]==core1)&&(my_pair->objs[1]==core0)) ) {
	    return my_pair;
	}
    }
    return NULL;
}

#if 0
IVP_Mindist_Collision_Watcher::IVP_Mindist_Collision_Watcher()
{
    some_pushes_after_turnaround=0;
    time_diff=0.0f;
}
#endif

// be sure the core point of contact is valid
// this function is called at generation of mindist. It should be called more often, as contact point can slide away
void IVP_Contact_Point::calc_virtual_mass_of_mindist() {
    //do we have two friction values ?
    IVP_Material *mtl[2];
    get_material_info(mtl);
    if( mtl[0]->second_friction_x_enabled || mtl[1]->second_friction_x_enabled ) {
        this->two_friction_values=IVP_TRUE;
    }
    
    IVP_Core *core[2];
    for(int i=0;i<2;i++) {
	core[i] = get_synapse(i)->get_object()->get_core();
    }
    IVP_DOUBLE virt_mass_mindist_no_dir;
    if(core[0]->physical_unmoveable) {
	virt_mass_mindist_no_dir =  core[1]->calc_virt_mass_worst_case(&tmp_contact_info->contact_point_cs[1]);
    } else {
	if(core[1]->physical_unmoveable) {
	    virt_mass_mindist_no_dir =  core[0]->calc_virt_mass_worst_case(&tmp_contact_info->contact_point_cs[0]);
	} else {
	    IVP_DOUBLE vmass_no_dir[2];
	    for (int j = 0;j<2;j++){
		vmass_no_dir[j] = core[j]->calc_virt_mass_worst_case(&tmp_contact_info->contact_point_cs[j]);
	    }
	    virt_mass_mindist_no_dir=(vmass_no_dir[0]*vmass_no_dir[1])/(vmass_no_dir[0]+vmass_no_dir[1]);
	}
    }
    this->inv_virt_mass_mindist_no_dir = 1.0f / virt_mass_mindist_no_dir;
    IVP_ASSERT(virt_mass_mindist_no_dir < 1e30f); // test for NaN
}

inline IVP_FLOAT ivp_minimum(IVP_FLOAT a,IVP_FLOAT b) {
    if(a > b) {
	return b;
    } else {
	return a;
    }
}

// returns IVP_TRUE when a friction system has been grown
IVP_BOOL IVP_Core::grow_friction_system() {

    IVP_IF( environment->get_debug_manager()->check_fs ) {
	printf("growing_fs %f core %lx\n",environment->get_current_time().get_time(),(long)this);
    }

    IVP_BOOL grew_new_contact_point=IVP_FALSE;
    
    //IVP_U_Vector<IVP_Core> wake_up_cores;
    IVP_Core *start_core=this;
    int c;
    for(c = objects.len()-1;c>=0;c--) {
	IVP_Real_Object *r_obj=this->objects.element_at(c);

	IVP_Synapse_Real *my_synapse, *next_synapse;
	for(my_synapse=r_obj->get_first_exact_synapse();   my_synapse;    my_synapse = next_synapse) {
	    next_synapse = my_synapse->get_next();
	    IVP_Mindist *my_mindist = my_synapse->get_mindist();

	    if (my_mindist->mindist_function != IVP_MF_COLLISION) {  // grow real collisions only
	      continue;
	    }
	    
	    IVP_DOUBLE mindist_distance;
	    
	    IVP_Core *other_core=my_mindist->get_synapse(0)->get_object()->get_core();
	    if(other_core == start_core) {
	        other_core=my_mindist->get_synapse(1)->get_object()->get_core();
	    }
	    if(other_core->physical_unmoveable) {
		if( this->physical_unmoveable ) {
		    //printf("unnecessary_mindist\n");
		    P_DELETE( my_mindist );
		}
		continue;
	    }

	    /* When there are cores in friction systems enable this part */
	    IVP_Friction_Info_For_Core *fr_info=other_core->moveable_core_has_friction_info();
	    if(fr_info) {
	        //continue;
		goto next_in_loop;
	    }
	    
	    my_mindist->recalc_mindist();
	    if (my_mindist->recalc_result != IVP_MDRR_OK){ 
		continue;
	    }
	    
	    mindist_distance = my_mindist->get_length();

	    if( mindist_distance < ivp_mindist_settings.max_distance_for_friction ) {
		IVP_Friction_System *affected_friction_system;
		IVP_BOOL having_new_dist;
		this->environment->sim_unit_mem->start_memory_transaction();
		my_mindist->try_to_generate_managed_friction(&affected_friction_system,&having_new_dist,this->sim_unit_of_core,IVP_TRUE); //do not call recalc_friction_s_vals
		this->environment->sim_unit_mem->end_memory_transaction();
		if(having_new_dist==IVP_TRUE) {
		    grew_new_contact_point=IVP_TRUE;
		    other_core->reset_freeze_check_values();
		}
	    }
	    //printf("\n\n\nnot_grow_with_core %lx\n\n\n",(long)other_core);
next_in_loop:;
	}
    }

    return grew_new_contact_point;
}


void IVP_Friction_System::get_controlled_cores(IVP_U_Vector<IVP_Core> *vectr) {
    vectr=NULL;
}

IVP_DOUBLE IVP_Friction_System::get_minimum_simulation_frequency() {
    return 1.0f;
}

IVP_U_Vector<IVP_Core> *IVP_Friction_System::get_associated_controlled_cores() {
    return &moveable_cores_of_friction_system;
}

void IVP_Friction_Sys_Energy::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> *) {
    //special case for one contact point
    if( (l_friction_system->friction_dist_number<=1 )&&1) {
	IVP_Contact_Point *cp=l_friction_system->get_first_friction_dist();
	cp->recalc_friction_s_vals();
	return;
    }
    
    l_friction_system->fs_recalc_all_contact_points();
#if !defined( NO_MUTUAL_ENERGYDESTROY )
    if(es->sim_unit->sim_unit_just_slowed_down) {
	l_friction_system->clear_integrated_anti_energy();
    }
    if(!es->sim_unit->sim_unit_has_fast_objects) {
	l_friction_system->remove_energy_gained_by_real_friction();
    }
#endif    
}

void IVP_Friction_Sys_Static::do_simulation_single_friction(IVP_Event_Sim *es) {
    //maybe destroy anti energy of first core_pair

    IVP_Contact_Point *cp=l_friction_system->get_first_friction_dist();

    cp->static_friction_single(es,ivp_mindist_settings.friction_dist,1.0f);

    if((cp->get_gap_length() >= ivp_mindist_settings.max_distance_for_friction) || (cp->get_lt()->friction_is_broken == IVP_TRUE)){    	    
	l_friction_system->delete_friction_distance(cp);
	return;
    }
}

void IVP_Friction_Sys_Static::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> * /*core_list*/) {
    if( (l_friction_system->friction_dist_number <= 1 )&&1) {
	do_simulation_single_friction(es);
    } else {
	l_friction_system->do_friction_system(es);
    }

    //when all fr_dists are deleted this affects sim_u (controller is deleted), but does no harm (luckily)
    
    if( l_friction_system->friction_dist_number == 0 ) {
      //sim_u->rem_sim_unit_controller( l_friction_system );
      //sim_u->rem_sim_unit_controller( &l_friction_system->static_fs_handle );
        IVP_Friction_System *temp_sys=l_friction_system;
	l_friction_system=NULL; 
	P_DELETE( temp_sys );
	es->sim_unit->union_find_needed_for_sim_unit=IVP_TRUE;
	return;
    }
    if( l_friction_system->union_find_necessary ) {
        l_friction_system->union_find_necessary=IVP_FALSE;
        //IVP_Core *obj=l_friction_system->l_environment->get_friction_manager()->union_find_fr_sys(l_friction_system);
	IVP_Core *obj=l_friction_system->union_find_fr_sys();
        if(obj) {
	    //l_friction_system->l_environment->get_friction_manager()->split_friction_system(obj,l_friction_system);
	    l_friction_system->split_friction_system(obj);
	    obj->sim_unit_of_core->union_find_needed_for_sim_unit=IVP_TRUE;
	    //obj->sim_unit_of_core->perform_test_and_split();
        }
    }
    IVP_IF(1) {
	l_friction_system->debug_clean_tmp_info();
    }
}

void IVP_Friction_System::do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> *) {
    //special case single friction
    if( ( friction_dist_number<=1 )&&1) {
	IVP_Contact_Point *cp=this->get_first_friction_dist();
	//cp->recalc_friction_s_vals();

	IVP_FLOAT max_local_spring_len = cp->get_possible_friction_slide_way(es) * es->delta_time * es->delta_time;

	cp->calc_pretension( max_local_spring_len );
	
	if(cp->two_friction_values!=IVP_TRUE) {
	    cp->friction_force_local_constraint_2d(es);
	} else {
	    cp->friction_force_local_constraint_1d(es);
	}
	
	return;
    }
    apply_real_friction(es);
}

void IVP_Friction_System::core_is_going_to_be_deleted_event(IVP_Core *core) {
    IVP_USE(core);
}

void IVP_Friction_Sys_Energy::core_is_going_to_be_deleted_event(IVP_Core *core) {
    IVP_USE(core);
}

int IVP_Synapse_Friction::get_material_index() const {
    return this->edge->get_triangle()->get_material_index();
}



IVP_BOOL IVP_Synapse_Friction::is_same_as(const IVP_Synapse_Real *syn1) const
{
    const IVP_Synapse_Friction *syn0 = this;
    IVP_ASSERT( syn0->l_obj == syn1->l_obj );
    
    IVP_SYNAPSE_POLYGON_STATUS syn_stat = syn0->get_status();
    if(syn_stat!= syn1->get_status() ) return IVP_FALSE;

    switch(syn_stat)
    {
        case IVP_ST_POINT: {
	    //check for ledge and start point index to identify a point
	    const IVP_Compact_Ledge *l0 = syn0->edge->get_compact_ledge();
	    const IVP_Compact_Ledge *l1 = syn1->edge->get_compact_ledge();
	    if (l0 != l1) return IVP_FALSE;
	    
	    if( syn0->edge->get_start_point_index() != syn1->edge->get_start_point_index() ){
		return IVP_FALSE;
	    }
	    return IVP_TRUE;
	}
        case IVP_ST_TRIANGLE: {
	    //immer eindeutig
	    if( syn0->edge->get_triangle() == syn1->edge->get_triangle() )
	    {
		return IVP_TRUE;
	    } else {
		return IVP_FALSE;
	    }
	}
    case IVP_ST_BALL: 
	// objects are already checked !!
	return IVP_TRUE;
    case IVP_ST_EDGE: {
	    //do not check side
	    // note by OS: check for tmp.common would work too: but discussion for concav objects needed
	if( syn0->edge == syn1->edge || syn0->edge->get_opposite() == syn1->edge)
	{
	    return IVP_TRUE;
	} else {
	    return IVP_FALSE;
	}
    }	    
        default:
	    CORE;
    }
    return IVP_FALSE;
}

IVP_BOOL IVP_Contact_Point::is_same_as(const IVP_Mindist *md2) const
{
    const IVP_Synapse_Friction *sy00 = this->get_synapse(0);
    const IVP_Synapse_Friction *sy01 = this->get_synapse(1);
    const IVP_Synapse_Real *sy10 =  md2->get_synapse(0);
    const IVP_Synapse_Real *sy11 =  md2->get_synapse(1);

    // check objects first
    if ( sy00->l_obj == sy10->l_obj && sy01->l_obj == sy11->l_obj) {
	if (sy00->is_same_as(sy10) && sy01->is_same_as(sy11)){
	    return IVP_TRUE;
	}
    }
    if ( sy01->l_obj == sy10->l_obj && sy00->l_obj == sy11->l_obj) {
	if (sy01->is_same_as(sy10) && sy00->is_same_as(sy11)){
	    return IVP_TRUE;
	}
    }
    return IVP_FALSE;
}

void IVP_Friction_Sys_Static::core_is_going_to_be_deleted_event(IVP_Core *del_core) {
    IVP_ASSERT(!del_core->physical_unmoveable);
    IVP_ASSERT(del_core->environment->state==IVP_ES_AT);

    IVP_Friction_Core_Pair *my_pair;
    int j;
    for(j=l_friction_system->fr_pairs_of_objs.len()-1;j>=0;j--) {
        my_pair=l_friction_system->fr_pairs_of_objs.element_at(j);
	if((my_pair->objs[0]==del_core)||(my_pair->objs[1]==del_core)) {
	    int k;
	    for(k=my_pair->fr_dists.len()-1;k>=0;k--) {
	        IVP_Contact_Point *fr_mindist=my_pair->fr_dists.element_at(k);
		this->l_friction_system->delete_friction_distance(fr_mindist);
	    }
	    //my_pair is now deleted!!!
	}
    }
    
    IVP_Friction_System *fs=this->l_friction_system;
    if(fs->friction_dist_number==0) {
      P_DELETE(fs);
    }
}
