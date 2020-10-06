// Copyright (C) 2000 Ipion Software GmbH. All rights reserved.


#include <ivp_physics.hxx>
#ifndef WIN32
#	 pragma implementation "ivp_controller_world_frict.hxx"
#endif
#include <ivp_cache_object.hxx>
#include <ivp_solver_core_reaction.hxx>
#include <ivp_controller_world_frict.hxx>

IVP_Template_Controller_World_Friction::IVP_Template_Controller_World_Friction()
{
    desired_speed_ws.set_to_zero();
    desired_rot_speed_cs.set_to_zero();
    IVP_FLOAT friction_val = 0.4f * 9.81f; //m/s
    friction_value_translation.set(friction_val,friction_val,friction_val);
    friction_value_rotation.set(friction_val*0.3,friction_val*0.3,friction_val*0.3);
}


IVP_Controller_World_Friction::IVP_Controller_World_Friction(IVP_Real_Object *obj, const IVP_Template_Controller_World_Friction *templ)
{
    IVP_Environment *env = obj->get_environment();
    env->get_controller_manager()->add_controller_to_core(this,obj->get_core());
    this->real_obj = obj;

    this->desired_speed_ws.set( &templ->desired_speed_ws );
    this->desired_rot_speed_cs.set( &templ->desired_rot_speed_cs );

    this->friction_value_translation.set(&templ->friction_value_translation);
    this->friction_value_rotation.set(&templ->friction_value_rotation);
}

void IVP_Controller_World_Friction::do_simulation_controller(IVP_Event_Sim *es, IVP_U_Vector<IVP_Core> *) {
    IVP_Core *my_core=this->real_obj->get_core();

    //*****************
    //translation speed
    //*****************
    {
		IVP_U_Float_Point current_ws_speed;
		current_ws_speed.set(&my_core->speed);

		IVP_U_Float_Point correction_speed_ws;
		correction_speed_ws.subtract(&desired_speed_ws,&current_ws_speed);

		//transform world_speed in
		const IVP_U_Matrix *transform_mat;
		transform_mat=my_core->get_m_world_f_core_PSI();

		IVP_U_Float_Point correction_speed_cs;
		transform_mat->vimult3(&correction_speed_ws,&correction_speed_cs);

		IVP_U_Float_Point max_friction_correction;
		max_friction_correction.set_multiple(&friction_value_translation,es->delta_time);

		// for each axis clip speed
		for(int i=0;i<3;i++) 
		{
			// was fabs, which was a sml call
			if(IVP_Inline_Math::fabsd(correction_speed_cs.k[i]) > max_friction_correction.k[i]) 
			{
				int sign_val=1;

				if( correction_speed_cs.k[i] < 0.0 ) 
				{
					sign_val=-1;
				}
					
				correction_speed_cs.k[i]=max_friction_correction.k[i]*sign_val;
			}
		}

		// transform back
		transform_mat->vmult3( &correction_speed_cs, &correction_speed_ws );
		my_core->center_push_core_multiple_ws( &correction_speed_ws, my_core->get_mass()  ); //apply impulse
    }

    //******************
    //rotation speed
    //******************
	{
		IVP_U_Float_Point current_cs_rot_speed;
		current_cs_rot_speed.set(&my_core->rot_speed);

		IVP_U_Float_Point correction_rot_speed_cs;
		correction_rot_speed_cs.subtract(&desired_rot_speed_cs,&current_cs_rot_speed);

		IVP_U_Float_Point max_friction_correction;
		max_friction_correction.set_multiple(&friction_value_rotation,es->delta_time);

		// for each axis clip speed
		for(int i=0;i<3;i++) 
		{
			// was fabs, which was a sml call
			if(IVP_Inline_Math::fabsd(correction_rot_speed_cs.k[i]) > max_friction_correction.k[i]) {
				int sign_val=1;

				if( correction_rot_speed_cs.k[i] < 0.0 ) 
				{
					sign_val=-1;
				}

				correction_rot_speed_cs.k[i]=max_friction_correction.k[i]*sign_val;
			}
		}

		//calculate impulses
		IVP_U_Float_Point correction_rot_impulse;
		IVP_U_Float_Point my_rot_inertias;
		my_rot_inertias=my_core->get_rot_inertia();
		correction_rot_impulse.k[0] = correction_rot_speed_cs.k[0] * my_rot_inertias.k[0]; 
		correction_rot_impulse.k[1] = correction_rot_speed_cs.k[1] * my_rot_inertias.k[1]; 
		correction_rot_impulse.k[2] = correction_rot_speed_cs.k[2] * my_rot_inertias.k[2]; 

		my_core->rot_push_core_cs( &correction_rot_impulse );
	}
}

void IVP_Controller_World_Friction::core_is_going_to_be_deleted_event(IVP_Core *core_i)
{
    IVP_ASSERT(real_obj->get_core() == core_i);
    P_DELETE_THIS(this);
}
    
IVP_Controller_World_Friction::~IVP_Controller_World_Friction(){
    real_obj->get_environment()->get_controller_manager()->remove_controller_from_core(this, real_obj->get_core());
}
