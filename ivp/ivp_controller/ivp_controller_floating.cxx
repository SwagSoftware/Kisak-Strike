// Copyright (C) 2000 Ipion Software GmbH. All rights reserved.

#include <ivp_physics.hxx>
#ifndef WIN32
#	 pragma implementation "ivp_controller_floating.hxx"
#endif
#include <ivp_cache_object.hxx>
#include <ivp_solver_core_reaction.hxx>
#include <ivp_controller_floating.hxx>

IVP_Template_Controller_Floating::IVP_Template_Controller_Floating()
{
    P_MEM_CLEAR(this);
    max_repulsive_force = P_FLOAT_MAX;
    max_adhesive_force = P_FLOAT_MAX;
    ray_direction_ws.set(0.0f, 1.0f, 0.0f);
}


void IVP_Template_Controller_Floating::set_position_ws( IVP_Real_Object *obj, const IVP_U_Point *pos_ws){
    IVP_Cache_Object *co = obj->get_cache_object_no_lock();
    co->transform_position_to_object_coords( pos_ws, &position_os );
}

void IVP_Template_Controller_Floating::set_ray_direction_ws( IVP_Real_Object *, const IVP_U_Point *dir_ws){
    //IVP_Cache_Object *co = obj->get_cache_object();
    //co->remove_reference();
    //co->transform_vector_to_object_coords( dir_ws, &ray_direction_os );
    ray_direction_ws.set(dir_ws);
}


IVP_Controller_Floating::IVP_Controller_Floating(IVP_Real_Object *obj, const IVP_Template_Controller_Floating *templ)
{
    IVP_Environment *env = obj->get_environment();
    env->get_controller_manager()->add_controller_to_core(this,obj->get_core());
    this->object = obj;

    this->max_repulsive_force = templ->max_repulsive_force;
    this->max_adhesive_force = templ->max_adhesive_force;

    this->position_os.set(&templ->position_os);
    this->ray_direction_ws.set(&templ->ray_direction_ws);
    
    this->target_distance = templ->target_distance;
    this->current_distance = templ->current_distance;
}

void IVP_Controller_Floating::set_current_distance(IVP_DOUBLE cd){ current_distance = cd; };
void IVP_Controller_Floating::set_target_distance(IVP_DOUBLE td){ target_distance = td; };

void IVP_Controller_Floating::set_ray_direction_ws( const IVP_U_Float_Point *new_dir ){
    ray_direction_ws.set(new_dir);
}

const IVP_U_Float_Point *IVP_Controller_Floating::get_ray_direction_ws(){
    return &ray_direction_ws;
}


void IVP_Controller_Floating::do_simulation_controller(IVP_Event_Sim *es, IVP_U_Vector<IVP_Core> *) {
    // ------------------------------------------------------------------------
    // Try to push object along the specified ray_direction
    // so that it reaches specified delta distance within d_time (es->delta_time)
    //
    // Note: This function is called during PSI
    // ------------------------------------------------------------------------
    // ------------------------------------------------------------------------
    // current situation
    // ------------------------------------------------------------------------
    if (this->do_ray_casting(es) == IVP_FAULT){
	return;
    }
    
    class IVP_Cache_Object *co = object->get_cache_object();
    IVP_U_Point position_ws;
    co->transform_position_to_world_coords(&position_os, &position_ws);

    IVP_Solver_Core_Reaction tcb;
    tcb.init_reaction_solver_translation_ws( object->get_core(), NULL, position_ws,  &ray_direction_ws, NULL, NULL);
    IVP_DOUBLE a = (current_distance - target_distance) * es->i_delta_time - tcb.delta_velocity_ds.k[0];
    IVP_DOUBLE impulse = a / tcb.get_m_velocity_ds_f_impulse_ds()->get_elem(0,0);
    if ( impulse * es->i_delta_time > max_adhesive_force){
	impulse = max_adhesive_force * es->delta_time;
    }
    if ( -impulse * es->i_delta_time > max_repulsive_force){
	impulse = -max_repulsive_force * es->delta_time;
    }
    co->remove_reference();
    
    IVP_U_Float_Point impulse_ds( impulse, 0,0);
    tcb.exert_impulse_dim1(object->get_core(),NULL, impulse_ds);
}

void IVP_Controller_Floating::core_is_going_to_be_deleted_event(IVP_Core *core_i)
{
    IVP_ASSERT(object->get_core() == core_i);
    P_DELETE_THIS(this);
}
    
IVP_Controller_Floating::~IVP_Controller_Floating(){
    object->get_environment()->get_controller_manager()->remove_controller_from_core(this, object->get_core());
}
