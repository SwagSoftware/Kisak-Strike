// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivp_cache_object.hxx"
#endif
#ifndef WIN32
#	pragma implementation "ivp_cache_ledge_point.hxx"
#endif

#include <ivp_physics.hxx>
#include <ivu_matrix_macros.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>
#include <ivp_core_macros.hxx>



void IVP_Cache_Object_Manager::invalid_cache_object(IVP_Real_Object *object)
{
    if (object->cache_object){
	IVP_ASSERT(object->cache_object->reference_count==0);
	IVP_ASSERT(object->cache_object->object==object);

	object->cache_object->object = NULL;
	object->cache_object = NULL;
    }
}

IVP_Cache_Object *IVP_Cache_Object_Manager::get_cache_object(IVP_Real_Object *object)
{
    IVP_Cache_Object *co;
    while ( (co = cache_object_at(reuse_loop_index))->reference_count){
	reuse_loop_index = (reuse_loop_index+1) & (n_cache_objects -1);
    }
    reuse_loop_index = (reuse_loop_index+1) & (n_cache_objects -1);
    IVP_Real_Object *obj = co->object;
    if (obj){
	invalid_cache_object(obj);	// invalid old caches for object
    }
    co->object = object;
    object->cache_object = co;

    co->valid_until_time_code = 0;	// invalid cache for moving objects
    // and set cache for not simulated objects
    if ( !IVP_MTIS_SIMULATED(object->flags.object_movement_state)){
	    co->update_cache_object();
    }

    return co;
}

#ifdef DEBUG
void IVP_Cache_Object_Manager::check()
{
    IVP_ASSERT( reuse_loop_index <= n_cache_objects);
    for (int i=0;i<n_cache_objects;i++){
		IVP_Cache_Object *co = cache_object_at(i);
		IVP_ASSERT( co->reference_count == 0);
		IVP_IF (co->object){
			IVP_ASSERT( co->object->cache_object == co);
		}
    }
}
#endif

IVP_Cache_Object_Manager::IVP_Cache_Object_Manager(int number_of_cache_elements)
{
    this->n_cache_objects = number_of_cache_elements;
	unsigned int size = sizeof(IVP_Cache_Object);
	size = ( size + 0xf) & ~0xf;	// align 16
    this->cache_objects_buffer = (char *)p_calloc(size, number_of_cache_elements);
    this->reuse_loop_index = 0;
}

IVP_Cache_Object_Manager::~IVP_Cache_Object_Manager()
{
    int i;
    for (i=0;i<this->n_cache_objects; i++){
	IVP_Cache_Object *co = cache_object_at(i);
 	if (co->object){
	    this->invalid_cache_object(co->object);
	}
    }
    P_FREE(this->cache_objects_buffer);
}

void IVP_Cache_Object::update_cache_object(){
    IVP_Environment *env = object->get_environment();

    IVP_Time current_time = env->get_current_time();
    valid_until_time_code = env->get_current_time_code();
    
    IVP_Core *core = object->get_core();
    if ( current_time - core->time_of_last_psi == 0.0f){
	q_world_f_object = core->q_world_f_core_last_psi;
	m_world_f_object.vv.set(&core->pos_world_f_core_last_psi);
	core_pos.set(&core->pos_world_f_core_last_psi);
    }else{
	object->get_core()->inline_calc_at_quaternion(current_time, &q_world_f_object);
	object->get_core()->inline_calc_at_position(current_time, &m_world_f_object.vv);
	core_pos.set(&m_world_f_object.vv);
    }
    q_world_f_object.set_matrix( & m_world_f_object );

    if (!object->flags.shift_core_f_object_is_zero){
	m_world_f_object.vmult4( &object->shift_core_f_object , &m_world_f_object.vv);
    }
    
    if ( object->q_core_f_object){
	q_world_f_object.inline_set_mult_quat( &q_world_f_object, object->q_core_f_object );
	q_world_f_object.set_matrix( &m_world_f_object );
    }
}

void IVP_Cache_Object::transform_position_to_object_coords(const IVP_U_Point *P_world, IVP_U_Point *P_object_out) const
{
    this->m_world_f_object.inline_vimult4(P_world, P_object_out);
}

void IVP_Cache_Object::transform_position_to_object_coords(const IVP_U_Point *P_world, IVP_U_Float_Point *P_object_out) const
{
    this->m_world_f_object.inline_vimult4(P_world, P_object_out);
}

void IVP_Cache_Object::transform_position_to_world_coords(const IVP_U_Float_Point *P_object, IVP_U_Point *P_world_out) const {
    this->m_world_f_object.inline_vmult4(P_object, P_world_out);
}


void IVP_Cache_Object::transform_vector_to_object_coords(const IVP_U_Point *P_world, IVP_U_Point *P_object_out) const
{
    this->m_world_f_object.inline_vimult3(P_world, P_object_out);
}

void IVP_Cache_Object::transform_vector_to_object_coords(const IVP_U_Float_Point *P_world, IVP_U_Float_Point *P_object_out) const
{
    this->m_world_f_object.inline_vimult3(P_world, P_object_out);
}

void IVP_Cache_Object::transform_vector_to_world_coords(const IVP_U_Point *P_object, IVP_U_Point *P_world_out) const
{
    this->m_world_f_object.inline_vmult3(P_object, P_world_out);
}

void IVP_Cache_Object::transform_vector_to_world_coords(const IVP_U_Float_Point *P_object, IVP_U_Float_Point *P_world_out) const
{
    this->m_world_f_object.inline_vmult3(P_object, P_world_out);
}





























