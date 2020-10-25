// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

// Note:
// freeze object must update    update_cache_object + set cache_ledge_point->valid_until_time_code
// delete object: must call invalid_cache_object   

#ifndef _IVP_CACHE_OBJECT_INCLUDED
#define _IVP_CACHE_OBJECT_INCLUDED

#ifndef WIN32
#	pragma interface
#endif


class IVP_Cache_Object {
    friend class IVP_Real_Object;
    friend class IVP_Cache_Object_Manager;
    friend class IVP_U_Matrix_Cache;
    
    IVP_Time_CODE	valid_until_time_code;	// compared to environment->get_current_time_code() !
    int			reference_count;	// number of references to this
    IVP_Real_Object	*object; // backlink to object
public:
    void remove_reference()
	{     
		IVP_ASSERT(reference_count == 1 );
		reference_count--; 
	};
    IVP_U_Quat		q_world_f_object;
    IVP_U_Matrix	m_world_f_object;
    IVP_U_Point         core_pos;
    
    void update_cache_object();

    void transform_vector_to_world_coords(const IVP_U_Point *P_object, IVP_U_Point *P_world_out) const;
    void transform_vector_to_world_coords(const IVP_U_Float_Point *P_object, IVP_U_Float_Point *P_world_out) const;
    void transform_vector_to_object_coords(const IVP_U_Point *P_world, IVP_U_Point *P_object_out) const;
    void transform_vector_to_object_coords(const IVP_U_Float_Point *P_world, IVP_U_Float_Point *P_object_out) const;

    void transform_position_to_world_coords(const IVP_U_Float_Point *P_object, IVP_U_Point *P_world_out) const;
    void transform_position_to_object_coords(const IVP_U_Point *P_world, IVP_U_Float_Point *P_object_out) const;
    void transform_position_to_object_coords(const IVP_U_Point *P_world, IVP_U_Point *P_object_out) const;
};

class IVP_Cache_Object_Manager {
    int n_cache_objects;
    int reuse_loop_index;	// rotates through the array
    char *cache_objects_buffer; // IVP_Cache_Object
public:
    static void invalid_cache_object(IVP_Real_Object *object);
	inline IVP_Cache_Object *cache_object_at(int i){
		const int size = (sizeof (IVP_Cache_Object ) + 0xf) & ~0xf;
		return  (IVP_Cache_Object *) (cache_objects_buffer + size * i);
	}
    
    IVP_Cache_Object *get_cache_object(IVP_Real_Object *object);
#ifdef DEBUG    
    void check();	// check when no locks are set
#endif    
    IVP_Cache_Object_Manager(int number_of_cache_elements);	// number_of_cache_elements is 2**x
    ~IVP_Cache_Object_Manager();
};

IVP_Cache_Object *IVP_Real_Object::get_cache_object(){
    // get cache from object manager

    if (!this->cache_object ){
	IVP_Environment *env = get_environment();
	this->cache_object = env->get_cache_object_manager()->get_cache_object(this);
    }
    this->cache_object->reference_count++;
    
    if ( IVP_MTIS_SIMULATED(this->flags.object_movement_state) ){
	IVP_Environment *env = get_environment();
	//lwss hack - ALERT! ALERT! this was changed from > to >= so that it would update on 0
	// It seems that the cache object is not always initialized and all the world coord math returns NaN in p_minimize_FF()
	// this is not a proper solution, but will at least let us load into maps like militia/train where it seems in-map physics objects like swinging signs trigger this code.
	// I've done a benchmark with this code and it seemed fine, but it was not a particularly extensive physics test!!
	//if ( env->get_current_time_code() > cache_object->valid_until_time_code){
	if ( env->get_current_time_code() >= cache_object->valid_until_time_code){
    //lwss end
	    cache_object->update_cache_object();
	}
    }
    return cache_object;
}

IVP_Cache_Object *IVP_Real_Object::get_cache_object_no_lock(){
    // get cache from object manager

    if (!this->cache_object ){
	IVP_Environment *env = get_environment();
	this->cache_object = env->get_cache_object_manager()->get_cache_object(this);
    }
    
    if ( IVP_MTIS_SIMULATED(this->flags.object_movement_state) ){
	IVP_Environment *env = get_environment();
	if ( env->get_current_time_code() > cache_object->valid_until_time_code){
	    cache_object->update_cache_object();
	}
    }
    return cache_object;
}

#endif


