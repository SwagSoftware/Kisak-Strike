// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>


#ifndef WIN32
#	 pragma implementation "ivp_phantom.hxx"
#endif
#include <ivp_phantom.hxx>

#include <ivp_mindist_intern.hxx>
#include <ivp_hull_manager.hxx>
#include <ivu_min_hash.hxx>
#include <cstdint> //lwss- x64 fixes

IVP_Template_Phantom::IVP_Template_Phantom(){
	P_MEM_CLEAR(this);
	exit_policy_extra_radius = 0.5f;
	exit_policy_extra_time = 0.5f;
}

void IVP_Controller_Phantom::add_listener_phantom( IVP_Listener_Phantom *listener){
	listeners.add(listener);
}

void IVP_Controller_Phantom::remove_listener_phantom( IVP_Listener_Phantom *listener){
	listeners.remove(listener);
}

IVP_Controller_Phantom::~IVP_Controller_Phantom(){
	for (int i = listeners.len()-1;i>=0;i--){
		IVP_Listener_Phantom *l = listeners.element_at(i);
		l->phantom_is_going_to_be_deleted_event(this);
	}
	object->controller_phantom = 0;
	P_DELETE(mindist_core_counter);
	P_DELETE(mindist_object_counter);
	
	P_DELETE(set_of_objects);
	P_DELETE(set_of_cores);
}

void IVP_Controller_Phantom::mindist_entered_volume(class IVP_Mindist *mindist)
{
	set_of_mindists.add_element(mindist);
	
	if (set_of_objects)
	{
		IVP_Real_Object *other_object;
		if (mindist->get_synapse(0)->get_object() == object)
		{
			other_object = mindist->get_synapse(1)->get_object();
		}
		else
		{
			other_object = mindist->get_synapse(0)->get_object();
		}
		//lwss - x64 fixes
		//int x = (int)mindist_object_counter->find_elem( other_object );
		intptr_t x = (intptr_t)mindist_object_counter->find_elem( other_object );
		//lwss end
		if (x)
		{
			mindist_object_counter->change_elem( other_object, (void *)(x+1));
		}
		else
		{
			mindist_object_counter->add_elem( other_object, (void *)1);
			set_of_objects->add_element(other_object);
		}
	}
	if (set_of_cores)
	{
		IVP_Real_Object *other_object;
		if (mindist->get_synapse(0)->get_object() == object)
		{
			other_object = mindist->get_synapse(1)->get_object();
		}
		else
		{
			other_object = mindist->get_synapse(0)->get_object();
		}
		IVP_Core *other_core= other_object->get_core();
		//lwss -x64 fixes
		//int x = (int)mindist_core_counter->find_elem( other_core );
		intptr_t x = (intptr_t)mindist_core_counter->find_elem( other_core );
		//lwss end
		if (x)
		{
			mindist_core_counter->change_elem( other_core, (void *)(x+1));
		}
		else
		{
			mindist_core_counter->add_elem( other_core, (void *)1);
			set_of_cores->add_element(other_core);
			for (int i = listeners.len()-1;i>=0;i--)
			{
				IVP_Listener_Phantom *l = listeners.element_at(i);
				l->core_entered_volume(this,other_core);
			}
		}
	}
	
	
	for (int i = listeners.len()-1;i>=0;i--)
	{
		IVP_Listener_Phantom *l = listeners.element_at(i);
		l->mindist_entered_volume(this,mindist);
	}
}

void IVP_Controller_Phantom:: mindist_left_volume(class IVP_Mindist *mindist)
{
	set_of_mindists.remove_element(mindist);
	if (set_of_objects)
	{
        IVP_Real_Object *other_object;
		if (mindist->get_synapse(0)->get_object() == object){
			other_object = mindist->get_synapse(1)->get_object();
		}
		else
		{
			other_object = mindist->get_synapse(0)->get_object();
		}
		//lwss -x64 fixes
		//int x = (int)mindist_object_counter->find_elem( other_object );
		intptr_t x = (intptr_t)mindist_object_counter->find_elem( other_object );
		//lwss end
		if (x>1)
		{
			mindist_object_counter->change_elem( other_object, (void *)(x-1));
		}
		else
		{
			mindist_object_counter->remove_elem( other_object);
			set_of_objects->remove_element(other_object);
		}
	}
	if (set_of_cores)
	{
        IVP_Real_Object *other_object;
		if (mindist->get_synapse(0)->get_object() == object)
		{
			other_object = mindist->get_synapse(1)->get_object();
		}
		else
		{
			other_object = mindist->get_synapse(0)->get_object();
		}
		IVP_Core *other_core= other_object->get_core();
		//lwss -x64 fixes
		//int x = (int)mindist_core_counter->find_elem( other_core );
		intptr_t x = (intptr_t)mindist_core_counter->find_elem( other_core );
		//lwss end
		if (x>1)
		{
			mindist_core_counter->change_elem( other_core, (void *)(x-1));
		}
		else
		{
			mindist_core_counter->remove_elem( other_core);
			set_of_cores->remove_element(other_core);
			for (int i = listeners.len()-1;i>=0;i--)
			{
				IVP_Listener_Phantom *l = listeners.element_at(i);
				l->core_left_volume(this,other_core);
			}
		}
	}
	
	for (int i = listeners.len()-1;i>=0;i--)
	{
		IVP_Listener_Phantom *l = listeners.element_at(i);
		l->mindist_left_volume(this,mindist);
	}
}

IVP_Controller_Phantom::IVP_Controller_Phantom(IVP_Real_Object *object_in, const IVP_Template_Phantom *templat):
set_of_mindists(16)
{
	object = object_in;
	if (templat->manage_intruding_objects)
	{
		set_of_objects = new IVP_U_Set_Active<IVP_Real_Object>(16);
		mindist_object_counter = new IVP_VHash_Store(16);
	} 
	else 
	{
		set_of_objects = NULL;
		mindist_object_counter = NULL;
	}
	
	if (templat->manage_intruding_cores)
	{
		set_of_cores = new IVP_U_Set_Active<IVP_Core>(16);
		mindist_core_counter = new IVP_VHash_Store(16);

		//lwss
		if( templat->manage_sleeping_cores )
        {
		    //TODO: hello friend this is unimplemented, looks like another IVP_U_SetActive(16) goes here.
        }
		//lwss end
	} 
	else 
	{
		set_of_cores = NULL;
		mindist_core_counter = NULL;
	}
	
	
	if (templat->dont_check_for_unmoveables)
	{
		// don�t change anything here
		
	}
	else
	{
		if (object->get_movement_state() == IVP_MT_STATIC)
		{
			object->set_movement_state(IVP_MT_STATIC_PHANTOM);
			object->get_environment()->get_mindist_manager()->recheck_ov_element(object);
		}
	}
	
	exit_policy_extra_radius = templat->exit_policy_extra_radius;
	
	// loop through all invalid mindists and insert them into mindist set
	{
		IVP_Synapse_Real *syn;
		for (syn = object->invalid_synapses; syn; syn = syn->get_next())
		{
			IVP_Mindist *mindist = syn->get_mindist();
			mindist->mindist_function = IVP_MF_PHANTOM;
			
			mindist_entered_volume( mindist );
			mindist->is_in_phantom_set = IVP_TRUE;
		}
	}
	
	// loop through all exact mindists and mark them as phantom mindists
	{
		IVP_Synapse_Real *syn;
		for (syn = object->exact_synapses; syn; syn = syn->get_next())
		{
			IVP_Mindist *mindist = syn->get_mindist();
			mindist->mindist_function = IVP_MF_PHANTOM;
			
			if (mindist->get_length() < 0.0f)
			{
				mindist_entered_volume( mindist );
				mindist->is_in_phantom_set = IVP_TRUE;
			}
			else
			{
				mindist->is_in_phantom_set = IVP_FALSE;
			}
		}
	}
	
	// loop through all hull mindists and mark them as phantom mindists
	{
		IVP_Hull_Manager *hm = object->get_hull_manager();
		IVP_U_Min_List *ss = hm->get_sorted_synapses();
		
		IVP_U_Min_List_Enumerator mindists(ss);
		IVP_Listener_Hull *supers;
		while ( (supers = (IVP_Listener_Hull*)mindists.get_next_element()) != NULL )
		{
			if (supers->get_type() != IVP_HULL_ELEM_POLYGON) continue;
			IVP_Synapse_Real *syn = (IVP_Synapse_Real*)supers;
			
			
			IVP_Mindist *md = syn->get_mindist();
			if (md->mindist_function == IVP_MF_COLLISION)
			{
				md->mindist_function = IVP_MF_PHANTOM;
			}
		}
	}
	client_data = 0;
}

//lwss - add this missing func
void IVP_Controller_Phantom::wake_all_sleeping_objects()
{
    int size = this->set_of_objects->n_elems();
    for( int i = 0; i < size; i++ )
    {
        IVP_Real_Object *object = (IVP_Real_Object*)this->set_of_objects->element_at( i );
        if( !object )
            continue;
        object->ensure_in_simulation();
    }
    //TODO: there is another loop just like this but with another array, unsure which one..
}
//lwss end

