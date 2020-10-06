// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.




void IVP_Synapse_Real::remove_exact_synapse_from_object(){
    if (next) next->prev = prev;
    if (prev){
	prev->next = next;
    }else{
	IVP_ASSERT( l_obj->exact_synapses == this );
	l_obj->exact_synapses = get_next();
    }
    IVP_IF(1){
      next = prev = this;
    }
}

void IVP_Synapse_Real::insert_exact_synapse_in_object(){
  IVP_ASSERT( next == this);
  IVP_ASSERT( prev == this);
    this->next = l_obj->exact_synapses;
    if (this->next){
	this->next->prev = this;
    }
    this->prev = NULL;
    l_obj->exact_synapses = this;
}

void IVP_Synapse_Real::remove_invalid_synapse_from_object(){
    if (next) next->prev = prev;
    if (prev){
	prev->next = next;
    }else{
	IVP_ASSERT( l_obj->invalid_synapses == this );
	l_obj->invalid_synapses = get_next();
    }
    IVP_IF(1){
      next = prev = this;
    }
}

void IVP_Synapse_Real::insert_invalid_synapse_in_object(){
    this->next = l_obj->invalid_synapses;
    if (this->next){
	this->next->prev = this;
    }
    this->prev = NULL;
    l_obj->invalid_synapses = this;
}




IVP_DOUBLE IVP_Synapse_Real::insert_in_hull_manager(IVP_DOUBLE rel_hull_time)
{
    IVP_Time time = l_obj->get_environment()->get_current_time();
    return l_obj->to_real()->get_hull_manager()->insert_synapse(this, time, rel_hull_time);
}

IVP_DOUBLE IVP_Synapse_Real::insert_lazy_in_hull_manager(IVP_DOUBLE rel_hull_time)
{
    IVP_Time time = l_obj->get_environment()->get_current_time();
    return l_obj->to_real()->get_hull_manager()->insert_lazy_synapse(this, time, rel_hull_time);
}
