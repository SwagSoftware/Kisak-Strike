// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#include <ivp_hull_manager.hxx>

#include <ivp_mindist_intern.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_compact_surface.hxx>


#include <ivp_phantom.hxx>
#include <ivp_range_manager.hxx>

void IVP_Mindist_Recursive::mindist_rescue_push(){
    // will be solved later
}

void IVP_Mindist_Recursive::do_impact(){
    IVP_Environment *env = get_environment();

    // check for terminal ledges
    IVP_Synapse_Real *syn0 = this->get_synapse(0);
    IVP_Synapse_Real *syn1 = this->get_synapse(1);

//	ivp_message("0x%x::do_impact with %d spawned mindists\n", this, this->get_spawned_mindist_count());

	if(this->get_spawned_mindist_count() > ivp_mindist_settings.max_spawned_mindist_count)
		goto do_impact;

    IVP_ASSERT( mindist_status == IVP_MD_EXACT);

    switch ( syn0->get_status()){
    case IVP_ST_BALL:
    case IVP_ST_POINT:
	switch ( syn1->get_status()){
	case IVP_ST_BALL:
	case IVP_ST_POINT:
	    goto do_impact;
	case IVP_ST_EDGE:
	    if ( !syn1->get_edge()->get_is_virtual() ) goto do_impact;
	    recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
	    break;
	case IVP_ST_TRIANGLE:
	    if ( !syn1->get_edge()->get_triangle()->get_is_virtual() ) goto do_impact;
	    recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
	    break;
	default: CORE;
	}
	break;
	
    case IVP_ST_EDGE:
	switch ( syn1->get_status()){
	case IVP_ST_BALL:
	case IVP_ST_POINT:
	    if ( !syn0->get_edge()->get_triangle()->get_is_virtual() ) goto do_impact;
	    recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
	    break;
	case IVP_ST_EDGE:
	    if ( !syn1->get_edge()->get_is_virtual() ){
		if ( !syn0->get_edge()->get_is_virtual() ){
		    goto do_impact;
		}
		recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
		break;
	    }
	    if ( !syn0->get_edge()->get_is_virtual() ){
		recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
		break;
	    }
	    // now check details
	    {
		// search smaller radius	    
		const IVP_Compact_Ledge *l0 = syn0->get_edge()->get_triangle()->get_compact_ledge();
		const IVP_Compact_Ledge *l1 = syn1->get_edge()->get_triangle()->get_compact_ledge();

		const IVP_Compact_Ledgetree_Node *n0 = l0->get_ledgetree_node();
		const IVP_Compact_Ledgetree_Node *n1 = l1->get_ledgetree_node();
		//lwss hack - add null checks here :(
		if( !n0 && !n1 )
        {
            // if they are both null just do normal I guess
            // TODO: what is normal? this is not used really, do we need one of the recursive ones instead?
            recursive_status = IVP_MR_NORMAL;
        }
		else if( n0 && n1 )
        {
		    // ORIGINAL CHECKS
            if ( n0->radius > n1->radius){
                recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
            }else{
                recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
            }
        }
		else
        {
            // if the 1st one is null, then we can assume the 2nd one is bigger?
            if( !n0 )
                recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
            // if the 2nd one is null, we can assume first one is bigger.
            if( !n1 )
                recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
		}
		//lwss end
	    }
	    break;
	default: CORE;
	}
	break;
    case IVP_ST_TRIANGLE:
	if ( !syn0->get_edge()->get_triangle()->get_is_virtual() ) goto do_impact;
	recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
	break;
    default:
	CORE;
    }
    {
	IVP_Mindist_Manager *mm = env->get_mindist_manager();
	mm->remove_exact_mindist(this);
	//mindists.ensure_capacity(16);

	IVP_DOUBLE dist0,dist1;
	env->get_range_manager()->get_coll_range_intra_objects(syn0->get_object(),syn1->get_object(),&dist0,&dist1);
	mm->insert_lazy_hull_mindist( this, dist0 + dist1 );
	this->mindist_status = IVP_MD_HULL_RECURSIVE;
	this->recheck_recursive_childs(dist0 + dist1);
	return;
    }
 do_impact:
    {
	IVP_Mindist::do_impact();
	return;
    };

}


void IVP_Mindist_Recursive::exact_mindist_went_invalid(IVP_Mindist_Manager *mm){
    IVP_Synapse_Real *syn0 = this->get_synapse(0);
    IVP_Synapse_Real *syn1 = this->get_synapse(1);


    if(this->get_spawned_mindist_count() > ivp_mindist_settings.max_spawned_mindist_count){
        mm->remove_exact_mindist(this);
	mm->insert_invalid_mindist(this);
	return;
    }


    // search smaller radius	    
    const IVP_Compact_Ledge *l0 = syn0->get_edge()->get_compact_ledge();
    const IVP_Compact_Ledge *l1 = syn1->get_edge()->get_compact_ledge();

    if ( l0->is_terminal() ){
	recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
    }else if ( l1->is_terminal() ){
	recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
    }else {
        const IVP_Compact_Ledgetree_Node *n0 = l0->get_ledgetree_node();
        const IVP_Compact_Ledgetree_Node *n1 = l1->get_ledgetree_node();
        //lwss hack - add null checks here :(
        if (!n0 && !n1) {
            // if they are both null just do normal I guess
            // TODO: what is normal? this is not used really, do we need one of the recursive ones instead?
            recursive_status = IVP_MR_NORMAL;
        } else if (n0 && n1) {
            // ORIGINAL CHECKS
            if (n0->radius > n1->radius) {
                recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
            } else {
                recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
            }
        } else {
            // if the 1st one is null, then we can assume the 2nd one is bigger?
            if (!n0)
                recursive_status = IVP_MR_SECOND_SYNAPSE_RECURSIVE;
            // if the 2nd one is null, we can assume first one is bigger.
            if (!n1)
                recursive_status = IVP_MR_FIRST_SYNAPSE_RECURSIVE;
        }
        //lwss end
    }
    mm->remove_exact_mindist(this);
    //mindists.ensure_capacity(16);

    IVP_DOUBLE dist0,dist1;
    IVP_Environment *env = syn0->get_object()->get_environment();
    env->get_range_manager()->get_coll_range_intra_objects( syn0->get_object(), syn1->get_object(),&dist0,&dist1);
    mm->insert_lazy_hull_mindist( this, dist0 + dist1 );

	this->mindist_status = IVP_MD_HULL_RECURSIVE;

	this->recheck_recursive_childs(dist0 + dist1);
}


void IVP_Mindist_Recursive::delete_all_children(){
	int l = mindists.len(); //@@CB
    for (int i = mindists.len()-1; i>=0;i--){
	IVP_Collision *ma = mindists.element_at(i);
	P_DELETE(ma);
    }
	this->change_spawned_mindist_count(-l); //@@CB
//	ivp_message("0x%x has %d total spawned mindists\n", this, this->get_spawned_mindist_count());//@@CB
    mindists.clear();
}

IVP_Mindist_Recursive::IVP_Mindist_Recursive(IVP_Environment *env, IVP_Collision_Delegator *del): 
  IVP_Mindist(env,del),mindists(0),spawned_mindist_count(0){
    recursive_status = IVP_MR_NORMAL;
}


IVP_Mindist_Recursive::~IVP_Mindist_Recursive(){
    // synapses will be removed by ~IVP_Mindist
    delete_all_children();
}

void IVP_Mindist_Recursive::collision_is_going_to_be_deleted_event(IVP_Collision *c){
    mindists.remove_allow_resort(c);
}

void IVP_Mindist_Recursive::recheck_recursive_childs(IVP_DOUBLE dist_intra){
    IVP_Real_Object *obj0 = get_synapse(0)->get_object();
    IVP_Real_Object *obj1 = get_synapse(1)->get_object();    

    IVP_Environment *env = obj0->get_environment();

	if(this->get_spawned_mindist_count() > ivp_mindist_settings.max_spawned_mindist_count)//@@CB
		return;

	const IVP_Compact_Ledge *sl[2] = { NULL, NULL }; // single reference ledge
    const IVP_Compact_Ledge *rl[2] = { NULL, NULL }; // single root ledge
    sl[1-recursive_status] = get_synapse(1-recursive_status)->get_edge()->get_triangle()->get_compact_ledge();
    rl[recursive_status] = get_synapse(recursive_status)->get_edge()->get_triangle()->get_compact_ledge();

    IVP_Mindist_Manager *mm = env->get_mindist_manager();

	int l = mindists.len(); //@@CB
    mm->create_exact_mindists(obj0,obj1,  dist_intra, &mindists, sl[0], sl[1], rl[0], rl[1], this);
	l = mindists.len() - l; //@@CB
	this->change_spawned_mindist_count(l); //@@CB
}


void IVP_Mindist_Recursive::invalid_mindist_went_exact(){
    delete_all_children();

    IVP_Real_Object *obj0 = get_synapse(0)->get_object();
    //IVP_Real_Object *obj1 = get_synapse(1)->get_object();    

    IVP_Environment *env = obj0->get_environment();

    IVP_Mindist_Manager *mm = env->get_mindist_manager();

    mm->remove_hull_mindist(this);
    mm->insert_exact_mindist( this );

    mindist_function = IVP_MF_COLLISION;
}

void IVP_Mindist_Recursive::rec_hull_limit_exceeded_event(){
  this->recalc_invalid_mindist();
    if (this->recalc_result == IVP_MDRR_OK){
      if (this->get_length() > ivp_mindist_settings.friction_dist){
	invalid_mindist_went_exact();
	return;
      }
    }
    
    IVP_Real_Object *obj0 = get_synapse(0)->get_object();
    IVP_Real_Object *obj1 = get_synapse(1)->get_object();    
    // build all mindists
    IVP_DOUBLE dist0,dist1;
    IVP_Environment *env = obj0->get_environment();
    env->get_range_manager()->get_coll_range_intra_objects(obj0,obj1,&dist0,&dist1);

    env->get_statistic_manager()->range_intra_exceeded++;

    this->recheck_recursive_childs(dist0 + dist1);

    IVP_Time t_now = env->get_current_time();
    obj0->get_hull_manager()->update_synapse( get_synapse(0), t_now, dist0);
    obj1->get_hull_manager()->update_synapse( get_synapse(1), t_now, dist1);    
}

//@@CB
void IVP_Mindist_Recursive::change_spawned_mindist_count(int change)
{
	this->spawned_mindist_count += change;
	this->delegator->change_spawned_mindist_count(change);
}

int IVP_Mindist_Recursive::get_spawned_mindist_count()
{
	if(this->delegator->get_spawned_mindist_count() > 0)
		return this->delegator->get_spawned_mindist_count();
	else
		return this->spawned_mindist_count;
}
//@@CB