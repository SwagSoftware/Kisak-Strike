// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PRIVATE

#include <ivp_physics.hxx>
#include <ivp_performancecounter.hxx>

#include <ivp_debug_manager.hxx>
#include <ivp_debug.hxx>

#ifndef WIN32
#pragma implementation "ivp_mindist.hxx"
#pragma implementation "ivp_mindist_intern.hxx"
#endif

#include <ivu_memory.hxx>
#include <ivp_hull_manager.hxx>

#include <ivp_mindist_intern.hxx>
#include <ivp_mindist_event.hxx>
#include <ivp_mindist_macros.hxx>

#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>
#include <ivp_compact_ledge_solver.hxx>
#include <ivp_compact_surface.hxx>

#include <ivp_core_macros.hxx>
#include <ivu_hash.hxx>
#include <ivu_diff_hash.hxx>
#include <ivp_time.hxx>

#include <ivp_phantom.hxx>
#include <ivp_collision_filter.hxx>
#include <ivp_clustering_longrange.hxx>
#include <ivp_clustering_visualizer.hxx>
#include <ivp_range_manager.hxx>
#include <ivp_universe_manager.hxx>
#include <ivp_surman_polygon.hxx>

//#include <../IVP_INTERN/ivp_friction.hxx> //for debugging
#include <ivp_friction.hxx> //for debugging @@CB

// #define MIN_DEBUG 1

// MINIMIZE distance from 2 convex bowls beginning with
// start position of 2 synapses.


IVP_Mindist_Settings ivp_mindist_settings;

void IVP_Mindist_Settings::set_collision_tolerance(IVP_DOUBLE t, IVP_DOUBLE gravLen){
    real_coll_dist =                         0.1f * t;

    //lwss - changes from debug bins
    //min_coll_dists = real_coll_dist        + 0.9f * t;
    min_coll_dists = t;
    //minimum_friction_dist = min_coll_dists + 0.0f * t;
    minimum_friction_dist = t;
    //lwss end

    for (int i = IVP_MAX_STEPS_FOR_COLLDIST_DECREASE-1; i >= 0 ; i-- ){
	coll_dists[i] = min_coll_dists + ( i / IVP_DOUBLE(IVP_MAX_STEPS_FOR_COLLDIST_DECREASE)) * ( minimum_friction_dist - min_coll_dists );
    }
    //lwss
    //friction_dist = minimum_friction_dist + 1.0f *t;
    friction_dist = minimum_friction_dist + t;
    //lwss end

    keeper_dist = friction_dist + (0.3f * t);

    //lwss
    //speed_after_keeper_dist = IVP_Inline_Math::ivp_sqrtf( 2.0f * (keeper_dist - min_coll_dists) * 9.81f ); //speed when falling down from keeper dist to coll dist
    speed_after_keeper_dist = IVP_Inline_Math::ivp_sqrtf( ((keeper_dist - t) + (keeper_dist - t)) * gravLen ); //speed when falling down from keeper dist to coll dist
    //lwss end
    distance_keepers_safety = 0.01f * t; 			//safety gap, when surpassed, mindist doesnt appear in complex
    max_distance_for_friction = friction_dist + 2.5f * t;
    max_distance_for_impact_system = friction_dist + 20.0f * t;

    min_vertical_speed_at_collision = 2.0f * t;	// meter /second

    mindist_change_force_dist = min_coll_dists * 0.1f;
}

void IVP_Mindist_Settings::set_event_queue_min_delta_time_base(IVP_DOUBLE base){
	event_queue_min_delta_time_base = base; //@@CB
}

IVP_Mindist_Settings::IVP_Mindist_Settings(){
    set_collision_tolerance( 0.01f );
	set_event_queue_min_delta_time_base(1.0f);
	max_spawned_mindist_count = 1000;
}



IVP_Mindist_Base::IVP_Mindist_Base(IVP_Collision_Delegator *del): IVP_Collision(del){
     coll_type = IVP_COLL_NONE;
     synapse_sort_flag = 0;
     recalc_result = IVP_MDRR_OK;
     is_in_phantom_set = IVP_FALSE;   // mindist is in phantom set_of_mindists already
     mindist_status = IVP_MD_UNINITIALIZED;
     mindist_function = IVP_MF_COLLISION;
     coll_dist_selector = IVP_MAX_STEPS_FOR_COLLDIST_DECREASE-1;
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED
     disable_halfspace_optimization = IVP_FALSE;
     sum_angular_hull_time = 0.0f;
#endif
     index = IVP_U_MINLIST_UNUSED;
}

IVP_Mindist::IVP_Mindist(IVP_Environment *my_env, IVP_Collision_Delegator *del): IVP_Mindist_Base(del){
  IVP_ASSERT(del);
    recalc_time_stamp = 0;
    last_visited_triangle=NULL;
    my_env->get_statistic_manager()->sum_of_mindists++;
    my_env->get_statistic_manager()->mindists_generated++;
}

IVP_Mindist *g_pCurrentMindist = NULL;
bool g_fDeferDeleteMindist = false;

/********************************************************************************
 *	Name:	    	~IVP_Mindist  	
 *	Description:	deletes mindist
 ********************************************************************************/
IVP_Mindist::~IVP_Mindist(){
#if defined(DEBUG)
	if ( g_pCurrentMindist==this)
	{
		IVP_Collision_Filter *coll_filter = get_synapse(0)->l_obj->get_environment()->get_collision_filter();
		coll_filter->check_objects_for_collision_detection( get_synapse(0)->l_obj, get_synapse(1)->l_obj );
	}
#endif
	IVP_ASSERT(g_pCurrentMindist!=this);
    IVP_Environment *l_environment = get_mindist_synapse(0)->get_object()->get_environment();
    l_environment->get_statistic_manager()->sum_of_mindists--;
    l_environment->get_statistic_manager()->mindists_deleted++;

    IVP_Mindist_Manager *mm = l_environment->get_mindist_manager();
    IVP_Mindist *md = (IVP_Mindist *)this;
    if (is_in_phantom_set){
      mm->mindist_left_phantom(md);
    }
    
    switch(mindist_status){
    case IVP_MD_EXACT: 
	mm->remove_exact_mindist(md);
	break;
    case IVP_MD_HULL:
    case IVP_MD_HULL_RECURSIVE:
	mm->remove_hull_mindist(md);      
	break;
    case IVP_MD_INVALID:
	mm->remove_invalid_mindist(md);
	break;
      
    case IVP_MD_UNINITIALIZED:
    break;
    }

    IVP_ASSERT (index == IVP_U_MINLIST_UNUSED);
    
    md->get_synapse(0)->get_object()->get_surface_manager()->remove_reference_to_ledge(md->get_synapse(0)->get_ledge());
    md->get_synapse(1)->get_object()->get_surface_manager()->remove_reference_to_ledge(md->get_synapse(1)->get_ledge());
    
    delegator->collision_is_going_to_be_deleted_event(this);
}


void IVP_Mindist::print(const char *)
{
    ivp_message("syn0: ");
    this->get_synapse(0)->print();
    ivp_message("\nsyn1: ");
    this->get_synapse(0)->print();
    ivp_message("\n");
}


void IVP_Mindist_Base::get_objects( IVP_Real_Object *objects_out[2] ){
    objects_out[0] = get_mindist_synapse(0)->get_object();
    objects_out[1] = get_mindist_synapse(1)->get_object();
}

void IVP_Mindist_Base::get_ledges( const IVP_Compact_Ledge *ledges_out[2] ){
  ledges_out[0] = get_mindist_synapse(0)->edge->get_compact_ledge();
  ledges_out[1] = get_mindist_synapse(1)->edge->get_compact_ledge();
}


void IVP_Synapse_Real::check_consistency_of_ledge(const IVP_Compact_Edge * /*test_edge*/ )const{
    return;
#if 0
    const IVP_Compact_Ledge *ledge = test_edge->get_compact_ledge();
    const IVP_Compact_Surface *sur = ((IVP_SurfaceManager_Polygon *)l_obj->get_surface_manager())->get_compact_surface();
    const IVP_Compact_Ledgetree_Node *root = sur->get_compact_ledge_tree_root();
    IVP_IF (root ->is_terminal()){
	const IVP_Compact_Ledge *ref_ledge = root->get_compact_ledge();
	IVP_ASSERT (  ref_ledge == ledge );
    }
#endif    
}


const IVP_Compact_Ledge *IVP_Synapse::get_ledge() const
{
    return edge->get_compact_ledge();
}


#if defined(DEBUG) || 1
void IVP_Synapse_Real::print()
{
    char *stat_text = NULL;
    int n_points = 0;
    printf("(syn) ");
    switch(this->status){
    case IVP_ST_POINT:
     	stat_text = (char *)"Point";
	n_points = 1;
	break;
    case IVP_ST_EDGE:
	stat_text = (char *)"Edge";
	n_points = 2;
	break;
    case IVP_ST_TRIANGLE:
	stat_text = (char *)"Triangle";
	n_points = 3;
	break;
    default:
	CORE;
    }
}
#endif

void IVP_Mindist_Manager::print_mindists()
{
    IVP_Mindist *mdist;
    int i;
    printf("\n--------------------------\n");
    for(i=0, mdist=this->exact_mindists; mdist; mdist=mdist->next, i++){
	mdist->print("exact:");
    }
    for(i=0, mdist=this->invalid_mindists; mdist; mdist=mdist->next, i++){
	mdist->print("invalid:");
    }
}

void IVP_Mindist::init_mindist(IVP_Real_Object *pop0, IVP_Real_Object *pop1,const IVP_Compact_Edge *e0,const  IVP_Compact_Edge *e1){
  IVP_Synapse_Real *syn0 = this->get_synapse(0);
  IVP_Synapse_Real *syn1 = this->get_synapse(1);

  switch (pop1->get_type()){
  case IVP_POLYGON:
    syn1->init_synapse(this, pop1,  e1, IVP_ST_POINT);
    pop1->get_surface_manager()->add_reference_to_ledge(e1->get_compact_ledge());
    break;
  case IVP_BALL:
	  if ( pop0->get_type() == IVP_BALL ){
		  if ( pop0->client_data < pop1->client_data ) {
			IVP_Synapse_Real *synh; synh = syn0; syn0 = syn1; syn1 = synh;
		  }
	  }else{
		IVP_Synapse_Real *synh; synh = syn0; syn0 = syn1; syn1 = synh;
	  }
    syn1->init_synapse(this, pop1,  e1,IVP_ST_BALL);
    break;
  default:
    CORE;
  }
  switch (pop0->get_type()){
  case IVP_POLYGON:
    syn0->init_synapse(this, pop0,  e0, IVP_ST_POINT);
    pop0->get_surface_manager()->add_reference_to_ledge(e0->get_compact_ledge());
    break;
  case IVP_BALL:
    syn0->init_synapse(this, pop0,  e0, IVP_ST_BALL);
    break;
  default:
    CORE;
  }

  syn0->set_synapse_mindist(this);
  syn1->set_synapse_mindist(this);

  this->sum_extra_radius = pop0->get_extra_radius() + pop1->get_extra_radius();

  IVP_Mindist_Manager *mm = pop0->get_environment()->get_mindist_manager();
  	    
  if (pop0->get_controller_phantom() || pop1->get_controller_phantom() ){
    this->mindist_function = IVP_MF_PHANTOM;
    mm->insert_and_recalc_phantom_mindist(this);   // #+# insert only
  }else{
    mm->insert_and_recalc_exact_mindist(this);  // #+# insert only (data cache !!)
  }

}

class IVP_MM_CMP_Key {
public:
    const IVP_Real_Object *object[2];
    const IVP_Compact_Ledge *ledge[2];
};

class IVP_MM_CMP {
public:
    static inline int calc_hash_index( IVP_MM_CMP_Key * o){
    //lwss -x64 fixes
	//int x = (int)o->ledge[0] ^ ( int(o->ledge[1])* 75 );
	intptr_t x = (intptr_t)o->ledge[0] ^ ( intptr_t (o->ledge[1])* 75 );
	//lwss end
	return x + 1023 * (x>>8);
    }

    // calc hash index of c, con is a reference
    static inline int calc_hash_index( IVP_Collision *c, IVP_MM_CMP_Key * /*ref_key*/){
	const IVP_Compact_Ledge *ledge[2];
	c->get_ledges(ledge);
	//lwss -x64 fixes
	//int x = (int)ledge[0] ^ ( int(ledge[1])* 75 );
	intptr_t x = (intptr_t)ledge[0] ^ ( intptr_t (ledge[1])* 75 );
	//lwss end
	return x + 1023 * (x>>8);
    }

    static inline IVP_BOOL are_equal( IVP_Collision *c, IVP_MM_CMP_Key *search ){
	const IVP_Compact_Ledge *ledge[2];
	c->get_ledges(ledge);
	if (ledge[0] == search->ledge[0] && ledge[1] == search->ledge[1]){
#ifdef DEBUG
	    IVP_Real_Object *object[2];
	    c->get_objects( object );
	    IVP_ASSERT( object[0] == search->object[0]);
	    IVP_ASSERT( object[1] == search->object[1]);
#endif	    
	    return IVP_TRUE;
	}

	return IVP_FALSE;
    }
};

class IVP_Vector_of_Collisions_128 : public IVP_U_Vector<class IVP_Collision> {
    void *elem_buffer[128];
public:
    IVP_Vector_of_Collisions_128(): IVP_U_Vector<class IVP_Collision>(&elem_buffer[0],128){;};
};


/********************************************************************************
 *	Name:	  	create_exact_mindists    	
 *	Description:	Insert mindists, between all pairs of ledges of two objects
 *	scan_radius:	the radius, the surface_manager is searched
 *	mindists:		an optional hash of mindists, which is checked and updated
 * 	returns:	the last collision created
 ********************************************************************************/
void IVP_Mindist_Manager::create_exact_mindists(IVP_Real_Object *pop0, IVP_Real_Object *pop1,
						IVP_DOUBLE scan_radius, IVP_U_FVector<IVP_Collision> *mindists,
						const IVP_Compact_Ledge *single_ledge0, const IVP_Compact_Ledge *single_ledge1,
						const IVP_Compact_Ledge *root_ledge0, const IVP_Compact_Ledge *root_ledge1,
						IVP_Collision_Delegator *oo_watcher){
    // takes first point of each pop and adds a mindist
    IVP_Environment *env = pop0->get_environment();
    IVP_Vector_of_Ledges_256 ledges0;
    IVP_Vector_of_Ledges_256 ledges1;

    if (!single_ledge0){
	IVP_DOUBLE search_radius_0;
	IVP_U_Point visitor_position_os0;
	IVP_Core *core1 = pop1->get_core();
	IVP_U_Point pos1; core1->inline_calc_at_position( env->get_current_time(), &pos1 );
	search_radius_0 = core1->upper_limit_radius + pop0->get_extra_radius() + scan_radius;
	IVP_Cache_Object *cache0 = pop0->get_cache_object_no_lock();
	cache0->m_world_f_object.vimult4( &pos1, &visitor_position_os0 );
	pop0->get_surface_manager()->get_all_ledges_within_radius( &visitor_position_os0, search_radius_0 , root_ledge0, NULL, single_ledge1, &ledges0);
    }else{
	ledges0.add( & (IVP_Compact_Ledge &)*single_ledge0);
    }

    if (!single_ledge1) {
	IVP_DOUBLE search_radius_1;
	IVP_U_Point visitor_position_os1;
	IVP_Core *core0 = pop0->get_core();
	IVP_U_Point pos0; core0->inline_calc_at_position( env->get_current_time(), &pos0 );
	search_radius_1 = core0->upper_limit_radius + pop1->get_extra_radius() + scan_radius;
	IVP_Cache_Object *cache1 = pop1->get_cache_object_no_lock();
	cache1->m_world_f_object.vimult4( &pos0, &visitor_position_os1 );
	pop1->get_surface_manager()->get_all_ledges_within_radius( &visitor_position_os1, search_radius_1 , root_ledge1, NULL, single_ledge0, &ledges1);	
    }else{
	ledges1.add( & (IVP_Compact_Ledge &)*single_ledge1);
    }
    	

#ifdef IVP_HOME_BUILD
    // IVP_SUPREME_SUPPORT
    ivp_global_clustering_visualizer.shortrange.analyze_collision(pop0, search_radius_1, pop1, search_radius_0);
#endif

    short a_buffer[1024];
    int buffer_size = 1024;
    
    { // optimize buffersize in order to reduce memory fills at diff_hash
	int ref_size = ledges0.len() * ledges1.len() + mindists->len()+ 1;
	ref_size <<= 1;
	while ( buffer_size > ref_size) buffer_size >>= 1;
    }

    // convert old list into hash
    IVP_Diff_Hash<IVP_MM_CMP, IVP_U_FVector<IVP_Collision>,IVP_Collision, IVP_MM_CMP_Key> diff_hash( mindists, a_buffer, buffer_size, NULL);
    IVP_Vector_of_Collisions_128 new_collisions;
    
    for (int i= ledges0.len()-1; i>=0; i--){
	const IVP_Compact_Ledge *l0 = ledges0.element_at(i);
	const IVP_Compact_Triangle *tri0 = l0->get_first_triangle();
	IVP_MM_CMP_Key key; key.ledge[0] = l0; key.object[0] = pop0;
	
	for (int j = ledges1.len()-1;j>=0;j--){
	    const IVP_Compact_Ledge *l1 = ledges1.element_at(j);
	    key.ledge[1] = l1; key.object[1] = pop1;
	    
	    if ( diff_hash.check_element( &key, NULL )){
		continue;
	    }
	    
	    IVP_Mindist *new_mindist;
	    if ( !l0->is_terminal() || !l1->is_terminal()){
	      new_mindist = new IVP_Mindist_Recursive(env, oo_watcher);
	    }else{
	      new_mindist = new IVP_Mindist(env, oo_watcher);
	    }
		IVP_ASSERT(new_collisions.n_elems <= new_collisions.memsize);
	    const IVP_Compact_Triangle *tri1 = l1->get_first_triangle();
	    new_mindist->init_mindist(pop0, pop1, tri0->get_first_edge(),tri1->get_first_edge());
		IVP_ASSERT(new_collisions.n_elems <= new_collisions.memsize);
	    new_collisions.add(new_mindist);
		IVP_ASSERT(new_collisions.n_elems <= new_collisions.memsize);
	}
    }
    { // delete unused collisions
	for ( int x = mindists->len()-1; x >= diff_hash.n_found_objects; x--){
	    IVP_Collision *c = mindists->element_at(x);
	    delete c;
	}
    }
    { // add new collisions
	for (int x = new_collisions.len()-1; x>=0; x--){
	    IVP_Collision *c = new_collisions.element_at(x);
	    mindists->add(c);
	}
    }
    
#if 0
    { // check final mindists elements
	for (int x = mindists->len()-1; x>=0; x--){
	    IVP_Collision *c = mindists->element_at(x);
	    const IVP_Compact_Ledge *ledges[2];
	    c->get_ledges(ledges);
	    IVP_ASSERT( ledges0.index_of( ledges[0] ) >= 0 && ledges1.index_of( ledges[1] )  >= 0 );
	}
    }
    { // check all ledge ledge combinations
	for (int i= ledges0.len()-1; i>=0; i--){
	    const IVP_Compact_Ledge *l0 = ledges0.element_at(i);
	    for (int j = ledges1.len()-1;j>=0;j--){
		const IVP_Compact_Ledge *l1 = ledges1.element_at(j);
		int x;
		for (x = mindists->len()-1; x>=0; x--){
		    IVP_Collision *c = mindists->element_at(x);
		    const IVP_Compact_Ledge *ledges[2];
		    c->get_ledges(ledges);
		    if (ledges[0] == l0 && ledges[1] == l1) {
			x = -1;
			break;
		    }
		}
		IVP_ASSERT( x = -1);
	    }
	}
    }
#endif	    

}


void IVP_Mindist_Manager::insert_exact_mindist( IVP_Mindist *new_mindist){
  IVP_ASSERT(new_mindist->mindist_status == IVP_MD_UNINITIALIZED);
  new_mindist->mindist_status = IVP_MD_EXACT;
  
  new_mindist->next = this->exact_mindists;
  new_mindist->prev = NULL;

  if(new_mindist->next){
    new_mindist->next->prev=new_mindist;
  }
  this->exact_mindists = new_mindist;
  
  IVP_Synapse_Real *syn0 = new_mindist->get_synapse(0);
  IVP_Synapse_Real *syn1 = new_mindist->get_synapse(1);

  syn0->insert_exact_synapse_in_object();
  syn1->insert_exact_synapse_in_object();
  
  if ((syn0->get_core()->car_wheel || syn1->get_core()->car_wheel))
  {
	  if ( new_mindist->mindist_function != IVP_MF_PHANTOM )
	  {
		  //ivp_message("exact mindist inserted %x at %f\n", new_mindist, syn0->get_object()->get_environment()->get_current_time().get_seconds());
		  wheel_look_ahead_mindists.add(new_mindist);
	  }
  }
  
}

void IVP_Mindist_Manager::insert_invalid_mindist( IVP_Mindist *new_mindist){
  IVP_ASSERT(new_mindist->mindist_status == IVP_MD_UNINITIALIZED);
  new_mindist->mindist_status = IVP_MD_INVALID;
  
  new_mindist->next = this->invalid_mindists;
  new_mindist->prev = NULL;

  if(new_mindist->next){
    new_mindist->next->prev=new_mindist;
  }
  this->invalid_mindists = new_mindist;
  new_mindist->get_synapse(0)->insert_invalid_synapse_in_object();
  new_mindist->get_synapse(1)->insert_invalid_synapse_in_object();
}

void IVP_Mindist::exact_mindist_went_invalid(IVP_Mindist_Manager *mm){
    mm->remove_exact_mindist(this);
    mm->insert_invalid_mindist(this);
}

// #+# kill this function due data cache problems on PSXII
void IVP_Mindist_Manager::insert_and_recalc_exact_mindist( IVP_Mindist *new_mindist){

    IVP_ASSERT(new_mindist->mindist_status == IVP_MD_UNINITIALIZED);
    IVP_ASSERT(new_mindist->mindist_function != IVP_MF_PHANTOM);
    
    new_mindist->mindist_status = IVP_MD_EXACT;
    
    new_mindist->next = this->exact_mindists;
    new_mindist->prev = NULL;

    if(new_mindist->next){
	new_mindist->next->prev=new_mindist;
    }

    IVP_Synapse_Real *syn0 = new_mindist->get_synapse(0);
    IVP_Synapse_Real *syn1 = new_mindist->get_synapse(1);
    
    syn0->insert_exact_synapse_in_object();
    syn1->insert_exact_synapse_in_object();
    
    //TL : when obj is revived it gets all mindists and builds friction system -> do not move mindist into hull
    
    this->exact_mindists = new_mindist;
    new_mindist->recalc_mindist();

    if ((syn0->get_core()->car_wheel || syn1->get_core()->car_wheel)){
	//ivp_message("exact mindist inserted %x at %f\n", new_mindist, syn0->get_object()->get_environment()->get_current_time().get_seconds());
	wheel_look_ahead_mindists.add(new_mindist);
    }

    
    if (new_mindist->recalc_result == IVP_MDRR_OK){
	IVP_Core *core0=new_mindist->get_synapse(0)->l_obj->physical_core;
	IVP_Core *core1=new_mindist->get_synapse(1)->l_obj->physical_core;
	IVP_Movement_Type allow_hull_bit;
	allow_hull_bit=(IVP_Movement_Type)((core0->movement_state)|(core1->movement_state));
	IVP_BOOL allow_hull_conversion;
	if(allow_hull_bit>=IVP_MT_GET_MINDIST)  {
	    allow_hull_conversion = IVP_FALSE;
	} else {
	    allow_hull_conversion = IVP_TRUE;
	}
	new_mindist->update_exact_mindist_events(allow_hull_conversion,IVP_EH_NOW);
    }else{
	new_mindist->exact_mindist_went_invalid(this);
    }
}

void IVP_Mindist_Manager::insert_and_recalc_phantom_mindist( IVP_Mindist *new_mindist){
    IVP_ASSERT(new_mindist->mindist_status == IVP_MD_UNINITIALIZED);
    IVP_ASSERT(new_mindist->mindist_function == IVP_MF_PHANTOM);
    
    new_mindist->recalc_invalid_mindist();  // be carefull, avoid deep recursions
    
    IVP_FLOAT uncertanty;
    if (new_mindist->recalc_result == IVP_MDRR_OK){
      if (new_mindist->get_length() > 0.0f){
	  if (new_mindist->is_in_phantom_set ){
	      mindist_left_phantom(new_mindist);
	  }
	  insert_exact_mindist(new_mindist);
	  new_mindist->update_exact_mindist_events(IVP_TRUE ,IVP_EH_BIG_DELAY);

	  return;
      }
      uncertanty =   - new_mindist->get_length();
    }else{
      uncertanty =   new_mindist->sum_extra_radius;
    }
    
    if(!new_mindist->is_in_phantom_set )
    {
	mindist_entered_phantom(new_mindist);
    }
    
    {
      IVP_Synapse_Real *syn = new_mindist->get_synapse(0);
      IVP_Real_Object *obj = syn->l_obj;
      IVP_Controller_Phantom *cp = obj->get_controller_phantom();
      if (cp){
	uncertanty += cp->exit_policy_extra_radius;
      }else{
	  IVP_Synapse_Real *syn2 = new_mindist->get_synapse(1);
	  IVP_Real_Object *obj2 = syn2->l_obj;
	  IVP_Controller_Phantom *cp2 = obj2->get_controller_phantom();
	  if (cp2){
	      uncertanty += cp2->exit_policy_extra_radius;
	  }
      }
    }
    insert_lazy_hull_mindist( new_mindist, uncertanty);    
}

class IVP_OO_CMP {
public:
    static inline int calc_hash_index( IVP_Real_Object * o){
    //lwss -x64 fixes
	//int x = (int)o;
	int x = (intptr_t)o;
	//lwss end
	return x + 1023 * (x>>8);
    }

    // calc hash index of c, con is a reference
    static inline int calc_hash_index( IVP_Collision *c, IVP_Real_Object * con){
	IVP_Real_Object *objects[2];
	c->get_objects(objects);
	//lwss -x64 fixes
	//int x = int(objects[0]) ^ int(objects[1]) ^ int(con);  // take other object (trick to avoid if)
	int x = intptr_t(objects[0]) ^ intptr_t(objects[1]) ^ intptr_t(con);  // take other object (trick to avoid if)
	//lwss end
	IVP_ASSERT( objects[0] == con || objects[1] == con );
	return x + 1023 * (x>>8);
    }

    static inline IVP_BOOL are_equal( IVP_Collision *c, IVP_Real_Object *search ){
	IVP_Real_Object *objects[2];
	c->get_objects(objects);
	if (objects[0] == search) return IVP_TRUE;
	if (objects[1] == search) return IVP_TRUE;
	return IVP_FALSE;
    }
};

class IVP_Vector_of_OV_Elements_128 : public IVP_U_Vector<class IVP_OV_Element> {
    void *elem_buffer[128];
public:
    IVP_Vector_of_OV_Elements_128(): IVP_U_Vector<class IVP_OV_Element>(&elem_buffer[0],128){;};
};

//vector of cores with 2 default elements already alloced
class IVP_Vector_of_Objects_128: public IVP_U_Vector<class IVP_Real_Object> {
    IVP_Real_Object *elem_buffer[128];
public:
    IVP_Vector_of_Objects_128(): IVP_U_Vector<IVP_Real_Object>( (void **)&elem_buffer[0],128 ){;};
};

void IVP_Mindist_Manager::recheck_ov_element(IVP_Real_Object *object){

    IVP_Vector_of_OV_Elements_128 colliding_elements; // for recheck_ov_element

    IVP_OV_Element *elem = object->get_ov_element();

    if(!elem) 
		return; // not collision enabled 

    // check surrounding

    // check, whether old elem is still useable
    IVP_Core *core = object->get_core();
    const IVP_U_Point *object_position = core->get_position_PSI();
    
#if 0
    const IVP_DOUBLE use_old_hull_factor = 0.5f;
    IVP_U_Point sphere_position; sphere_position.set(elem->center);

    IVP_DOUBLE moved_distance = sphere_position.quad_distance_to(object_position);
    IVP_DOUBLE old_hull_time = elem->radius - core->upper_limit_radius;
    if (moved_distance < use_old_hull_factor * old_hull_time){
	return;
    }
#endif
    environment->ov_tree_manager->remove_ov_element( elem);

    environment->get_statistic_manager()->range_world_exceeded++;

    elem->center.set(object_position);

    IVP_DOUBLE radius;

    IVP_Hull_Manager *hm = object->get_hull_manager();

    if (!scanning_universe)	{	// check for new objects in the universe manager
	IVP_DOUBLE hull_time = environment->range_manager->get_coll_range_in_world(object);	// distance to check
	IVP_DOUBLE real_check_sphere = hull_time + core->upper_limit_radius;

	IVP_Universe_Manager *um = environment->get_universe_manager();
	if (um){
	    // only ask for new objects if object can collide 
	    IVP_Movement_Type mt = object->get_movement_state();
	    if ( IVP_MTIS_CAN_COLLIDE(mt)){
		scanning_universe = IVP_TRUE;   // needed to avoid recursion
		um->ensure_objects_in_environment( object, &elem->center, real_check_sphere);
		scanning_universe = IVP_FALSE;  // finished
	    }
	}
	radius = environment->ov_tree_manager->insert_ov_element( elem, real_check_sphere, real_check_sphere, &colliding_elements);
	
	IVP_DOUBLE real_hull_time =   radius - core->upper_limit_radius;
	elem->add_to_hull_manager( hm, real_hull_time );	    // insert into event queue
	
    } else {	// recursive call by IVP_Universe_Manager, use minimal radius
	IVP_DOUBLE real_check_sphere = core->upper_limit_radius;
	radius = environment->ov_tree_manager->insert_ov_element( elem, real_check_sphere, real_check_sphere, NULL);
	IVP_DOUBLE real_hull_time =   P_DOUBLE_EPS;	// recheck as soon as possible because it's not checked now
	elem->add_to_hull_manager( hm, real_hull_time );	    // insert into event queue
	return;			// thats it, IVP_Universe_Manager can only add objects which do not have collision candidates except object
    }

    IVP_Collision_Filter *coll_filter = environment->get_collision_filter();

    short a_buffer[1024];
    int buffer_size = 1024;
    { // optimize buffersize in order to reduce memory fills at diffhash
	int ref_size = elem->collision_fvector.len() + colliding_elements.len() + 1;
	ref_size <<= 2;
	while ( buffer_size > ref_size) buffer_size >>= 1;
    }
    // convert old list into hash
    IVP_Diff_Hash<IVP_OO_CMP, IVP_U_FVector<IVP_Collision>,IVP_Collision, IVP_Real_Object> diff_hash( &elem->collision_fvector, a_buffer, buffer_size, object );

    // compare new and old list and create/delete elements accordingly
    // old elements will be sorted to the start of the vector
    // unused to the end
    // new objects into a new vector
    IVP_Vector_of_Objects_128 new_objects;
    
    { // compare new list with old list
      for (int i= colliding_elements.len()-1;i>=0;i--){
	IVP_OV_Element *el = colliding_elements.element_at(i); 

	// now build a new one
	IVP_Real_Object *obj0=object;
	IVP_Real_Object *obj1=el->real_object;
	if( !IVP_MTIS_CAN_COLLIDE(obj0->get_movement_state()) && !IVP_MTIS_CAN_COLLIDE( obj1->get_movement_state())){
	  continue; 
	}
	if(obj0->friction_core == obj1->friction_core) 
	    continue;

	if((obj0->get_core()->pinned && obj1->get_core()->pinned) ||
		(obj0->get_core()->pinned && obj1->get_core()->physical_unmoveable) ||
		(obj0->get_core()->physical_unmoveable && obj1->get_core()->pinned))
	    continue;
	
	if (coll_filter->check_objects_for_collision_detection(obj0,obj1) == IVP_FALSE) continue;
	// search for existing
	if ( diff_hash.check_element( obj1, obj0 )){
	    continue;
	}
	new_objects.add(obj1);
      }
    }
    { // delete unused collisions
	for ( int x = elem->collision_fvector.len()-1; x >= diff_hash.n_found_objects; x--){
	    IVP_Collision *c = elem->collision_fvector.element_at(x);
		//
		// HACKHACK: E3 2003, get around delete of collision object that is calling us up the stack
		//
		if ( c != g_pCurrentMindist )
		{
		    P_DELETE(c);
		}
		else
		{
			g_fDeferDeleteMindist = true;
		}
	}
    }
    { // create new ones
	for ( int x = new_objects.len()-1; x>=0; x -- ) {
	    IVP_Real_Object *obj1 = new_objects.element_at(x);
	    for (int k = environment->collision_delegator_roots.len()-1; k>=0; k--){
		IVP_Collision_Delegator_Root *cdr = environment->collision_delegator_roots.element_at(k);
		IVP_Collision *coll = cdr->delegate_collisions_for_object(object, obj1);
		if (coll) break;
	    }
	}
    }
}

 
void IVP_Mindist_Manager::enable_collision_detection_for_object(IVP_Real_Object *object){
    P_DELETE(object->ov_element);
    object->ov_element = new IVP_OV_Element(object);
    recheck_ov_element(object);
}


/** recalculation of all exact mindist at the end of PSI */
void IVP_Mindist_Manager::recalc_all_exact_mindists_events()
{
	IVP_Mindist *mdist, *mdist_next;
	
	for( mdist=this->exact_mindists; mdist; mdist=mdist_next )
	{
		mdist_next = mdist->next;
		IVP_ASSERT(mdist->mindist_status == IVP_MD_EXACT);
		IVP_IF_PREFETCH_ENABLED(mdist_next)
		{
			IVP_PREFETCH_BLOCK(mdist_next,sizeof(*mdist_next));
		}
		mdist->update_exact_mindist_events(IVP_TRUE, IVP_EH_SMALL_DELAY);
	}
}

/** recalculation of all exact mindist at the end of PSI */
void IVP_Mindist_Manager::recalc_all_exact_wheel_mindist()
{
	for ( int i = wheel_look_ahead_mindists.len()-1; i>=0; i--)
	{
		IVP_Mindist *mdist = wheel_look_ahead_mindists.element_at(i);
		
		IVP_ASSERT( mdist->mindist_function != IVP_MF_PHANTOM );
		this->recalc_exact_mindist(mdist);
		
		IVP_Synapse_Real *syn0 = mdist->get_sorted_synapse(0);
		IVP_Synapse_Real *syn1 = mdist->get_sorted_synapse(1);
		
		if (syn0->get_status()!=IVP_ST_BALL) 
		{
			IVP_Synapse_Real *temp=syn0;    
			syn0=syn1;    
			syn1=temp;
		}

		if ( syn1->get_edge() == mdist->last_visited_triangle ) 
			continue;
		
		if( (syn1->get_status()==IVP_ST_TRIANGLE) && (syn0->get_status()==IVP_ST_BALL) ) 
		{
			IVP_FLOAT md_len = mdist->get_length();
			if( md_len < ivp_mindist_settings.max_distance_for_friction ) 
			{
				if ( syn1->get_ledge()->is_terminal() )
				{  
					mdist->create_cp_in_advance_pretension(syn0->l_obj,md_len);
					//ivp_message("wheel check mindist %x at %f\n", mdist,    environment->get_current_time().get_seconds());
					
					//wheel_look_ahead_mindists.remove_at_and_allow_resort( i );
					mdist->last_visited_triangle = syn1->get_edge();
				}
			}
		}
	}
}


/** recalculation of all exact mindist at the end of PSI */
void IVP_Mindist_Manager::recalc_all_exact_mindists(){
    IVP_Mindist *mdist, *mdist_next;
    for(mdist=this->exact_mindists; mdist; mdist=mdist_next){
	mdist_next = mdist->next;
	IVP_ASSERT(mdist->mindist_status == IVP_MD_EXACT);

	IVP_IF_PREFETCH_ENABLED(mdist_next){
	    IVP_PREFETCH_BLOCK(mdist_next,sizeof(*mdist_next));
	}
	this->recalc_exact_mindist(mdist);
    }
}

/* check length, check for hull, check for coll events*/
void IVP_Mindist::update_exact_mindist_events(IVP_BOOL allow_hull_conversion, IVP_MINDIST_EVENT_HINT event_hint)
{
	IVP_IF(1) 
	{
		IVP_ASSERT( mindist_status == IVP_MD_EXACT);
		IVP_Debug_Manager *dm=get_environment()->get_debug_manager();
		if(dm->file_out_impacts) 
		{
			fprintf(dm->out_deb_file,"doing_mindist_events %lx at %f\n",0x0000ffff&(long)this,get_environment()->get_current_time().get_time());
		}
	}
	
	IVP_Synapse_Real *syn0 = get_sorted_synapse(0);
	IVP_Synapse_Real *syn1 = get_sorted_synapse(1);
	IVP_Core *core0 = syn0->get_core();
	IVP_Core *core1 = syn1->get_core();
	IVP_Environment *env = get_environment();
	
	// speed is speed toward each other
	IVP_Mindist_Event_Solver mim;
	{
		mim.sum_max_surface_rot_speed = core0->max_surface_rot_speed + core1->max_surface_rot_speed;
		mim.worst_case_speed = mim.sum_max_surface_rot_speed + core0->current_speed + core1->current_speed;
	}
	
	if (this->index != IVP_U_MINLIST_UNUSED)
	{
		IVP_Time_Manager *time_manager = env->get_time_manager();
		time_manager->remove_event(this);
		this->index = IVP_U_MINLIST_UNUSED;
	}
	
	{  
		// check for hull
		IVP_DOUBLE coll_dist = this->get_coll_dist();
		IVP_DOUBLE hull_check_len = coll_dist + env->get_delta_PSI_time() * mim.worst_case_speed * 2.1f;
		IVP_Mindist_Manager *my_manager = env->get_mindist_manager();
		
		if( get_length() > hull_check_len)
		{
			if (allow_hull_conversion)
			{
				// check if hull survives at least some psi's
				my_manager->remove_exact_mindist(this);	
				my_manager->insert_hull_mindist( this, get_length() - coll_dist ); // secdist is subtracted, because hull manager is already checked till next PSI
			}
			return;
		}
	}
	
	{
		mim.projected_center_speed =  core1->speed.dot_product(&contact_plane) - core0->speed.dot_product(&contact_plane);	// inverse to direction
		
		IVP_DOUBLE rot0_projected = contact_plane.dot_product( &core0->rotation_axis_world_space);
		rot0_projected *= rot0_projected;
		rot0_projected = IVP_Inline_Math::ivp_sqrtf(1.001f - rot0_projected) * core0->max_surface_rot_speed;	// make sure sqrt does not crash -> increase result
		
		IVP_DOUBLE rot1_projected = contact_plane.dot_product( &core1->rotation_axis_world_space);
		rot1_projected *= rot1_projected;
		rot1_projected = IVP_Inline_Math::ivp_sqrtf(1.001f - rot1_projected) * core1->max_surface_rot_speed;
		mim.max_coll_speed = rot0_projected + rot1_projected + mim.projected_center_speed;
		mim.t_now =  env->get_current_time();
		mim.t_max =  env->get_next_PSI_time();
	}
	
	IVP_DOUBLE len = get_length();
	
	if( mim.max_coll_speed < ivp_mindist_settings.speed_after_keeper_dist ) 
	{	
		if( len < ivp_mindist_settings.keeper_dist ) 
		{
			//mim.flag_near_slow_impact = IVP_TRUE;
		}
		if (mim.max_coll_speed < P_DOUBLE_EPS)
		{
			return; // no movement towards each other!!
		}
	}
	
	IVP_DOUBLE d_time = mim.t_max - mim.t_now;
	IVP_DOUBLE d_len = d_time * mim.max_coll_speed;	// max length that the system can move till next PSI
	IVP_DOUBLE sec_dist = this->get_coll_dist() + d_len;
	
	IVP_IF(ivp_check_debug_mindist(this))
	{
		ivp_message("%32s  event  len_now %g, sec_dist:%g\n","update_exact_mindist_events",  get_length(), sec_dist);
	}
	
	if (len >= sec_dist)
	{  
		// no collision possible
		return;
	}
	
	// real check next collision time
	if (this->mindist_function == IVP_MF_PHANTOM) 
	{
		// no deep checking for phantoms //@@CB
		return;
	} 
	
	mim.mindist = this;
	mim.environment = env;
	
	/* ************** calc the time of the next event ******************* */
	/* ************** calc the time of the next event ******************* */
	/* ************** calc the time of the next event ******************* */
	if (coll_dist_selector > 0)
	{ 
		// decrease collision distance
		static int count = 0;
		if (count ++ > 2)
		{
			coll_dist_selector-=1;
			count = 0;
		}
	}
	mim.calc_time_of_next_event();
	
	IVP_Time &next_event = mim.event_time_out;
	
	IVP_IF(ivp_check_debug_mindist(this))
	{
		ivp_message("%32s     calc_time_of_next_event called: ","''");
		if (mim.event_type_out != IVP_COLL_NONE)		
			ivp_message("event at %f\n", next_event.get_time());
		else		
			ivp_message("no\n");
	}
	
	if (mim.event_type_out != IVP_COLL_NONE)
	{
		if ( next_event - mim.t_now < P_FLOAT_RES)
		{	// maybe an error
			if (event_hint == IVP_EH_NOW)
			{
				next_event = mim.t_now;
			}
			else
			{
				// check if we can do recalc  right now
				// only if allowed and event is a coll event
				IVP_DOUBLE mdist = len - ivp_mindist_settings.real_coll_dist;
				if ( (event_hint == IVP_EH_BIG_DELAY) || (mim.event_type_out & 0x0f)) 
				{	// 
					if (mdist >= P_DOUBLE_RES)
					{
						next_event = env->get_current_time() + mdist/mim.worst_case_speed + 
							(ivp_mindist_settings.event_queue_min_delta_time_base * 1e-4f) * env->get_delta_PSI_time(); //@@CB
					}
					else
					{
						next_event = env->get_current_time() + (ivp_mindist_settings.event_queue_min_delta_time_base * 1e-3f) 
							* env->get_delta_PSI_time(); //@@CB
					}
				}
				else
				{
					if (mdist >= P_DOUBLE_RES)
					{
						next_event = env->get_current_time() + 0.1f * mdist/mim.worst_case_speed 
							+ (ivp_mindist_settings.event_queue_min_delta_time_base * 1e-7f) * env->get_delta_PSI_time(); //@@CB
					}
					else
					{
						next_event = env->get_current_time() + (ivp_mindist_settings.event_queue_min_delta_time_base * 1e-5f) 
							* env->get_delta_PSI_time(); //@@CB
					}
				}

				if (next_event - env->get_next_PSI_time() >= 0.0f) 
					return;
			}
		}
		
		// insert real event in task manager
		IVP_Time_Manager *tm = env->get_time_manager();
		
		if (0)
		{	
			tm->insert_event(this,next_event);
		}
		else
		{
			IVP_FLOAT event_time = next_event - tm->base_time;
			this->index = tm->min_hash->add((void *)this, event_time);
		}
		
		IVP_ASSERT(this->index !=IVP_U_MINLIST_UNUSED);
		
		this->coll_type = mim.event_type_out;
	}
}

void IVP_Synapse::hull_manager_is_reset(IVP_FLOAT dt, IVP_FLOAT center_dt){  // hull_manager is reset
    IVP_Mindist *md = (IVP_Mindist *) this->get_synapse_mindist();
    md->hull_manager_is_reset(dt,center_dt);
}

void IVP_Synapse::hull_limit_exceeded_event(IVP_Hull_Manager *, IVP_HTIME hull_intrusion_value){
    IVP_Mindist *md = (IVP_Mindist *) this->get_synapse_mindist();
    md->mindist_hull_limit_exceeded_event(hull_intrusion_value);
}

void IVP_Synapse::hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *){

    delete this->get_synapse_mindist(); //dont use P_DELETE, because mindist deletes synapse and P_DELETE sets pointer to NULL
}

void ivp_optimiztion_center_check_successfull(){
  ;
}

void IVP_Mindist::hull_manager_is_reset(IVP_FLOAT dt, IVP_FLOAT center_dt){  // hull_manager is reset
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED
    sum_angular_hull_time += dt - center_dt;
#endif
}


void IVP_Mindist::mindist_hull_limit_exceeded_event( IVP_HTIME hull_intrusion_value){
    if (mindist_status == IVP_MD_HULL_RECURSIVE){
	IVP_Mindist_Recursive *mr = (IVP_Mindist_Recursive *)this;
	mr->rec_hull_limit_exceeded_event();
	return;
    }
    IVP_ASSERT(mindist_status == IVP_MD_HULL );

    IVP_Synapse_Real *syn0 = get_sorted_synapse(0);
    IVP_Synapse_Real *syn1 = get_sorted_synapse(1);


    IVP_Real_Object *obj0 = syn0->get_object();
    IVP_Real_Object *obj1 = syn1->get_object();
    IVP_Hull_Manager *m0 = obj0->get_hull_manager();
    IVP_Hull_Manager *m1 = obj1->get_hull_manager();

    IVP_Core *core0 = obj0->get_core();
    IVP_IF_PREFETCH_ENABLED(IVP_TRUE){
        m0->prefetch0_hull();
	IVP_PREFETCH(&core0, &((IVP_Core *)0)->current_speed);
	IVP_PREFETCH(core0->get_position_PSI(),0);
    }

    IVP_Core *core1 = obj1->get_core();
    IVP_IF_PREFETCH_ENABLED(IVP_TRUE){
        m1->prefetch0_hull();
	IVP_PREFETCH(&core1,&((IVP_Core *)0)->current_speed);
	IVP_PREFETCH(core1->get_position_PSI(),0);
    }

    IVP_Time current_time = obj0->get_environment()->get_current_time();
    IVP_U_Point center0; core0->inline_calc_at_position( current_time, &center0 );
    IVP_U_Point center1; core1->inline_calc_at_position( current_time, &center1 );

    IVP_U_Point diff; diff.subtract( &center0, &center1 );
    IVP_DOUBLE qdistance  = diff.quad_length();

    IVP_DOUBLE speed0 = core0->current_speed + core0->max_surface_rot_speed + P_DOUBLE_EPS; // 
    IVP_DOUBLE speed1 = core1->current_speed + core1->max_surface_rot_speed + P_DOUBLE_EPS;
    IVP_DOUBLE speed = speed0 + speed1;

    IVP_IF(ivp_check_debug_mindist(this)){
	ivp_message("%32s center_distance %f old_dist %f\n","hull_limit_exceeded_event",  IVP_Inline_Math::ivp_sqrtf(qdistance), get_length());
    }

    m0->prefetch1_hull();
    m1->prefetch1_hull();

#if defined(IVP_HALFSPACE_OPTIMIZATION_ENABLED)
    // fast check projected centers
    if (!disable_halfspace_optimization){
      // calc real center movement
      IVP_DOUBLE new_contact_dot_diff_center = diff.dot_product( &contact_plane );
      IVP_DOUBLE delta_center_distance = contact_dot_diff_center - new_contact_dot_diff_center;

      // calc estimated angular movement
      IVP_DOUBLE current_angular_hull_time_0 = m0->get_angular_hull_time(current_time);
      IVP_DOUBLE current_angular_hull_time_1 = m1->get_angular_hull_time(current_time);

      IVP_DOUBLE sum_angular = current_angular_hull_time_0 + current_angular_hull_time_1;
      IVP_DOUBLE angular_movement = sum_angular - this->sum_angular_hull_time;

      IVP_DOUBLE estimated_distance = this->get_length() - angular_movement - delta_center_distance + hull_intrusion_value;
      // this estimated_distance can now be used to throw mindist into hull again
      IVP_DOUBLE const look_ahead_factor = 6.0f;
	IVP_IF( 0 && this->contact_plane.k[1] == -1.0f && !this->mindist_function){
	    ivp_message("sum_angular %f   angular_movement %f    sum_angular_hull_time %f\n", 
		sum_angular, angular_movement, sum_angular_hull_time);
	}

      
      if ( estimated_distance > speed * obj0->get_environment()->get_delta_PSI_time() * look_ahead_factor){
	// successfull delay
#ifdef LINUX
	ivp_optimiztion_center_check_successfull();
#endif
	IVP_IF(ivp_check_debug_mindist(this)){
	  ivp_message("%32s estimated_distance into hull dist: %f error %f speed %f\n","hull_limit_exceeded_event",  IVP_Inline_Math::ivp_sqrtf(qdistance),estimated_distance,speed);
	}

        this->sum_angular_hull_time = sum_angular;
	this->contact_dot_diff_center = new_contact_dot_diff_center;
	this->len_numerator = estimated_distance - hull_intrusion_value;

	IVP_DOUBLE dist_factor = estimated_distance / speed;
	IVP_DOUBLE dist0 = dist_factor * speed0;
	IVP_DOUBLE dist1 = dist_factor * speed1;
	IVP_IF( 0 &&  this->contact_plane.k[1] == -1.0f && !this->mindist_function){
	    ivp_message("est_dist %f, center_dist %f   sum_angular %f\n", 
		len_numerator, delta_center_distance, sum_angular);
	}

	
	syn0->get_hull_manager()->update_synapse(syn0,current_time, dist0);
	syn1->get_hull_manager()->update_synapse(syn1,current_time, dist1);
	return;
      }
    }
#endif
    
    // fast check sphere first
    if (0){ // check for small objects
      IVP_FLOAT radius = core0->upper_limit_radius + core1->upper_limit_radius;
      IVP_FLOAT qradius = radius * radius;

      if ( qdistance > qradius){ // seems like two small objects

	const IVP_DOUBLE look_ahead_time = 0.5f;
	IVP_DOUBLE hull_radius = radius + speed * look_ahead_time;
	qradius = hull_radius * hull_radius;

	if ( qdistance > qradius){ // seems like two small objects
	  IVP_DOUBLE distance = IVP_Inline_Math::ivp_sqrtf( qdistance)  - radius;
	  IVP_IF(ivp_check_debug_mindist(this)){
	    ivp_message("%32s center_distance %f  speed %f\n","hull_limit_exceeded_event",  distance, speed);
	  }
	 
	  IVP_DOUBLE dist0 = distance * 0.5f;
	  IVP_DOUBLE dist1 = distance * 0.5f;
	  IVP_Time t_now = obj0->get_environment()->get_current_time();
	  syn0->get_hull_manager()->update_synapse(syn0,t_now, dist0);
	  syn1->get_hull_manager()->update_synapse(syn1,t_now, dist1);
	  return;
	}
      }
    }
    
    IVP_Mindist_Manager *mgr = obj0->get_environment()->get_mindist_manager();
    mgr->remove_hull_mindist(this);

    if ( this->mindist_function != IVP_MF_PHANTOM){  // #+# just insert in list and wait for delayed update
      mgr->insert_and_recalc_exact_mindist(this);
    }else{
      mgr->insert_and_recalc_phantom_mindist(this); //@@CB
    }
}

void IVP_Mindist::create_cp_in_advance_pretension(IVP_Real_Object *robject,float gap_len) {
    IVP_Synapse_Friction *old_fr_syn = robject->get_first_friction_synapse();
    if(old_fr_syn) {
        robject->environment->sim_unit_mem->start_memory_transaction();

	    IVP_Friction_System *associated_fs;
	    IVP_BOOL success;
	    IVP_Simulation_Unit *sim_unit_not_destroy=NULL;
	    IVP_BOOL call_recalc_svals=IVP_TRUE;

            IVP_Contact_Point *new_cp=try_to_generate_managed_friction(&associated_fs,&success,sim_unit_not_destroy,call_recalc_svals);


	    if(success==IVP_TRUE) {
			IVP_Contact_Point *old_cp=old_fr_syn->get_contact_point();
			IVP_Time old_time = old_cp->last_time_of_recalc_friction_s_vals;
			old_cp->last_time_of_recalc_friction_s_vals = robject->environment->get_current_time();
			old_cp->recalc_friction_s_vals();

			new_cp->now_friction_pressure = old_cp->now_friction_pressure;
			old_cp->now_friction_pressure = 0.0f;
			new_cp->last_gap_len = gap_len;
			new_cp->last_time_of_recalc_friction_s_vals = old_time;
			old_cp->last_time_of_recalc_friction_s_vals = old_time;

			IVP_U_Float_Point world_friction_vec;
			world_friction_vec.set_multiple(&old_cp->tmp_contact_info->span_friction_v[0],old_cp->span_friction_s[0]);
			world_friction_vec.add_multiple(&old_cp->tmp_contact_info->span_friction_v[1],old_cp->span_friction_s[1]);

			float s1,s2;
			s1=new_cp->tmp_contact_info->span_friction_v[0].dot_product(&world_friction_vec);
			s2=new_cp->tmp_contact_info->span_friction_v[1].dot_product(&world_friction_vec);
			new_cp->span_friction_s[0]= s1;
			new_cp->span_friction_s[1]= s2;
	    }

        robject->environment->sim_unit_mem->end_memory_transaction();
	}
}

void IVP_Mindist::simulate_time_event(IVP_Environment * env)
{
#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
	env->get_performancecounter()->pcount(IVP_PE_AT_INIT);
#endif

    IVP_ASSERT( index == IVP_U_MINLIST_UNUSED );
    IVP_Mindist *l_mindist = this;
    
    // update and check wether announced situation change really has occurred
     IVP_IF(ivp_check_debug_mindist(l_mindist)){
	 IVP_DOUBLE md_len = l_mindist->get_length();
	 ivp_message("%32s COLL_TYPE %X: old_len=%g\n", "Time_Event_Mindist_Coll", l_mindist->coll_type, md_len);
     }

     l_mindist->recalc_mindist();	// calculate the current situation, no phantoms
     IVP_ASSERT( l_mindist->mindist_function != IVP_MF_PHANTOM);

    // what situation is announced ?
     IVP_IF(ivp_check_debug_mindist(l_mindist)){
	 IVP_DOUBLE md_len = l_mindist->get_length();
	 ivp_message("%32s COLL_TYPE %X: current_len=%g\n", "Time_Event_Mindist_Coll", l_mindist->coll_type, md_len);
     }
    
     if (l_mindist->recalc_result == IVP_MDRR_OK){ 
       if ( (l_mindist->coll_type & 0x0f) == 0){		// real collision might have happened
	 IVP_ASSERT(l_mindist->mindist_status == IVP_MD_EXACT);
       
	 IVP_FLOAT md_len = l_mindist->get_length();
	 IVP_FLOAT sec_dist = this->get_coll_dist() + ivp_mindist_settings.mindist_change_force_dist;
	 if(md_len < sec_dist){
	   // mindist shorter than this -> impact
       //ivp_message("do impact %x at %f\n", this, this->get_synapse(0)->get_object()->get_environment()->get_current_time().get_seconds());

	   l_mindist->do_impact();
	 }else{
    	    l_mindist->update_exact_mindist_events(IVP_FALSE, IVP_EH_BIG_DELAY);	// recalculate mindist, no hull conversion
	 }
       }else{
	     l_mindist->update_exact_mindist_events(IVP_FALSE, IVP_EH_SMALL_DELAY);	// recalculate mindist, no hull conversion
       }
     }

#ifdef IVP_ENABLE_PERFORMANCE_COUNTER
	env->get_performancecounter()->pcount(IVP_PE_AT_END);
#endif
    /*
     IVP_FLOAT md_len = l_mindist->get_length();
     
     if( md_len < ivp_mindist_settings.max_distance_for_friction ) {
	    IVP_Synapse *syn1=&l_mindist->synapse[0];
	    IVP_Synapse *syn2=&l_mindist->synapse[1];
	    if(syn1->get_status()!=IVP_ST_BALL) {
		    IVP_Synapse *temp=syn1;    syn1=syn2;    syn2=temp;
	    }	    
//check ball / triangle
	    if( (syn2->get_status()==IVP_ST_TRIANGLE)&&(syn1->get_status()==IVP_ST_BALL) ) {
	    if( syn1->l_obj->get_core()->car_wheel ) {
		    create_cp_in_advance_pretension(syn1->l_obj,md_len);
		    }
	    }
     }
   */
}



/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/
/*****************************************************************/

void IVP_Mindist_Manager::remove_exact_mindist(IVP_Mindist *del_mindist)
{
	if (del_mindist->index != IVP_U_MINLIST_UNUSED)
	{
		IVP_Time_Manager *time_manager = environment->get_time_manager();
		time_manager->remove_event(del_mindist);
		del_mindist->index = IVP_U_MINLIST_UNUSED;
	}
	
	IVP_ASSERT(del_mindist->mindist_status == IVP_MD_EXACT); // requires friction mode
	IVP_IF(1)
	{
		del_mindist->mindist_status = IVP_MD_UNINITIALIZED;
	}
	// de-link mindist from manager
	if(del_mindist->next)
	{
		del_mindist->next->prev = del_mindist->prev;
	}
	if(del_mindist->prev)
	{
		del_mindist->prev->next = del_mindist->next;
	}
	else
	{
		this->exact_mindists = del_mindist->next;
	}
	
	del_mindist->get_synapse(0)->remove_exact_synapse_from_object();
	del_mindist->get_synapse(1)->remove_exact_synapse_from_object();
	
	int i = wheel_look_ahead_mindists.index_of( del_mindist );
	//ivp_message("exact mindist removed %x at %f index %i\n", del_mindist, environment->get_current_time().get_seconds(), i);
	if (i>=0)
	{
		wheel_look_ahead_mindists.remove_at_and_allow_resort(i);
	}
}

void IVP_Mindist_Manager::remove_invalid_mindist(IVP_Mindist *del_mindist)
{
	IVP_ASSERT(del_mindist->mindist_status == IVP_MD_INVALID); // requires friction mode
	
	IVP_IF(1)
	{
		del_mindist->mindist_status = IVP_MD_UNINITIALIZED;
	}
	// de-link mindist from manager
	if(del_mindist->next)
	{
		del_mindist->next->prev = del_mindist->prev;
	}
	if(del_mindist->prev)
	{
		del_mindist->prev->next = del_mindist->next;
	}
	else
	{
		this->invalid_mindists = del_mindist->next;
	}
	del_mindist->get_synapse(0)->remove_invalid_synapse_from_object();
	del_mindist->get_synapse(1)->remove_invalid_synapse_from_object();
}

void IVP_Mindist_Manager::remove_hull_mindist(IVP_Mindist *del_mindist)
{
	IVP_ASSERT(del_mindist->mindist_status == IVP_MD_HULL || del_mindist->mindist_status == IVP_MD_HULL_RECURSIVE);
	
	IVP_IF(1)
	{
		del_mindist->mindist_status = IVP_MD_UNINITIALIZED;
	}
	IVP_Synapse_Real *syn0 = del_mindist->get_synapse(0);
	IVP_Synapse_Real *syn1 = del_mindist->get_synapse(1);
	syn0->get_hull_manager()->remove_synapse(syn0);
	syn1->get_hull_manager()->remove_synapse(syn1);
}

void IVP_Mindist_Manager::insert_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time0, IVP_HTIME hull_time1)
{
	IVP_ASSERT(md->mindist_status == IVP_MD_UNINITIALIZED);
	md->mindist_status = IVP_MD_HULL;
	
	IVP_Synapse_Real *syn0 = md->get_synapse(0);
	IVP_Synapse_Real *syn1 = md->get_synapse(1);
	
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED	
	IVP_DOUBLE center0 = syn0->insert_in_hull_manager(hull_time0);
	IVP_DOUBLE center1 = syn1->insert_in_hull_manager(hull_time1);
	md->sum_angular_hull_time = center0 + center1;
#else
	syn0->insert_in_hull_manager(hull_time0);
	syn1->insert_in_hull_manager(hull_time1);
#endif
}

void IVP_Mindist_Manager::insert_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time)
{
	IVP_Real_Object *obj0 = md->get_synapse(0)->l_obj;
	IVP_Real_Object *obj1 = md->get_synapse(1)->l_obj;
	
	if ( !IVP_MTIS_IS_MOVING(obj0->get_movement_state()))
	{
		insert_hull_mindist( md, 0.0f, hull_time );
	}
	else if ( !IVP_MTIS_IS_MOVING(obj1->get_movement_state()))
	{
		insert_hull_mindist( md, hull_time, 0.0f );
	}
	else
	{
		IVP_Core *core0 = obj0->get_core();
		IVP_Core *core1 = obj1->get_core();
		
		IVP_FLOAT speed0 = core0->max_surface_rot_speed + core0->current_speed + P_FLOAT_EPS;
		IVP_FLOAT speed1 = core1->max_surface_rot_speed + core1->current_speed + P_FLOAT_EPS;
		IVP_FLOAT speed_corr_0 = speed0 + 0.1f * speed1;
		IVP_FLOAT speed_corr_1 = speed1 + 0.1f * speed0;
		IVP_FLOAT sum_speed = speed_corr_0 + speed_corr_1;
		IVP_FLOAT factor = hull_time / sum_speed;
		insert_hull_mindist( md, factor * speed_corr_0, factor * speed_corr_1 );
	}
}

void IVP_Mindist_Manager::insert_lazy_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time0, IVP_HTIME hull_time1)
{
	IVP_ASSERT(md->mindist_status == IVP_MD_UNINITIALIZED);
	md->mindist_status = IVP_MD_HULL;
	
	md->get_synapse(0)->insert_lazy_in_hull_manager(hull_time0);
	md->get_synapse(1)->insert_lazy_in_hull_manager(hull_time1);
}


void IVP_Mindist_Manager::insert_lazy_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time)
{
	IVP_Real_Object *obj0 = md->get_synapse(0)->l_obj;
	IVP_Real_Object *obj1 = md->get_synapse(1)->l_obj;
	
	if ( !IVP_MTIS_IS_MOVING(obj0->get_movement_state()))
	{
		insert_lazy_hull_mindist( md, P_FLOAT_EPS, hull_time );
		IVP_IF(ivp_check_debug_mindist(md))
		{
			ivp_message("%32s into hull, htimes %f: %f\n","insert_lazy_hull_mindist", P_DOUBLE_EPS, hull_time);
		}
	}
	else if ( !IVP_MTIS_IS_MOVING(obj1->get_movement_state()))
	{
		insert_lazy_hull_mindist( md, hull_time, P_FLOAT_EPS );
		IVP_IF(ivp_check_debug_mindist(md))
		{
			ivp_message("%32s into hull, htimes %f: %f\n","insert_lazy_hull_mindist", hull_time, P_DOUBLE_EPS);
		}
	}
	else
	{
		IVP_Core *core0 = obj0->get_core();
		IVP_Core *core1 = obj1->get_core();
		
		IVP_FLOAT speed0 = core0->max_surface_rot_speed + core0->current_speed + P_FLOAT_EPS;
		IVP_FLOAT speed1 = core1->max_surface_rot_speed + core1->current_speed + P_FLOAT_EPS;
		IVP_FLOAT speed_corr_0 = speed0 + 0.1f * speed1;
		IVP_FLOAT speed_corr_1 = speed1 + 0.1f * speed0;
		IVP_FLOAT sum_speed = speed_corr_0 + speed_corr_1;
		IVP_FLOAT factor = hull_time / sum_speed;
		insert_lazy_hull_mindist( md, factor * speed_corr_0, factor * speed_corr_1 );
		
		IVP_IF(ivp_check_debug_mindist(md))
		{
			ivp_message("%32s into hull, htimes %f: %f\n","insert_lazy_hull_mindist", factor * speed_corr_0, factor * speed_corr_1);
		}
	}
}


void IVP_Mindist_Manager::mindist_entered_phantom(IVP_Mindist *mdist){ //@@CB
  if (! mdist->is_in_phantom_set ){
    mdist->is_in_phantom_set = IVP_TRUE;
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED
    mdist->disable_halfspace_optimization = IVP_TRUE;
#endif    
    {
      IVP_Real_Object* obj0 = mdist->get_synapse(0)->get_object();
      {
	  IVP_Controller_Phantom * cp0 = obj0->get_controller_phantom();
	  if (cp0)  cp0->mindist_entered_volume(mdist);
      }
    }
    {
      IVP_Real_Object* obj1 = mdist->get_synapse(1)->get_object();
      {
	  IVP_Controller_Phantom * cp1 = obj1->get_controller_phantom();
	  if (cp1)  cp1->mindist_entered_volume(mdist);
      }
    }
  }
}

void IVP_Mindist_Manager::mindist_left_phantom(IVP_Mindist *mdist){
  if (mdist->is_in_phantom_set ){
	    
#ifdef IVP_HALFSPACE_OPTIMIZATION_ENABLED
      mdist->disable_halfspace_optimization = IVP_FALSE;
#endif      
    mdist->is_in_phantom_set = IVP_FALSE;
    {
      IVP_Synapse_Real *syn0 = mdist->get_synapse(0);
      IVP_Controller_Phantom * cp0 = syn0->get_object()->get_controller_phantom();
      if (cp0)  cp0->mindist_left_volume(mdist);
    }
    {
      IVP_Synapse_Real *syn1 = mdist->get_synapse(1);
      IVP_Controller_Phantom * cp1 = syn1->get_object()->get_controller_phantom();
      if (cp1)  cp1->mindist_left_volume(mdist);
    }
  }

}

IVP_Mindist_Manager::IVP_Mindist_Manager(IVP_Environment *i_env)
{
    P_MEM_CLEAR(this);
    this->environment = i_env;
}

IVP_Mindist_Manager::~IVP_Mindist_Manager() {
    IVP_Mindist *mdist, *mdist_next;
    
    for(mdist=this->exact_mindists; mdist; mdist=mdist_next){
	mdist_next = mdist->next;
	P_DELETE(mdist);
    }

    for(mdist=this->invalid_mindists; mdist; mdist=mdist_next){
	mdist_next = mdist->next;
	P_DELETE(mdist);
    }
}



void IVP_Mindist_Manager::recalc_exact_mindist( IVP_Mindist *mdist){
      mdist->recalc_mindist();	
      if (mdist->mindist_function == IVP_MF_COLLISION){	
	if (mdist->recalc_result == IVP_MDRR_OK ){
	  return;
	}
	mdist->exact_mindist_went_invalid(this);
	return;
      }

      // mindist function is IVP_MF_PHANTOM
      IVP_FLOAT uncertanty;

      if (mdist->recalc_result == IVP_MDRR_OK){
	if (mdist->get_length() > 0.0f){
	  if (mdist->is_in_phantom_set ){
	    this->mindist_left_phantom(mdist);
	  }
	  return;
	}
	uncertanty =   - mdist->get_length();
      }else{
	uncertanty =   mdist->sum_extra_radius;
      }

      // mindist has entered phantom
      IVP_Real_Object* obj0 = mdist->get_synapse(0)->get_object();
      IVP_Real_Object* obj1 = mdist->get_synapse(1)->get_object();

      // early exit for phantom / phantom
      if(obj0->get_controller_phantom() && obj1->get_controller_phantom())
      {
	return;
      }

      if(obj0->get_controller_phantom())
      {
        uncertanty += obj0->get_controller_phantom()->exit_policy_extra_radius;
      }
      else
      {
        uncertanty += obj1->get_controller_phantom()->exit_policy_extra_radius;
      }
      
      if(!mdist->is_recursive())
      {
	// early exit for phantom / convex test
	if (! mdist->is_in_phantom_set )
	{
	    this->mindist_entered_phantom(mdist);
	    this->remove_exact_mindist(mdist);
	    this->insert_lazy_hull_mindist( mdist, uncertanty);
	}
      } else {
#ifdef IVP_PHANTOM_FULL_COLLISION
        //obj1 has a root convex hull
	mdist->exact_mindist_went_invalid(this);
#else
	// early exit for phantom / convex hull test
	if (! mdist->is_in_phantom_set )
	{
	    this->mindist_entered_phantom(mdist);
	    this->remove_exact_mindist(mdist);
	    this->insert_lazy_hull_mindist( mdist, uncertanty);
	}		
#endif
      }
}








