// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PROTECTED

#ifndef _IVP_MINDIST_INTERN_INCLUDED
#define _IVP_MINDIST_INTERN_INCLUDED

#ifndef WIN32
#	pragma interface
#endif


class IVP_Compact_Edge;

#ifndef IVP_FVECTOR_INCLUDED
#	include  "ivu_fvector.hxx"
#endif

#ifndef _IVP_MINDIST_INCLUDED
#	include <ivp_mindist.hxx>
#endif

//#define IVP_MINDIST_BEHAVIOUR_DEBUG	// do core as soon as possible

#if defined(DEBUG) && defined(IVP_MINDIST_BEHAVIOUR_DEBUG)
#	define DEBUG_CHECK_LEN
#endif

#define IVP_MAX_MINIMIZE_BEFORE_HASH_CHECK 20
#define IVP_MAX_PIERCINGS 2
#define IVP_LOOP_HASH_SIZE 20
#define IVP_MAX_PIERCE_CNT 0xFFFE

#define IVP_MIN_TERMINATION_QDLEN 1E-3f		// generates core
#define IVP_MIN_TERMINATION_QDLEN_EPS 1E-6f	// generates warning

class IVP_Compact_Edge;
class IVP_Environment;
class IVP_Collision;

class IVP_Synapse_Real;
class IVP_Contact_Point;
class IVP_Actuator;
class IVP_Compact_Ledge;
class IVP_Mindist_OO_Watcher;
class IVP_Mindist;
class IVP_OO_Watcher;

#define IVP_MAX_STEPS_FOR_COLLDIST_DECREASE 64

extern class IVP_Mindist_Settings{
public:
    // mindist
    IVP_FLOAT real_coll_dist;		   // very absolute bound of distance
    IVP_FLOAT min_coll_dists;              // the minimum of all call dists
    IVP_FLOAT coll_dists[IVP_MAX_STEPS_FOR_COLLDIST_DECREASE];			// Minimal distance between objects before collision occurs [meter]

    // friction
    IVP_FLOAT minimum_friction_dist;       // for friction checks for impact
    IVP_FLOAT friction_dist;		   // distance for friction
    IVP_FLOAT keeper_dist;		   // try to hold objects with a simple spring
    IVP_FLOAT speed_after_keeper_dist;     // speed when falling from keeper_dist to coll_dist
    IVP_FLOAT distance_keepers_safety;     // safety gap, when surpassed mindist doesnt appear in complex
    IVP_FLOAT max_distance_for_friction;	// when to through away friction mindist
    IVP_FLOAT max_distance_for_impact_system; //check friction mindinst for impact when in this range

    // collision
    IVP_FLOAT mindist_change_force_dist;	// in [meter]
    IVP_FLOAT min_vertical_speed_at_collision;  //
    // collision is assumed when distance is < coll_dist + mindist_change_force_dist

	// spawned mindists for recursive hulls @@CB
	int max_spawned_mindist_count;

	// base size for event queue delta time stepping @@CB
	IVP_DOUBLE event_queue_min_delta_time_base;

    void set_collision_tolerance(IVP_DOUBLE = 0.01f, IVP_DOUBLE = 9.81f); //lwss: add extra double
	void set_event_queue_min_delta_time_base(IVP_DOUBLE); //@@CB
    IVP_Mindist_Settings();
} ivp_mindist_settings;



/********************************************************************************
 *	Name:	  	IVP_Collision_Delegator_Root_Sphere     	
 *	Description:	Simple root collision delegator which generates functions
 *			useable for convex subparts
 ********************************************************************************/
class IVP_Collision_Delegator_Root_Mindist : public IVP_Collision_Delegator_Root {
public:
  virtual void object_is_removed_from_collision_detection(IVP_Real_Object *);
  virtual IVP_Collision *delegate_collisions_for_object(IVP_Real_Object *base_object, IVP_Real_Object *colliding_element);

  virtual void environment_is_going_to_be_deleted_event(IVP_Environment *env);
  virtual void collision_is_going_to_be_deleted_event(class IVP_Collision *t);
  ~IVP_Collision_Delegator_Root_Mindist();
  IVP_Collision_Delegator_Root_Mindist();
};



/********************************************************************************
 *	Name:	  	IVP_Synapse_OO     	
 *	Description:	Synapses for the IVP_OO_Watcher
 ********************************************************************************/
class IVP_Synapse_OO: public IVP_Listener_Hull {
    friend class IVP_OO_Watcher;
    IVP_Real_Object *object;
    IVP_OO_Watcher *watcher;
    virtual ~IVP_Synapse_OO();
    IVP_Synapse_OO(){;};
  void init_synapse_oo(IVP_OO_Watcher *, IVP_Real_Object *);
public:
    IVP_HULL_ELEM_TYPE get_type(){ return IVP_HULL_ELEM_OO_WATCHER; };
    void hull_limit_exceeded_event(IVP_Hull_Manager *hull_manager, IVP_HTIME);
    void hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *);		// function have to remove itself from hull manager
};



/********************************************************************************
 *	Name:	  	IVP_OO_Watcher     	
 *	Description:	Manages a hash of IVP_Mindist_OO_Watcher
 ********************************************************************************/
class IVP_OO_Watcher : public IVP_Collision, public IVP_Collision_Delegator {
    IVP_Synapse_OO  synapses[2];
    IVP_U_FVector<IVP_Collision>  mindists;
protected:
    void get_objects( IVP_Real_Object *objects_out[2] );
    void get_ledges( const IVP_Compact_Ledge *ledges_out[2] );

    IVP_Synapse_OO *get_synapse(int i){ return &synapses[i]; };
public:
    // IVP_Collision_Delegator
    void collision_is_going_to_be_deleted_event(class IVP_Collision *t);
    
    void hull_limit_exceeded_event();
    void hull_manager_is_going_to_be_deleted_event ();
    virtual ~IVP_OO_Watcher();	    
    IVP_OO_Watcher(IVP_Collision_Delegator *del, IVP_Real_Object *obj0, IVP_Real_Object *obj1);
};

/********************************************************************************
 *	Name:	  	IVP_Synapse_Real     	
 *	Description:	Adds additional functionality to IVP_Synapse,
 *	Attention:	No additional storage elements allowed in IVP_Synapse_Real
 ********************************************************************************/
class IVP_Synapse_Real: public IVP_Synapse {
protected:
  friend class IVP_Mindist;
  friend class IVP_Mindist_Manager;
  friend class IVP_Contact_Point;
public:

protected:

  // debug
  void check_consistency_of_ledge(const IVP_Compact_Edge *edge) const;

  inline void remove_exact_synapse_from_object();
  inline void insert_exact_synapse_in_object();

  inline void remove_invalid_synapse_from_object();
  inline void insert_invalid_synapse_in_object(); 

  virtual ~IVP_Synapse_Real(){;};
public:
  inline void  update_synapse(const IVP_Compact_Edge *e, IVP_SYNAPSE_POLYGON_STATUS s){
    IVP_IF(s!=IVP_ST_BALL){	check_consistency_of_ledge(e);    }
    edge = e;
    status = s;
}
protected:
    IVP_Synapse_Real(){};
    void init_synapse( IVP_Mindist *min, IVP_Real_Object *object_to_link, const IVP_Compact_Edge *e, IVP_SYNAPSE_POLYGON_STATUS s ){
	IVP_Synapse::init_synapse_real((IVP_Mindist_Base *)min, object_to_link);
	update_synapse(e,s);};
public:
  inline IVP_Core *get_core()const{ return l_obj->physical_core; };

  virtual void print();

  IVP_Hull_Manager *get_hull_manager(){ return get_object()->get_hull_manager();};
  inline IVP_DOUBLE insert_in_hull_manager(IVP_DOUBLE rel_hull_time); // returns current center pos
  inline IVP_DOUBLE insert_lazy_in_hull_manager(IVP_DOUBLE rel_hull_time);
  
  IVP_Synapse_Real       *get_next(){ return (IVP_Synapse_Real *)next;};
  IVP_Synapse_Real       *get_prev(){ return (IVP_Synapse_Real *)prev;};
  IVP_Mindist            *get_mindist(){ return (IVP_Mindist *) get_synapse_mindist(); };
};


// Minimal Dist Recalc types
enum IVP_MRC_TYPE {
    IVP_MRC_UNINITIALIZED = 0,
    IVP_MRC_OK = 1,
    IVP_MRC_ENDLESS_LOOP=2,
    IVP_MRC_BACKSIDE=3,
    IVP_MRC_ALREADY_CALCULATED=4,	// see status for details
    IVP_MRC_ILLEGAL=5
};

enum IVP_MINDIST_EVENT_HINT {
    IVP_EH_NOW,
    IVP_EH_SMALL_DELAY,
    IVP_EH_BIG_DELAY
};

class IVP_Mindist: public IVP_Mindist_Base // 
{
    friend class IVP_Mindist_Manager;
    friend class IVP_Contact_Point;
    friend class IVP_Mindist_Event_Solver;
    friend class IVP_Mindist_Minimize_Solver;
    friend class IVP_Synapse;
    friend class IVP_Synapse_Real;
protected:    
    IVP_Time_CODE recalc_time_stamp;	// compared to env->get_current_time_code
public:
    IVP_Mindist *next;	// in mindist_manager (exact , no hull nor sphere)
    IVP_Mindist *prev;
    const IVP_Compact_Edge *last_visited_triangle;  // optimization for cars
protected:
    virtual void mindist_rescue_push();

    void mindist_hull_limit_exceeded_event(IVP_HTIME hull_intrusion_value);
    void hull_manager_is_reset(IVP_FLOAT dt,IVP_FLOAT center_dt);
public:
  IVP_Synapse_Real *get_synapse(int i) const { return (IVP_Synapse_Real *)&synapse[i]; };
  IVP_Synapse_Real *get_sorted_synapse(int i) const { return (IVP_Synapse_Real *)&synapse[synapse_sort_flag ^ i]; };
    IVP_DOUBLE get_coll_dist(){ return ivp_mindist_settings.coll_dists[coll_dist_selector]; }
				    
    IVP_Mindist(IVP_Environment *env, IVP_Collision_Delegator *del);
  virtual ~IVP_Mindist();

  virtual IVP_BOOL is_recursive() { return IVP_FALSE; }; //@@CB

    IVP_Environment *get_environment(){ return get_synapse(0)->get_object()->get_environment();};
    
    /********************************************************************************
     *	Name:	    	init_mindist  	
     *	Description:	part of the constructor
     ********************************************************************************/
  void init_mindist(IVP_Real_Object *pop0, IVP_Real_Object *pop1,const IVP_Compact_Edge *e0,const  IVP_Compact_Edge *e1);
  
    /********************************************************************************
     *	section:	debug 	
     ********************************************************************************/
    void print(const char *text);

    void simulate_time_event(IVP_Environment *env);

    /********************************************************************************
     *	Name:	    	recalc_mindist  	
     *	Description:	finds the shortest distance between two convex ledges
     ********************************************************************************/
    IVP_MRC_TYPE recalc_mindist();

    /********************************************************************************
     *	Name:	    	recalc_invalid_mindist  	
     *	Description:	tries to reactivate invalid mindists (same as recalc_mindist
     * 			but optimized for invalid baviour
     ********************************************************************************/
    IVP_MRC_TYPE recalc_invalid_mindist();

    virtual void exact_mindist_went_invalid(IVP_Mindist_Manager *mm);
    
    /********************************************************************************
     *	Name:	    	update_exact_mindist_events  	
     *	Description:	given a correctly calculated mindist, this
     *			function looks into the future and generates events.
     *	Note:		needs recalc_mindist to be called before
     ********************************************************************************/
    void update_exact_mindist_events(IVP_BOOL allow_hull_conversion, IVP_MINDIST_EVENT_HINT allow_events_at_now);			// after recalc_mindist check for collision
    
    virtual void do_impact(); // does everything, including recalculation of everything, PSI synchronization etc ...
    
    /********************************************************************************
     *	Name:	    	try_to_generate_managed_friction  	
     *	Description:	looks for an is_same_as contact point
     *			If not found creates a copy of this
     *	Note:		This function has lot's of side effects, nevertheless
     *			it's quite robust
     ********************************************************************************/
    IVP_Contact_Point *try_to_generate_managed_friction(IVP_Friction_System **associated_fs,IVP_BOOL *having_new,IVP_Simulation_Unit *sim_unit_not_destroy,IVP_BOOL call_recalc_svals);

	// in case a ball is involved in mindist and this ball is attached to a car,
	//     this function is called to create a Contact_Point before a impact appears.
	//     the Contact_Point is fed with the correct friction values of an already existing Contact_Point
    void create_cp_in_advance_pretension(IVP_Real_Object *robject,float gap_len);
};

enum IVP_MINDIST_RECURSIVE_TYPES {
    IVP_MR_NORMAL= -1,
    IVP_MR_FIRST_SYNAPSE_RECURSIVE = 0,
    IVP_MR_SECOND_SYNAPSE_RECURSIVE = 1
};

class IVP_Mindist_Recursive: public IVP_Mindist, public IVP_Collision_Delegator {
    void delete_all_children();
    virtual void collision_is_going_to_be_deleted_event(class IVP_Collision *t);  // should remove t from its internal structures
    void recheck_recursive_childs(IVP_DOUBLE hull_dist_intra_object);
    void invalid_mindist_went_exact();
public:
    virtual void mindist_rescue_push();
    void rec_hull_limit_exceeded_event();
    virtual void exact_mindist_went_invalid(IVP_Mindist_Manager *mm);

    IVP_MINDIST_RECURSIVE_TYPES recursive_status;
    IVP_U_FVector<IVP_Collision> mindists;
	//@@CB
	int spawned_mindist_count;

	void change_spawned_mindist_count(int change);
	int get_spawned_mindist_count();

	virtual IVP_BOOL is_recursive() { return IVP_TRUE; };
	//@@CB

    virtual void do_impact(); // recursive check for sub mindists or IVP_Mindist::do_impact
  
    IVP_Mindist_Recursive(IVP_Environment *env, IVP_Collision_Delegator *del);
    ~IVP_Mindist_Recursive();
};



/********************************************************************************
 *	Name:	    	IVP_Mindist_Manager  	
 *	Description:	manages all exact mindists, hull_mindists are handled by
 *			the IVP_Hull _Manager
 ********************************************************************************/
class IVP_Mindist_Manager
{
    friend class IVP_Real_Object;
    IVP_BOOL	scanning_universe;			// set to true during queries to the IVP_Universe_Manager
public:
    IVP_Environment *environment; // backlink, to be set, for objects and phys properties
    
    IVP_Mindist *exact_mindists;
    IVP_U_Vector<IVP_Mindist> wheel_look_ahead_mindists;
    
    IVP_Mindist *invalid_mindists;
    
    /********************************************************************************
     *	Name:	    	create_exact_mindists  	
     *	Description:	generates all mindists between two objects
     *	Gen Types:	default_mindist_to_use or
     *			class IVP_Mindist_OO_Watcher (needs IVP_Mindist_OO_Watcher);
     ********************************************************************************/
    static void create_exact_mindists(IVP_Real_Object *obj0, IVP_Real_Object *obj1,
				      IVP_DOUBLE scan_radius,
				      IVP_U_FVector<IVP_Collision> *existing_collisions,
				      const IVP_Compact_Ledge *single_ledge0, const IVP_Compact_Ledge *single_ledge1,
				      const IVP_Compact_Ledge *root_ledge0, const IVP_Compact_Ledge *root_ledge1,
				      IVP_Collision_Delegator *del);

    

    void insert_exact_mindist(IVP_Mindist *new_mindist);	// insert in linked list only
    void insert_and_recalc_exact_mindist(IVP_Mindist *new_mindist);	// insert no phantoms in linked list and recalc and update_events
    void insert_and_recalc_phantom_mindist(IVP_Mindist *new_mindist);	// insert phantoms in linked list and recalc and update_events
    void remove_exact_mindist(IVP_Mindist *del_mindist);

    void insert_invalid_mindist(IVP_Mindist *new_mindist);	// insert in linked list only
    void remove_invalid_mindist(IVP_Mindist *del_mindist);

    static void mindist_entered_phantom(IVP_Mindist *mdist);
    static void mindist_left_phantom(IVP_Mindist *mdist);

    void remove_hull_mindist(IVP_Mindist *del_mindist);
    static void insert_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time0, IVP_HTIME hull_time1);
    static void insert_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time);
    
    static void insert_lazy_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time0, IVP_HTIME hull_time1);
    static void insert_lazy_hull_mindist(IVP_Mindist *md, IVP_HTIME hull_time);

    void recheck_ov_element(IVP_Real_Object *object);	// recheck the ov_tree


	void recalc_exact_mindist( IVP_Mindist *mdist);
	void recalc_all_exact_mindists();
	void recalc_all_exact_wheel_mindist();
    
    void recalc_all_exact_mindists_events();
  //void recalc_all_invalid_mindists();


    void enable_collision_detection_for_object(IVP_Real_Object *); // creates ov element for object
    
    void print_mindists(); // instead of graphic
    
    IVP_Mindist_Manager(IVP_Environment *i_env);
    ~IVP_Mindist_Manager();    
};



#endif













