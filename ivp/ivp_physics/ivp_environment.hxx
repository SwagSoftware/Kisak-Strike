// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_environment.hxx	
 *	Description:	???
 *	Attention:	stdio.h must be included before.
 ********************************************************************************/

#ifndef _IVP_ENVIRONMENT_INCLUDED
#define _IVP_ENVIRONMENT_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#define IVP_Environment_Magic_Number 123456
#define IVP_MOVEMENT_CHECK_COUNT 10 //do not always check movement state

class IVP_BetterStatisticsmanager;
class IVP_Sim_Units_Manager;
class IVP_OV_Tree_Manager;
class IVP_Collision_Filter;
class IVP_Range_Manager;
class IVP_Controller_Manager;
class IVP_U_Active_Value_Manager;
class IVP_Material_Manager;
class IVP_U_Memory;
class IVP_Listener_PSI;
class IVP_Environment_Manager;
class IVP_Application_Environment;
class IVP_Event_Collision;
class IVP_Event_Object;
class IVP_Debug_Manager;
class IVP_Draw_Vector_Debug;
class IVP_Debug_Manager;
class IVP_Actuator_Spring;
class IVP_Template_Spring;
class IVP_Actuator_Suspension;
class IVP_Template_Suspension;
class IVP_Template_Force;
class IVP_Template_Torque;
class IVP_Template_Rot_Mot;
class IVP_Template_Stabilizer;
class IVP_Template_Check_Dist;
class IVP_Controller_Motion;
class IVP_Actuator_Spring;
class IVP_Actuator_Force;
class IVP_Actuator_Torque;
class IVP_Actuator_Rot_Mot;
class IVP_Actuator_Stabilizer;
class IVP_Actuator_Check_Dist;
class IVP_Template_Constraint;
class IVP_Constraint;
class IVP_Collision_Delegator_Root;
class IVP_PerformanceCounter;
class IVP_Anomaly_Manager;
class IVP_Anomaly_Limits;
class IVP_Time_Manager;
class IVP_Cluster_Manager;
class IVP_Mindist_Manager;
class IVP_Cache_Object_Manager;

/********************************************************************************
 *	Name:	     	IVP_ENV_STATE	
 *	Description:	State of the IVP_Environment	
 ********************************************************************************/
enum IVP_ENV_STATE {
    IVP_ES_PSI,   	// Normal PSI controller mode
    IVP_ES_PSI_INTEGRATOR,
    IVP_ES_PSI_HULL,  	// Special PSI mode, where no forces are allowed
    IVP_ES_PSI_SHORT,
    IVP_ES_PSI_CRITIC,
    IVP_ES_AT    	// Between PSIs (Any Time), normally a collision
};



/********************************************************************************
 *	Name:	  	IVP_Statistic_Manager    	
 *	Description:	Collects statistic information
 ********************************************************************************/
class IVP_Statistic_Manager {
public:
    IVP_Environment *l_environment;
    //statistic section
    IVP_Time last_statistic_output;

    // impact section
    IVP_FLOAT max_rescue_speed;
    IVP_FLOAT max_speed_gain;
    int impact_sys_num;
    int impact_counter;		// number of total impacts
    int impact_sum_sys;		// number of system impacts (which are much faster)
    int impact_hard_rescue_counter;	// number of bad impacts (very slow movement)
    int impact_rescue_after_counter;
    int impact_delayed_counter;	// number of impacts, where one object gets a delayed speed update
    int impact_coll_checks;	// number of AT collision checks during impacts
    int impact_unmov;
    
    IVP_DOUBLE sum_energy_destr;

    // collision checks section
    int sum_of_mindists;	// number of mindists (not cleared by clear_statistic)
    int mindists_generated;	// number of mindist just created
    int mindists_deleted;	// number of just deleted mindists

    int range_intra_exceeded;   // number of events: sphere for checking two objects recalculated
    int range_world_exceeded;	// number of events: sphere for checking object - world recalculated

	// friction
    int processed_fmindists;	// number of calls to recalc_friction_s_vals
    int  global_fmd_counter;

    void clear_statistic();
    void output_statistic();

    IVP_Statistic_Manager();
};

class IVP_Freeze_Manager {
public:
    void init_freeze_manager();
    IVP_FLOAT freeze_check_dtime; // default 0.3 sec
    IVP_Freeze_Manager();
};

class IVP_Listener_Constraint
{
public:
	virtual void event_constraint_broken(IVP_Constraint *) = 0;
#ifdef HAVANA_CONSTRAINTS
	virtual void event_constraint_broken(class hk_Breakable_Constraint*) = 0;
#endif // HAVANA_CONSTRAINTS
};


/********************************************************************************
 *	Name:	  	IVP_Environment     	
 *	Description:	The physical world:
 *			The is the main class to handle simulation
 ********************************************************************************/
class IVP_Environment {		// the environment
public: //lwss
    friend class IVP_Simulation_Unit;
    friend class IVP_Friction_System;
    friend class IVP_Friction_Sys_Static;
    friend class IVP_Friction_Solver;
    friend class IVP_Friction_Core_Pair;
    friend class IVP_Impact_Solver; 		//for debug statistics only
    friend class IVP_Impact_System; 		//for debug statistics only
    friend class IVP_Impact_Solver_Long_Term; 	//for debug statistics only
    friend class IVP_Contact_Point;            // for debug statistics only
    friend class IVP_Environment_Manager;
    friend class IVP_Time_Event_PSI;
    friend class IVP_Real_Object;
    friend class IVP_Time_Manager;
    friend class IVP_Mindist_Manager;
    friend class IVP_Calc_Next_PSI_Solver;

    friend class IVP_Mindist;
    friend class IVP_Core;


    class IVP_Standard_Gravity_Controller *standard_gravity_controller;
    IVP_Time_Manager 	     *time_manager;  
	
    IVP_Sim_Units_Manager    *sim_units_manager;
    IVP_Cluster_Manager      *cluster_manager;
    IVP_Mindist_Manager      *mindist_manager;
    IVP_OV_Tree_Manager      *ov_tree_manager;
    IVP_Collision_Filter     *collision_filter;
    IVP_Range_Manager	     *range_manager;
    IVP_Anomaly_Manager	     *anomaly_manager;
    IVP_Anomaly_Limits	     *anomaly_limits;

	IVP_PerformanceCounter   *performancecounter;
    class	IVP_Universe_Manager  *universe_manager;
    class IVP_Real_Object    *static_object;		// used for anchors which are not fixed to objects

    IVP_Statistic_Manager    statistic_manager; // lwss:+104 bytes
    IVP_Freeze_Manager       freeze_manager;
    IVP_BetterStatisticsmanager *better_statisticsmanager;
    IVP_Controller_Manager   *controller_manager;
  
    IVP_Cache_Object_Manager *cache_object_manager;
    
    IVP_U_Active_Value_Manager 	*l_active_value_manager; // backlink to local environment manager
    IVP_Material_Manager *l_material_manager;
    IVP_U_Memory *short_term_mem; 		//temporary memory (use only between start_memory_transaction and end_memory_transaction)
    IVP_U_Memory *sim_unit_mem;                 //memory is cleaned when sim_unit or impact_system is finished
    
    IVP_DOUBLE time_since_last_blocking;
    
    IVP_DOUBLE delta_PSI_time;			// delta time between two PSIs ( default 1/66 seconds. range: IVP_MAX_DELTA_PSI_TIME - IVP_MIN_DELTA_PSI_TIME seconds ) //lwss: +260 bytes
    IVP_DOUBLE inv_delta_PSI_time;
    
    IVP_U_Point gravity; 	// gravity in meter/(sec*sec) see do_gravity() in core
    IVP_FLOAT gravity_scalar;       // same as above but only length
    IVP_U_Vector<IVP_Listener_Collision> 	collision_listeners;
    IVP_U_Vector<IVP_Listener_PSI> 		psi_listeners;
    IVP_U_Vector<IVP_Core> 		        core_revive_list;
    IVP_U_Vector<IVP_Listener_Constraint> 	constraint_listeners;
    
    
    char *auth_costumer_name;
    unsigned int auth_costumer_code;
    int pw_count;
    
    IVP_Environment(IVP_Environment_Manager *manager,IVP_Application_Environment *appl_env,const char *costumer_name,unsigned int auth_code);
    IVP_Environment_Manager *environment_manager;
    void simulate_psi(IVP_Time psi_time);

    inline IVP_BOOL must_perform_movement_check() {
		next_movement_check--;

		if(next_movement_check==0) {
			next_movement_check=IVP_MOVEMENT_CHECK_COUNT*3/2 + (short)(ivp_rand() * (IVP_MOVEMENT_CHECK_COUNT/2));
			return IVP_TRUE;
		} else {
			return IVP_FALSE;
		}
	};

	void add_revive_core(IVP_Core *r_obj);
    void remove_revive_core(IVP_Core *r_obj);
    void revive_cores_PSI(); // at the beginning of PSI: revive cores
    
    IVP_Time current_time;
    IVP_Time time_of_next_psi; //lwss: +376
    IVP_Time time_of_last_psi;	// not for public usage
    IVP_Time_CODE current_time_code;
    IVP_Time_CODE mindist_event_timestamp_reference;	
    short next_movement_check;
    
    IVP_ENV_STATE state;

    void set_current_time(IVP_Time time);
    void do_d_events();
    // INTERN_END
    
#ifdef DEBUG
    void invalid_time_code(){ current_time_code++;};	// invalid current time code
#endif
    void fire_event_pre_collision( IVP_Event_Collision *);
    void fire_event_post_collision( IVP_Event_Collision *);
    void fire_event_friction_created( class IVP_Event_Friction *coll);
    void fire_event_friction_deleted( class IVP_Event_Friction *coll);
    void fire_event_friction_pair_created( class IVP_Friction_Core_Pair *pair);
    void fire_event_friction_pair_deleted( class IVP_Friction_Core_Pair *pair);
    
    void fire_event_PSI( );

    //some PSI grid dependent constants
    IVP_DOUBLE integrated_energy_damp;
    IVP_DOUBLE get_integrated_energy_damp() { return integrated_energy_damp; };
    /***********************************************************************************
     *			Internal public functions, handle with care:
     ***********************************************************************************/
public:
    void set_event_function();
    IVP_U_Vector<IVP_Listener_Object> global_object_listeners;
    IVP_U_Vector<class IVP_Collision_Delegator_Root> collision_delegator_roots;
    
    void fire_event_object_created(IVP_Event_Object *);
    void fire_event_object_deleted(IVP_Event_Object *);
    void fire_event_object_frozen (IVP_Event_Object *);
    void fire_event_object_revived(IVP_Event_Object *);
    void fire_object_is_removed_from_collision_detection(IVP_Real_Object *o);
	void fire_event_constraint_broken(IVP_Constraint *);
#ifdef HAVANA_CONSTRAINTS
	void fire_event_constraint_broken(class hk_Breakable_Constraint*);
#endif // HAVANA_CONSTRAINTS

    ////////// debugging
    IVP_Debug_Manager *debug_information;
    IVP_BOOL delete_debug_information;

    IVP_Draw_Vector_Debug *draw_vectors;
    void add_draw_vector(const IVP_U_Point *start_ws,const IVP_U_Float_Point *vec,const char *debug_text,int color);
    void delete_draw_vector_debug();    
    
    //////////// time management
    IVP_Time get_next_PSI_time(){ return time_of_next_psi; } //lwss + 376
    IVP_Time get_old_time_of_last_PSI() { return time_of_last_psi; };

    IVP_FLOAT get_delta_PSI_time() { return delta_PSI_time; }
    IVP_FLOAT get_inv_delta_PSI_time() { return inv_delta_PSI_time; }

    IVP_Time_CODE get_current_time_code(){ return current_time_code; }

    //lwss add
    void force_psi_on_next_simulation();
    //lwss end
    IVP_ENV_STATE get_env_state() { return state; };	// return state;

    //////// private managers
    IVP_Time_Manager		 *get_time_manager() const	        { return time_manager;};
    IVP_Controller_Manager       *get_controller_manager() const        { return controller_manager; }; 
    IVP_Mindist_Manager		 *get_mindist_manager() const	        { return mindist_manager; };
    IVP_Sim_Units_Manager        *get_sim_units_manager() const         { return sim_units_manager; };

    IVP_Cache_Object_Manager	 *get_cache_object_manager() const      { return cache_object_manager; };
    IVP_OV_Tree_Manager          *get_ov_tree_manager() const           { return ov_tree_manager; };
    IVP_Cluster_Manager          *get_cluster_manager() const           { return cluster_manager; };
    IVP_Debug_Manager		 *get_debug_manager() const             { return debug_information; };
    IVP_U_Memory		 *get_memory_manager() const	        { return short_term_mem; };
    IVP_Cluster                  *get_root_cluster();
    IVP_PerformanceCounter	*get_performancecounter()  		{ return performancecounter; };

    IVP_U_Memory *get_sim_unit_mem(){ return sim_unit_mem; };
    IVP_U_Memory *get_short_term_mem(){ return short_term_mem; };






    /***********************************************************************************
     *			real public functions:
     ***********************************************************************************/

    //////////// public managers
    IVP_Range_Manager		 *get_range_manager() const 			{ return range_manager; };
    IVP_Material_Manager 	 *get_material_manager() const			{ return l_material_manager; };
    IVP_Collision_Filter	 *get_collision_filter() const			{ return collision_filter; };
    IVP_Statistic_Manager	 *get_statistic_manager()			{ return &statistic_manager; };
    IVP_BetterStatisticsmanager	 *get_betterstatisticsmanager()			{ return better_statisticsmanager; };
    IVP_U_Active_Value_Manager	 *get_active_value_manager() const		{ return l_active_value_manager; };
    IVP_Universe_Manager 	 *get_universe_manager() const			{ return universe_manager; };
    IVP_Anomaly_Manager		 *get_anomaly_manager() const                   { return anomaly_manager; }
    IVP_Anomaly_Limits		 *get_anomaly_limits() const                   { return anomaly_limits; }

    IVP_Real_Object	         *get_static_object() const                     { return static_object; }; // a env. global static ball
    IVP_Freeze_Manager           *get_freeze_manager()                          { return &freeze_manager; };
    //////////// time management
    IVP_Time get_current_time(){ return current_time; }
    void 	set_delta_PSI_time( IVP_DOUBLE new_delta_PSI_time );		// range: IVP_MIN_DELTA_PSI_TIME - IVP_MAX_DELTA_PSI_TIME

    //lwss add
    float get_global_collision_tolerance();
    // added gravLen arg
    void set_global_collision_tolerance( IVP_DOUBLE tolerance = 0.01f, IVP_DOUBLE gravLen = 9.81f );   // set the collision tolerance (try to go for higher values if possible)
    //lwss end

    ////////// world creation

    // this is preleminary: allows to define breakable objects.
    // to create fixed complex objects, see the IVP_SurfaceBuilder_Ledge_Soup classes
    // Note: merge objects before collision detection is enabled and objects are revived !!!
    //       you may use IVP_Object_Attach::xxxx()  instead
    void merge_objects(IVP_U_Vector<IVP_Real_Object> *obj_to_merge);


    /********************************************************************************
     *	Name:	  	create_polygon /    create_ball 	
     *	Description:	Creates objects
     *	Note:		Objects are not collision detection enabled:
     *				to enable collision detection call
     *				IVP_Real_Object::enable_collision_detection
     *			Objects are not simulated when created. Simulation is started
     *			when objects collide or IVP_Real_Object::ensure_in_simulation() is called
     ********************************************************************************/
    IVP_Polygon *create_polygon(IVP_SurfaceManager *surface_manager,	// a surface_manager manager
				const IVP_Template_Real_Object *templ_obj,	// used to set mass center and rotation inertia directly (no automatism)
				const IVP_U_Quat *quat_world_f_object,	// the rotation of the object
				const IVP_U_Point *position);		// the position
    
    IVP_Ball *create_ball(	const IVP_Template_Ball *templ_ball,		// the radius
				const IVP_Template_Real_Object *templ_obj,	// used to set mass center and rotation inertia directly (no automatism)
				const IVP_U_Quat *quat_world_f_object,	// the rotation of the object
				const IVP_U_Point *position);		// the position
    

    // simple (predefined) actuators
    IVP_Actuator_Spring  *create_spring(IVP_Template_Spring *templ);     // creates a damped spring, Note IVP_Controller_Stiff_Spring are better
    IVP_Actuator_Suspension *create_suspension(IVP_Template_Suspension *templ); // creates a special vehicle spring
 
    IVP_Actuator_Force   *create_force(IVP_Template_Force *templ);       // creates a force actuator
    IVP_Actuator_Torque  *create_torque(IVP_Template_Torque *templ);     // creates a torque actuator
    IVP_Actuator_Rot_Mot *create_rotmot(IVP_Template_Rot_Mot *templ);    // creates a motor

    IVP_Actuator_Stabilizer   *create_stabilizer(IVP_Template_Stabilizer *templ);       // used for cars
    IVP_Actuator_Check_Dist   *create_check_dist(IVP_Template_Check_Dist *templ);       // creates a distance checker

    ////////// create constraints
    // for mor info on constraints look at ivp_template_constraint.hxx
    IVP_Constraint *create_constraint(const IVP_Template_Constraint *template_constraint);

    IVP_Controller_Motion *create_controller_motion(IVP_Real_Object *, const class IVP_Template_Controller_Motion *);

    const IVP_U_Point *get_gravity(){ return &gravity; };		// current gravity vector, default (0,9.83f,0);
    void set_gravity(IVP_U_Point *gravity);				// sets gravity vector (can be changed at any time)
	class IVP_Standard_Gravity_Controller *get_gravity_controller( void ) { return standard_gravity_controller; }
    
    /////////// global listening (private listening: see IVP_Real_Object)
    void add_listener_object_global(IVP_Listener_Object *); // adds a callback which listens to events of all objects
    void install_listener_object_global(IVP_Listener_Object *); // adds a callback which listens to events of all objects if its not in the list already
    void remove_listener_object_global(IVP_Listener_Object *);
    
    void add_listener_object_private(IVP_Real_Object *, IVP_Listener_Object *);		// adds a callback which listens to object events of one object
    void remove_listener_object_private(IVP_Real_Object *, IVP_Listener_Object *);	// (same as IVP_Real_Object::add_listener_object())

    void add_listener_collision_global(IVP_Listener_Collision *);	// adds a callback which listens to all collisions
    void remove_listener_collision_global(IVP_Listener_Collision *);

    void add_listener_collision_private(IVP_Real_Object *, IVP_Listener_Collision *);	// adds a callback which listens to just one object
    void remove_listener_collision_private(IVP_Real_Object *, IVP_Listener_Collision *);// (note: same as IVP_Real_Object::add_listener_collision())

    void add_listener_PSI(IVP_Listener_PSI *);			// adds a callback called at the beginning of every PSI
    void remove_listener_PSI(IVP_Listener_PSI *);

	void add_listener_constraint_global(IVP_Listener_Constraint *);
	void remove_listener_constraint_global(IVP_Listener_Constraint *);

    /////////// simulation
    void simulate_until(IVP_Time time);				// the MAIN function: simulates the entire physical environment until 'time'
    void simulate_dtime(IVP_DOUBLE deta_time);			// the MAIN function: simulates the entire physical for a given time
    void simulate_time_step( IVP_FLOAT sub_psi_pos = 1.99 );    // simulate exactly one time step (for synchronous use of the physics engine (e.g. playstation 2).

    void simulate_variable_time_step( IVP_FLOAT delta_time );    // simulate exactly one time step with a variable length
                                                                 // delta_time has to be in between 0.1 and 0.005 

    void reset_time();                                          // resets the time to zero

    void *client_data; //store pointer to custom datastructure

    /////////// construct/destruct: constructor see: IVP_Environment_Manager
    ~IVP_Environment();

    int environment_magic_number; //to detect if environment is valid (debug reasons)    
};

/********************************************************************************
 *	Name:	     	IVP_Application_Environment  	
 *	Description:	The Ipion Virtual Physics Engine users should provide
 *			an instance of IVP_Application_Environment to
 *			handle different events during simulation.
 *	Attention:	All values have to be set.
 ********************************************************************************/
class IVP_Application_Environment {
public:
    int	n_cache_object;		  // == 2**x number of caches for moving objects, should be larger than typical number of moving objects

    char *scratchpad_addr;        // scratch pad: a fast area which is temporarily used and is already cached in
    int scratchpad_size;

    /********************************************************************************
     *	Name:	     	Several custom managers  	
     *	Description:	To override basic functionality of the Ipion Physics Engine
     *               	you may replace some of it's internal managers by your
     *                	own version
     ********************************************************************************/
    IVP_Material_Manager     *material_manager; 	// a manager which serves friction and elasticity coefficients
    IVP_Collision_Filter     *collision_filter;		// a filter that checks which objects should be watched for collisions
    IVP_Universe_Manager     *universe_manager;		// see IVP_Universe_Manager
    IVP_PerformanceCounter   *performancecounter;       //
    IVP_Anomaly_Manager	     *anomaly_manager;
    IVP_Anomaly_Limits	     *anomaly_limits;

    IVP_Collision_Delegator_Root *default_collision_delegator_root; // an optional user defined default root collision delegator

    IVP_U_Active_Value_Manager *env_active_float_manager;	// a hash of active_values
    IVP_Range_Manager *range_manager;			// optional custom range_manager

    
    IVP_Application_Environment();
};

class IVP_Vector_of_Hulls_128: public IVP_U_Vector<IVP_Hull_Manager_Base> {
    IVP_Hull_Manager_Base *elem_buffer[128];
public:
    IVP_Vector_of_Hulls_128(): IVP_U_Vector<IVP_Hull_Manager_Base>((void **)&elem_buffer[0], 128){;};
};

class IVP_Vector_of_Cores_128: public IVP_U_Vector<IVP_Core> {
    void *elem_buffer[128];
public:
    IVP_Vector_of_Cores_128(): IVP_U_Vector<IVP_Core>(&elem_buffer[0], 128){;};
};


/********************************************************************************
 *	Name:	    	IVP_Environment_Manager  	
 *	Description:	Factory for IVP_Environments
 *	Note:		When creating an environment, create a fully initialized
 *			IVP_Application_Environment first.
 ********************************************************************************/
class IVP_Environment_Manager {
    friend class IVP_Environment;
private:
    static IVP_Environment_Manager static_environment_manager;
    IVP_Environment_Manager();
public:
    ~IVP_Environment_Manager();
    int ivp_willamette_optimization;


    IVP_U_Vector<IVP_Environment> environments;
    IVP_Environment *create_environment(IVP_Application_Environment *appl_env,const char *costumer_name,unsigned int auth_code);
    static IVP_Environment_Manager* IVP_CDECL get_environment_manager();			
};

#endif


