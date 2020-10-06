// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_real_object.hxx	
 *	Description:	???
 ********************************************************************************/

#ifndef _IVP_REAL_OBJECT_INCLUDED
#define _IVP_REAL_OBJECT_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_U_MINLIST_INCLUDED
#	include <ivu_min_list.hxx>
#endif
// resolve forward references 
class IVP_Template_Phantom;
class IVP_SurfaceManager;
class IVP_Template_Real_Object;
class IVP_Material;
class IVP_OV_Element;
class IVP_OV_Element;
class IVP_Listener_Collision;
class IVP_Listener_Object;
class IVP_Radar;
class IVP_Template_Anchor;
class IVP_Hull_Manager;
class IVP_Synapse_Friction;
class IVP_Anchor;

#define IVP_NO_COLL_GROUP_STRING_LEN 8

/********************************************************************************
 *	Name:	    	IVP_Movement_Type   	
 *	Description:	Indicates the coarse speed status of an object
 *	Attention:	Order is important, see macros
 ********************************************************************************/
enum IVP_Movement_Type {
    IVP_MT_UNDEFINED = 0,
    IVP_MT_MOVING = 0x01,	  // fast 
    IVP_MT_SLOW = 0x02,	          // slow
    IVP_MT_CALM = 0x03, 	  // slow for more than a certain time span (currently 1 second)
    IVP_MT_NOT_SIM =0x08,	  // not simulated, but can be changed to be simulated
    IVP_MT_STATIC_PHANTOM = 0x09, // like static, but object may be moved by setting the matrix directly
    IVP_MT_STATIC = 0x10,	  // static object
    IVP_MT_GET_MINDIST = 0x21     // not really a movement type, used to get all mindists with recheck_ov_element
};

#define IVP_MTIS_SIMULATED(var) ( var < IVP_MT_NOT_SIM )
#define IVP_MTIS_CAN_COLLIDE(var) ( var & 0x07 )
#define IVP_MTIS_IS_MOVING(var) (  var & 0x07 )
#define IVP_MTIS_IS_STATIC(var) ( var & 0x10 )

class IVP_Hull_Manager_Base_Gradient {
public:
    IVP_Time last_vpsi_time;
    IVP_FLOAT gradient;		    // slightly higher than the real gradient 
    IVP_FLOAT center_gradient;  
    IVP_FLOAT hull_value_last_vpsi;        
    IVP_FLOAT hull_center_value_last_vpsi;    
    IVP_FLOAT hull_value_next_psi;	// optimistic, may be less
    int time_of_next_reset;  		// counting seconds only
public:
    IVP_Hull_Manager_Base_Gradient() : last_vpsi_time (0.0f) {; };
    ~IVP_Hull_Manager_Base_Gradient(){;};
};

class IVP_Hull_Manager_Base: protected IVP_Hull_Manager_Base_Gradient {
protected:
    // Manages a sorted event list of synapses for one Real_Object **/
    IVP_U_Min_List sorted_synapses;
public:
    IVP_Hull_Manager_Base();
    ~IVP_Hull_Manager_Base();
};


/*********************************************************************
 *	Name:	    	IVP_Real_Object  	
 *	Description:	Base of all real objects.
 *			Real object means that this object has a mass,
 *			can collide, etc.
 *********************************************************************/
class IVP_Real_Object_Fast_Static: public IVP_Object {
protected:
    class IVP_Controller_Phantom *controller_phantom;
    class IVP_Synapse_Real *exact_synapses;		// Linked list of all exact synapses, no sphere nor hull.
    IVP_Synapse_Real     *invalid_synapses;         // Linkes list of all intruded synapses
    IVP_Synapse_Friction *friction_synapses;	// Linked list of contact points.
    IVP_U_Quat	         *q_core_f_object;	 // in object !!!!
    IVP_U_Float_Point 	 shift_core_f_object;
    IVP_Real_Object_Fast_Static(IVP_Cluster *father, const IVP_Template_Object *templ): IVP_Object( father, templ){;};
public:
    const IVP_U_Float_Point *get_shift_core_f_object() const { return &shift_core_f_object; };
};

// add dynamics
class IVP_Real_Object_Fast: public IVP_Real_Object_Fast_Static {
protected:
    class IVP_Cache_Object *cache_object;		// Intermediate caches
    IVP_Hull_Manager_Base hull_manager; 		// Internal usage
public:
    struct {
	IVP_Movement_Type  object_movement_state:8; // moving, not_simulated, static only
	IVP_BOOL      	collision_detection_enabled:2;	/* Collision detection is enabled. If it is disabled, not even
							 * the IVP_Collision_Filter is called */
	IVP_BOOL 	shift_core_f_object_is_zero:2;
	
	unsigned int	object_listener_exists:1;   /* is set to one if an object listener exists */
	unsigned int	collision_listener_exists:1; /* all flags of object listeners functions */
	unsigned int	collision_listener_listens_to_friction:1;
    } flags;
    IVP_Real_Object_Fast(IVP_Cluster *father, const IVP_Template_Object *templ): IVP_Real_Object_Fast_Static( father, templ){;};
};

class IVP_Real_Object: public IVP_Real_Object_Fast {

    friend class IVP_Anchor;
    friend class IVP_Core;
    friend class IVP_Friction_System;
    friend class IVP_Synapse_Real;
    friend class IVP_Cache_Object;
    friend class IVP_Cache_Object_Manager;
    friend class IVP_Synapse_Friction;
    friend class IVP_Contact_Point;
    friend class IVP_Simulation_Unit;
    friend class IVP_Controller_Phantom;
    friend class IVP_Example_Boundingboxes;
    
//private: //lwss- make public
public:
    friend class IVP_Merge_Core;
    IVP_Anchor *anchors;			// Linked list of all anchors @@@ remove anchor concept

    friend class IVP_Object_Attach;
    void unlink_contact_points(IVP_BOOL silent); //FALSE means wake up the cores
    //lwss add
    void unlink_contact_points_for_object( IVP_Real_Object *object );
    void force_grow_friction_system();
    //lwss end
    void clear_internal_references();

    friend class IVP_Mindist_Manager;
protected:    
    IVP_SurfaceManager *surface_manager;	// The surface_manager defines the surface structure for this object.
    
    /* The following section is used to calculate intermediate values */

    friend class IVP_Core_Collision;
    friend class IVP_Calc_Next_PSI_Solver;
    void update_exact_mindist_events_of_object();
    void revive_nearest_objects_grow_fs();
    void get_all_near_mindists();
    void recalc_exact_mindists_of_object();
    void recalc_invalid_mindists_of_object();
  
    virtual void set_new_quat_object_f_core( const IVP_U_Quat *new_quat_object_f_core, const IVP_U_Point *trans_object_f_core); // calls set_new_m...
public:
    virtual void set_new_m_object_f_core( const IVP_U_Matrix *new_m_object_f_core );	 /* specifies new objects coordinate system in core space */
    void init_object_core(IVP_Environment *env, const IVP_Template_Real_Object *tpop);
protected:

    IVP_Real_Object(IVP_Cluster *father,IVP_SurfaceManager *, const IVP_Template_Real_Object *,
		    const IVP_U_Quat *q_world_f_obj, const IVP_U_Point *position);


    virtual ~IVP_Real_Object();	// to delete the object use delete_and_check_vicinity or delete_silently

/********************************************************************************
 *	The physics simulation internal public section. Handle with care:
 ********************************************************************************/
public:    
    char nocoll_group_ident[IVP_NO_COLL_GROUP_STRING_LEN];	/* Identifier for filtering collisions, used only 
				 * if the IVP_Collision_Filter_Coll_Group_Ident is
				 * the installed collision filter, see IVP_Application_Environment. */
    ///////// Simulation internals
    IVP_Material 	*l_default_material;	// default material of object
    IVP_OV_Element   	*ov_element;		// used to trace objects movement within the environment->ov_tree
    IVP_FLOAT extra_radius;	                        // extra collision radius around the object

    ///////// Core 
    IVP_Core *physical_core; // Coordinate system, mass, speed, etc... (eventually merged)
    IVP_Core *friction_core; // Long term (merged) core for frictions, when objs are splitted make sure to call 'recheck_ov_element'
    IVP_Core *original_core;

    ////////// Anchors
    IVP_Anchor *get_first_anchor(){ return anchors; };
    void insert_anchor(IVP_Anchor *new_anchor);
    void remove_anchor(IVP_Anchor *destroy_anch);

    IVP_Synapse_Real *get_first_exact_synapse(){ return exact_synapses; };
    IVP_Synapse_Friction *get_first_friction_synapse(){ return friction_synapses; };
    IVP_Hull_Manager *get_hull_manager(){ return (IVP_Hull_Manager *)&this->hull_manager; };
    void reset_time( IVP_Time offset);

    void revive_object_for_simulation(); // wake up object immediately ; @@@IK don't use this - use ensure_in_simulation() instead!
    
    IVP_SurfaceManager     *get_surface_manager()const { return surface_manager; }
    IVP_OV_Element	   *get_ov_element(){ return ov_element; }

    void set_movement_state(IVP_Movement_Type mt) { flags.object_movement_state = mt; };

    IVP_FLOAT get_extra_radius() const { return extra_radius; };	// returns an extra collision radius around the object#
	void set_extra_radius( IVP_DOUBLE new_radius );
	inline IVP_Cache_Object *get_cache_object_no_lock(); // see get_cache_object, but restricted usage


    IVP_Real_Object *to_nonconst() const { return (IVP_Real_Object *)this; }; // remove const
/********************************************************************************
 *	The real end user public section:
 ********************************************************************************/
    
    void *client_data; //not used by physics, provided for customer

    
    /********************************************************************************
	 *	Name:	    	get_cache_object 	
	 *	Description:	Get a matrix that allows to transform between object and
	 *			world space. It also locks the cache element.
	 *			To unlock, call IVP_Cache_Object::remove_reference().
	 *	Note:		Include ivp_cache_object.hxx to use this method.
	 ********************************************************************************/
    inline IVP_Cache_Object *get_cache_object();

     /*************************************************************************************
	 *	Name:	    	set_new_surface_manager 	
	 *	Descripion:	sets a new geometry for this object
	 *      Note:           does not update rotation inertias etc..
	 ************************************************************************************/
    void set_new_surface_manager( IVP_SurfaceManager *);

     /*************************************************************************************
	 *	Name:	    	recalc_core_radius 	
	 *	Descripion:	updates the upper_limit_radius stored in the core
	 ************************************************************************************/
	void recalc_core_radius();

	/*************************************************************************************
	 *	Name:	    	get_geom_xxx 	
	 *	Note:		Geometric center/radius is not the same as mass center/radius!
	 ************************************************************************************/
    void get_geom_center_world_space(IVP_U_Point *center_ws_out) const;
    IVP_FLOAT get_geom_radius() const; 	// [m]
    IVP_FLOAT get_geom_center_speed() const; 	// [m/s]
    void get_geom_center_speed_vec(IVP_U_Point *speed_ws_out) const; // [m/s]

    
        /***********************************************************************************
	 *	Name:	    	calc_at_matrix 	
	 *	Description:	Get the object's matrix for the display engine at Any Time (at).
	 *	Note:		The matrix is an interpolation of two PSI matrizes.
	 **********************************************************************************/
    void calc_at_matrix(IVP_Time current_time_in, IVP_U_Matrix   *m_world_f_object_out ) const;
    void calc_at_quaternion(IVP_Time current_time_in, IVP_U_Quat *q_world_f_object_out, IVP_U_Point *position_ws_out) const;

        /***********************************************************************************
	 *	Name:	    	beam_object_to_new_position 	
	 *	Description:	Set the object's matrix
	 *      Parameter:      rotation:   the new rotation
	 *                      position:   the new position
	 *                      optimize_for_repeated_calls: set to true if you are planning
	 *                      to call this function every frame
	 *	Attention:	If the objects penetrates other objects at the new position,
	 *                      no collisions will be generated !!!
	 **********************************************************************************/
    void beam_object_to_new_position( const IVP_U_Quat *rotation, const IVP_U_Point *position, IVP_BOOL optimize_for_repeated_calls = IVP_FALSE);

    
        /****************************************************************************************************
	 *	Name:	    	get_m_world_f_object_AT 	
	 *	Description:	Same as calc_at_matrix but point of time is taken from environment.
	 *	Note:		Implementation looks like:
	 *			'calc_at_matrix(get_environment()->get_current_time(), m_world_f_object_out);'
	 ****************************************************************************************************/
    void   get_m_world_f_object_AT   (IVP_U_Matrix *m_world_f_object_out) const;
    void   get_quat_world_f_object_AT(IVP_U_Quat *quat_world_f_object, IVP_U_Point *position) const; // ditto (using quaternions)

    
        /**************************************************************************************
	 *	Name:	    	calc_m_core_f_object 	
	 *	Description:	Calculates a matrix to transform between core and object space.
	 **************************************************************************************/
    void   calc_m_core_f_object(IVP_U_Matrix * m_core_f_object);

    IVP_Core *get_core()	  const { return physical_core;}; // Returns the current core. 
    IVP_Core *get_original_core() const { return original_core; }; // Returns the core that was valid at creation of object.
    
    IVP_Movement_Type get_movement_state() { return (IVP_Movement_Type)flags.object_movement_state; }; // See declaration of IVP_Movement_Type.

    
    //////// Simulation
    IVP_BOOL disable_simulation();          // Freeze object immediately disregarding the speed, return IVP_TRUE when success
    void ensure_in_simulation(); 	// Frozen object is woken up in next PSI (fast).
    void ensure_in_simulation_now(); 	// Wakes up object immediately (slower).


    /////// Collision Detection
    void enable_collision_detection(IVP_BOOL enable = IVP_TRUE);
    int get_collision_check_reference_count(); // see IVP_Universe_Manager for details

    
        /********************************************************************************
	 *	Name:	    	async_add_speed_object_ws and async_add_rot_speed_object_cs	
	 *	Description: 	Easy to use functions used to specify speed and rotation speed
	 *			changes of an object at any time.
	 *	Side Effects:	Object is woken up.
	 *			The speed changes are committed at next PSI.
	 ********************************************************************************/
    void async_push_object_ws( const IVP_U_Point *position_ws, const IVP_U_Float_Point *impulse_ws); // impulse vec's length = impulse strength [kg*m/s]
    void async_add_speed_object_ws( const IVP_U_Float_Point *speed_ws );	   // add a value to the core's speed_change variable
    void async_add_rot_speed_object_cs( const IVP_U_Float_Point *rotation_vec_cs ); // add a value to the core's rot_speed_change variable
    
    IVP_BOOL is_collision_detection_enabled() { // Returns IVP_FALSE if IVP_Environment::enable_collision_detection was not called
	return (IVP_BOOL)flags.collision_detection_enabled;
    };

    /////// Listening
    void add_listener_collision(IVP_Listener_Collision *);	// Adds a private collision callback.
    void remove_listener_collision(IVP_Listener_Collision *);	// Removes a private collision callback.

    void add_listener_object(IVP_Listener_Object *);	        // Adds an private object callback.
    void remove_listener_object(IVP_Listener_Object *);	        // Removes a private object callback.


        /**************************************************************************************
	 *	Name:	    	set_motion_controller 	
	 *	Description:	Specifies an external motion controller.
	 * 			Overrides any existing motion controller for this object.
	 * 			NULL removes controller.
	 *      Version info:   use IVP_Real_Object::create_controller_motion to get same functionality
	 **************************************************************************************/

    void convert_to_phantom(const IVP_Template_Phantom  *tmpl);
    IVP_Controller_Phantom *get_controller_phantom(){ return controller_phantom; };

    
        /**************************************************************************************
	 *	Name:	    	recheck_collision_filter 	
	 *	Description:	to be called when the collisionfilter has a new behaviour
	 **************************************************************************************/
    void recheck_collision_filter();

    /**************************************************************************************
	 *	Name:	    	do_radar_checking 	
	 *	Description:	Checks vicinity for objects
	 * 			that can possibly collide with this object, 
	 *	Version Info:   High optimization of the collision detection engine
	 *                  made this routine finding only very nearby objects
	 **************************************************************************************/
    void do_radar_checking(IVP_Radar *radar);

    
        /**************************************************************************************
	 *	Name:	    	delete_and_check_vicinity 	
	 *	Description:	deletes object and revives surroundings
	 **************************************************************************************/
    void delete_and_check_vicinity();

    
        /**************************************************************************************
	 *	Name:	    	delete_silently 	
	 *	Description:	Silently deletes object, vicinity will stay frozen
	 **************************************************************************************/
    void delete_silently();    

    /**************************************************************************************
     *	   Change on the fly section
     *	   Some physical values can be changed on the fly
     *     Be sure to call recompile_values_changed after changing
     *     Materials can be changed too (directly)
     *     Call recompile_material_changed after material was changed
     **************************************************************************************/
    void change_mass(IVP_FLOAT new_mass); //changes rot_inertia too
    void change_unmovable_flag( IVP_BOOL unmovable_flag );
    void change_fast_piling_allowed(IVP_BOOL flag);
	void set_pinned(IVP_BOOL is_pinned); //@@CB
    
    void change_nocoll_group_ident(const char *new_identify_string ); // note: max length IVP_NO_COLL_GROUP_STRING_LEN
    
    void recompile_values_changed();
    void recompile_material_changed();
};

#endif
