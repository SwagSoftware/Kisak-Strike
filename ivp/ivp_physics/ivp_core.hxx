// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC
#ifndef _IVP_CORE_INCLUDED
#define _IVP_CORE_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

/********************************************************************************
 *	Name:	       	ivp_core.hxx
 *	Description:	the real simulation core of an object
 *	Attention:	many internal variables are defined here, so don't get confused.
 *	Version Info:	this file constantly evolves, handle with care
 ********************************************************************************/

const IVP_DOUBLE MAX_OBJECT_ROT_SPEED = IVP_PI * 0.5f; // max_rotation speed

class IVP_Object;
class IVP_Real_Object;
class IVP_Core;
class IVP_Friction_System;
class IVP_Environment;
class IVP_Friction_Info_For_Core;
class IVP_Core_Merged;
class IVP_Friction_Hash;
class IVP_Simulation_Unit;

#define IVP_OBJECT_MOVING_FAST 1.0f /*[m/sec]*/

// #define DEBUG_FRICTION_CONSISTENCY


/********************************************************************************
 *	Name:	    	IVP_Vec_PCore   	
 *	Description:	Class used as convinient way to calculate the core
 *			coordinates of a direction.
 *	Attention:	Does only work during PSIs
 ********************************************************************************/
class IVP_Vec_PCore : public IVP_U_Float_Point {
public:
    IVP_Vec_PCore(const IVP_Core *pc, const IVP_U_Float_Point *direction_ws_in);
};

class IVP_Old_Sync_Rot_Z {
public:    
    IVP_U_Float_Point	old_sync_rot_speed;
    IVP_U_Quat 	old_sync_q_world_f_core_next_psi;
    IVP_BOOL	was_pushed_during_i_s;      
};

/********************************************************************************
 *	Name:	     	IVP_Core_Friction_Info  	
 *	Description:	internal usage
 ********************************************************************************/
union IVP_Core_Friction_Info { 
    struct {
	IVP_Friction_Hash *l_friction_info_hash;
    } for_unmoveables;
    struct {
	IVP_Friction_Info_For_Core *moveable_core_friction_info;
    } for_moveables;
};

class IVP_Vector_of_Objects:public  IVP_U_Vector<IVP_Real_Object> {
    IVP_Real_Object *elem_buffer[1];
public:
    IVP_Vector_of_Objects(): IVP_U_Vector<IVP_Real_Object>( (void **)&elem_buffer[0],1 ){;};
    void reset(){ elems = (void **)&elem_buffer[0]; memsize = 1; }
};
/********************************************************************************
 *	Name:	       	IVP_Core
 *	Description:	The core of an object is the class handling the physical
 *			simulation part of an object that is independent from
 *                      surface descriptions etc. Through splitting the logical
 *			object from this simulation part, you can use a
 *                      single core for more than one object, thus allowing merged
 *			objects (e.g. to create breakable objects).
 *	Attention:	Use only functions that are declared at the end
 *			of this class declaration.
 ********************************************************************************/

class IVP_Core_Fast_Static {
	friend class IVP_Real_Object;
public:
    /********************************************************************************
     *	type:		private, public READ ONLY
     *	section:	basic, non redundant, long time unchangeable section
     *	Note (*):	Only values marked with (*) may be changed during simulation without problems
     *			(don't forget to call calc_calc() afterwards)
     *  Note:		For memory reasons, the flags will be combined to a bit-area soon.
     *			So use the corresponding functions to read/set them.
     ********************************************************************************/
    IVP_BOOL 		fast_piling_allowed_flag:2;
    IVP_BOOL		physical_unmoveable:2;	// never any movement
    IVP_BOOL            is_in_wakeup_vec:2;
    IVP_BOOL		rot_inertias_are_equal:2;
	IVP_BOOL		pinned:2;
    
    IVP_FLOAT		upper_limit_radius;	// the radius of the object around the mass center
    IVP_FLOAT		max_surface_deviation;	// for all P of all Surfaces S:		maximum P * S

    IVP_Environment 	*environment; 	// Backlink
    
    class IVP_Constraint_Car_Object *car_wheel;  // bad bad hack for optimized wheel behaviour;
    
protected:
    IVP_U_Float_Hesse	rot_inertia;            // needed for energy calculation, mass is w part
public:
    
    IVP_U_Float_Point	rot_speed_damp_factor;	// (*) dampening of speed, one value per axis: new speed *= 1/e**(rot_speed_damp_factor * delta_time)

    /********************************************************************************
     *	type:		private, public READ ONLY
     *	section:	basic, redundant, long time unchangeable section (updated by calc_calc)
     ********************************************************************************/
private:
    IVP_U_Float_Hesse	inv_rot_inertia;  // 1.0f/rot_inertia, 4th value is inverse_mass
public:

    IVP_FLOAT		speed_damp_factor;	// (*) same for speed
    IVP_FLOAT		inv_object_diameter; 		// 2.0f/upper_limit_radius  
	IVP_U_Float_Point *spin_clipping;   // if set, than angular velocities are individually clipped against this value

    /********************************************************************************
     *	section:	internal logical section
     ********************************************************************************/
    IVP_Vector_of_Objects objects; 	// All objects for this core.

    /********************************************************************************
     *	type:	  private, internal
     *	section:  friction, systems
     ********************************************************************************/
    IVP_Core_Friction_Info core_friction_info;


    const class IVP_U_Point_4 *get_inv_masses(){ return (class IVP_U_Point_4*)& inv_rot_inertia;} // combined solution for inertias
    IVP_FLOAT get_mass() const{ return rot_inertia.hesse_val; };

    const IVP_U_Float_Point *get_rot_inertia()const{ return &rot_inertia;};
    const IVP_U_Float_Point *get_inv_rot_inertia() const{ return &inv_rot_inertia;};
    IVP_FLOAT get_inv_mass() const{ return inv_rot_inertia.hesse_val;};

};

class IVP_Core_Fast_PSI:  public IVP_Core_Fast_Static {
public:
    /********************************************************************************
     *	type		private, public READ ONLY
     *	section:	basic simulation part, non redundant part
     ********************************************************************************/
    IVP_Movement_Type	movement_state:8;		// Movement state
    IVP_BOOL            temporarily_unmovable:8;

    short               impacts_since_last_PSI;
    IVP_Time     	time_of_last_psi;		// Indicates point in simulation time the last matrizes are valid at.    

    /********************************************************************************
     *	type:		private, public READ ONLY
     *	section:	basic simulation part, redundant part, calculated by calc_next_PSI_matrix
     ********************************************************************************/
    IVP_FLOAT	i_delta_time;				// 1.0f / (delta_time)

    // #+# check for empty space (due to align(16))
    IVP_U_Float_Point	rot_speed_change;		// Accumulates async_rot_pushes, units like this->rot_speed.
    IVP_U_Float_Point	speed_change;			// Accumulates async_pushes, units like this->speed.

    IVP_U_Float_Point	rot_speed;			// Rotation speed around core axles [rad/s]
    IVP_U_Float_Point	speed;				// Translation speed in world system [m/s]

    IVP_U_Point pos_world_f_core_last_psi;
    IVP_U_Float_Point delta_world_f_core_psis;

    IVP_U_Quat	q_world_f_core_last_psi;	// Rotation of core using quaternions (see m_world_f_core_x_PSI)
    IVP_U_Quat 	q_world_f_core_next_psi;	// is is the quat to use at a psi

    IVP_U_Matrix	m_world_f_core_last_psi;	/* The core matrix valid at PSI (rotation part is redundant, see q_world_f_core_last_psi),							 * use get_m_world_f_core_PSI() to get a valid pointer to this matrix */
};

/********************************************************************************
 *	type:		IVP_Core_Fast
 *	description:	data neded between two psis
 ********************************************************************************/
class IVP_Core_Fast : public IVP_Core_Fast_PSI {
public:
    // needed for collision detection
    IVP_U_Float_Point rotation_axis_world_space;        // normized q_ncore_f_core
    IVP_FLOAT current_speed;		// speed.real_length()
    IVP_FLOAT abs_omega;		// absolute angular velocity interpolated movement
    IVP_FLOAT max_surface_rot_speed;	// abs_omega * max_surface_deviation
};


class IVP_Core: public IVP_Core_Fast {  // 
    friend class IVP_Environment;
    
protected:
    void init(IVP_Real_Object *io);
    IVP_Core(IVP_Real_Object *io);
public:
        
    IVP_U_Vector<class IVP_Controller>  controllers_of_core; //all controllers our core is controlled of (friction systems, constraints, force fields ...)
    IVP_Core_Merged	*merged_core_which_replace_this_core;	// Or NULL when core is terminal.
    IVP_Simulation_Unit *sim_unit_of_core;  
     
    /********************************************************************************
     *	type:	  private: internal
     *	section:  reference matrizes used for movement checking, #+# maybe only for moving objects
     ********************************************************************************/
    IVP_Time			time_of_calm_reference[2];
    IVP_U_Float_Quat		q_world_f_core_calm_reference[2];	// used to check the movement of an objec
    IVP_U_Float_Point		position_world_f_core_calm_reference[2];    

#ifdef DEBUG_FRICTION_CONSISTENCY
    IVP_U_Vector<IVP_Friction_Info_For_Core> list_debug_hash;
#endif
    
    IVP_Core *union_find_get_father();      
    union { // #+# merge with tmp_null
	IVP_Core *union_find_father; //  for friction system
    }tmp; //any values is allowed when not used

    union {
	IVP_Old_Sync_Rot_Z *old_sync_info; //is also used as flag ("is core known to impact system")
    } tmp_null; //warning: tmp_null has to be 0 when not used (some values are flags)

    
    IVP_Time_CODE mindist_event_already_done;  // used at the end of a impact_system cmp to environment->mindist_event_timestamp_reference

    /********************************************************************************
     *	type:		private, internal
     *	section:	constructor
     ********************************************************************************/
    IVP_Core(IVP_Real_Object *object, const IVP_U_Quat *q_world_f_object_init, const IVP_U_Point *position, IVP_BOOL physical_unmoveable_, IVP_BOOL enable_piling_optimization);
    ~IVP_Core();    
    
    void add_core_controller(IVP_Controller *cntrl);
    void rem_core_controller(IVP_Controller *cntrl);
    
    void add_friction_info(IVP_Friction_Info_For_Core *my_fr_system);
    void delete_friction_info(IVP_Friction_Info_For_Core *my_fr_system);    
    void unlink_friction_info(IVP_Friction_Info_For_Core *my_fr_system);
    IVP_BOOL grow_friction_system(); //returns true when system has been grown    
    IVP_Friction_Info_For_Core *get_friction_info(IVP_Friction_System *my_fr_system);
    IVP_Friction_Info_For_Core *moveable_core_has_friction_info();
    void unmovable_core_debug_friction_hash();

    /********************************************************************************
     *	type:		private, internal
     *	section:	friction debug
     ********************************************************************************/
    void core_plausible_check();
    void rot_speed_plausible_check(const IVP_U_Float_Point *rot_speed);
    void debug_vec_movement_state();
    void debug_out_movement_vars();
    /********************************************************************************
     *	type:		private, internal
     *	section:	FUNCTIONS friction
     ********************************************************************************/
    
    void update_exact_mindist_events_of_core();// sets mindist_event_already_done and recalculates exact mindist events to all objects with flag not set
    
     /********************************************************************************
     *	type:		private, internal
     *	section:	FUNCTIONS logical
     ********************************************************************************/
    void unlink_obj_from_core_and_maybe_destroy(IVP_Real_Object *remove_obj);
    void core_add_link_to_obj(IVP_Real_Object *add_obj);

    void set_fast_piling_allowed(IVP_BOOL bool_flag) { fast_piling_allowed_flag=bool_flag; };
    IVP_BOOL fast_piling_allowed() { return (IVP_BOOL)fast_piling_allowed_flag; };


    /********************************************************************************
     *	type:		private, public handle with care
     *	section:	initializing basic, non redundant, long time unchangeable section
     ********************************************************************************/    
    void set_radius(IVP_FLOAT upper_limit, IVP_FLOAT max_dev);
    void transform_PSI_matrizes_core(const IVP_U_Matrix *m_core_f_core);	// Multiply matrizes with m_core_f_core
    
    /********************************************************************************
    *	Name:	      	set_mass 	
    *	Description:	sets the mass and multiplies all rot_inertias with
    *                   new_mass/old_mass.
    *   Attention:      Does not update spring constants that are declared
    *   	        relative.
    *                   set_mass is not forwarded to original cores, thus
    *                   set_mass may only temporarly change the mass of the
    *                   object (till next PSI, object breaks .)
    ********************************************************************************/
    void set_mass(IVP_FLOAT mass);
    void set_rotation_inertia( const IVP_U_Float_Point *);

    inline    void clip_velocity(IVP_U_Float_Point *velocity, IVP_U_Float_Point *angular_velocity); // parameters are input and output !!!

    inline void calc_next_PSI_matrix(IVP_U_Vector<IVP_Core> *cores_which_needs_calc_next_psi, class IVP_Event_Sim *){ // call commit_all_calc_next_PSI_matrix afterwards
	cores_which_needs_calc_next_psi->add(this);
    }
    
    void calc_next_PSI_matrix_zero_speed(class IVP_Event_Sim *);
    
    
    /********************************************************************************
     *	type:		public
     *	section:	updates the redundant, long time unchangeable section
     ********************************************************************************/
    void calc_calc();					// calculate redundant part

    void synchronize_with_rot_z();    // synchronize m_world_f_core_last_psi and time_of_last_psi with interpolation
    void undo_synchronize_rot_z();
    
    
    /********************************************************************************
     *	type:		private, internal
     *	section:	core state change handling
     ********************************************************************************/
    void init_core_for_simulation();
    void stop_physical_movement(); // to avoid unwanted inter/extrapolations on calm objects
    void fire_event_object_frozen(); // just send events
    void freeze_simulation_core();
    void stop_movement_without_collision_recheck();
    IVP_BOOL revive_simulation_core(); // returns true when datastructures are changed
    void ensure_core_in_simulation_delayed() { environment->add_revive_core(this); }
    void revive_adjacent_to_unmoveable();
    void ensure_core_to_be_in_simulation();
    void ensure_all_core_objs_in_simulation(); // for merged cores!
    void ensure_all_core_objs_in_simulation_now(); // for merged cores!
    void reset_freeze_check_values();
    
    /********************************************************************************
     *	type:		private, internal
     *	section:	different simulation functions
     ********************************************************************************/
    IVP_Movement_Type calc_movement_state(IVP_Time psi_time);

    void create_collision_merged_core_with(IVP_Core *other_core);		// create a merged core
    void set_matrizes_and_speed(IVP_Core_Merged *template_core, IVP_U_Matrix *m_CORE_f_core_out); // for splitting

    void reset_time( IVP_Time offset);
    void damp_object(IVP_DOUBLE virt_delta_time,const IVP_U_Float_Point *rotation_factor, IVP_DOUBLE speed_factor);
    void global_damp_core(IVP_DOUBLE delta_time);
    
    /********************************************************************************
     *	type:		PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC
     *	type:		PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC
     *	type:		PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC
     *	type:		PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC
     *	type:		PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC PUBLIC
     *	Description:	All functions below this line are meant real public.
     *
     *	Notes:		Use these methods ONLY at PSI!!
     *			floats are used for os and cs and directions
     *			duoble is used for ws
     *
     *	 async_		The core has two sets of speed variables:
     *			speed (rot_speed) and speed_change (rot_speed_change)
     *			the xxx_change variables are used to collect changes to
     *	 		be added to the real values later.
     *			ALL FUNCTIONS, WHICH MODIFY THE xxx_change VARIABLES
     *			HAVE THE async PREFIX
     *	
     *	 test_		In some cases the user does not want to modify the core but
     *			wants to get the result in his own speed and rot_speed variables
     *			THIS FUNCTIONS HAVE THE test_ PREFIX
     *
     *	 _ws		xxx_ws means function/variable works in world space
     *	 _cs		xxx_cs means function/variable works in core space
     *			No such suffix means functions needs parameters defined in different spaces,
     *			check the variable names for details.
     *
     *	 inline_	All inline functions have got the 'inline_' prefix, include ivp_core_macros.hxx
     *			to use them.
     *
     *	 _AT		Means: Any time, not restricted to PSI states
     *			(If not otherwise mentioned, functions work correctly at PSI time only)
     *
     *	 rot_push_  	The 'rot_' prefix means that the function exerts a rotation
     *			impulse only.
     *
     *	 center_push_	the 'center_' prefix means, core is pushed in mass center,
     *			rot_speed is not changed.
     *
     *	_on_test	In all standard situations, the core's speed and rot_speed
     *			variables are taken as source. If these values are parameters
     *			to the function, the _on_test suffix is appended to the
     *			function name
     ********************************************************************************/


    
    /********************************************************************************
     *	section:	get the ws speed of any point fixed in core space
     ********************************************************************************/
    void   get_surface_speed( const IVP_U_Float_Point *position_cs_in, IVP_U_Float_Point *speed_ws_out) const;
    void   get_surface_speed_ws( const IVP_U_Point *position_ws_in, IVP_U_Float_Point *speed_ws_out);
    void   get_surface_speed_on_test(const IVP_U_Float_Point *position_cs_in, const IVP_U_Float_Point *center_speed_in, const IVP_U_Float_Point *rot_speed_in,
				     IVP_U_Float_Point *speed_ws_out) const;
        
   /********************************************************************************
    *	Name:		get_diff_surface_speed_of_two_cores
    *	Description:	core_A->get_surface_speed() - core_B->get_surface_speed()
    ********************************************************************************/
    static void	get_diff_surface_speed_of_two_cores(const IVP_Core *core_A, const IVP_Core *core_B,
						    const IVP_U_Float_Point *position_A_cs, const IVP_U_Float_Point *position_B_cs,
						    IVP_U_Float_Point *speed_ws_A_minus_B_out);
    

    // #+# remove, use inline_get_surface_speed_parallel_to_vector_on_test instead
    static void get_diff_surface_speed_of_two_cores_on_test(const IVP_Core *core_A, const  IVP_Core *core_B,
							    const IVP_U_Float_Point *position_A_cs, const IVP_U_Float_Point *position_B_cs,
							    const IVP_U_Float_Point *speed_A, const IVP_U_Float_Point *rot_speed_A,
							    const IVP_U_Float_Point *speed_B, const IVP_U_Float_Point *rot_speed_B,
							    IVP_U_Float_Point *speed_ws_A_minus_B_out);

    
        
    /********************************************************************************
     *	section:	push the core at a given point and given impulse (#+# will be removed soon)
     ********************************************************************************/
    void       push_core (const IVP_U_Float_Point *position_cs_in, const IVP_U_Float_Point *impulse_cs_in, const IVP_U_Float_Point *impulse_ws_in);
               // very fast, but needs redundant input values
    void async_push_core (const IVP_U_Float_Point *position_cs_in, const IVP_U_Float_Point *impulse_cs_in, const IVP_U_Float_Point *impulse_ws_in);
    
    void  test_push_core (const IVP_U_Float_Point *position_cs_in, const IVP_U_Float_Point *impulse_cs_in, const IVP_U_Float_Point *impulse_ws_in,
			 IVP_U_Float_Point *speed_change_out, IVP_U_Float_Point *rot_change_out) const;
               // very fast, but needs redundant input values
    
   /********************************************************************************
    *	Description:	convinient routines of the above functions
    ********************************************************************************/
    void        push_core_ws( const IVP_U_Point *position_ws, const IVP_U_Float_Point	*impulse_ws);
    void  async_push_core_ws( const IVP_U_Point	*position_ws, const IVP_U_Float_Point	*impulse_ws);

   /********************************************************************************
    *	Description:	apply an impulse at the center of mass
    ********************************************************************************/
    void       center_push_core_multiple_ws(const IVP_U_Float_Point *impulse_ws, IVP_DOUBLE factor = 1.0f);
    void async_center_push_core_multiple_ws(const IVP_U_Float_Point *impulse_ws, IVP_DOUBLE factor = 1.0f);


   /********************************************************************************
    *	Description:	apply an angular impulse at the center of mass
    ********************************************************************************/
    void       rot_push_core_multiple_ws(const IVP_U_Float_Point *angular_impulse_ws, IVP_DOUBLE factor = 1.0f);
    void async_rot_push_core_multiple_ws(const IVP_U_Float_Point *angular_impulse_ws, IVP_DOUBLE factor = 1.0f);

   /*******************************************************************************
    *	Name:	Rot Pushes
    *	Description: Exerts a rotation impulse
    *	Note:	The length of the axis * [factor] is the power of rotation impulse
    ********************************************************************************/
    void       rot_push_core_cs          ( const IVP_U_Float_Point *angular_impulse_cs);
    void       rot_push_core_multiple_cs ( const IVP_U_Float_Point *angular_impulse_cs, IVP_DOUBLE factor = 1.0f);
    void async_rot_push_core_multiple_cs ( const IVP_U_Float_Point *angular_impulse_cs, IVP_DOUBLE factor = 1.0f);
    void  test_rot_push_core_multiple_cs ( const IVP_U_Float_Point *angular_impulse_cs, IVP_DOUBLE factor, IVP_U_Float_Point *delta_rot_speed_out);
    
   /********************************************************************************
    *	Name:	      	get_energy_on_test 	
    *	Description:    Returns the kinetic energy of the object
    ********************************************************************************/
    IVP_DOUBLE get_energy_on_test(const IVP_U_Float_Point *speed_vec, const IVP_U_Float_Point *rot_speed_vec);

    
   /********************************************************************************
    *	Name:	      	get_rot_inertia_cs 	
    *	Description:	Get the virtual projected rotation inertia for one cs axis
    ********************************************************************************/
    IVP_DOUBLE get_rot_inertia_cs(const IVP_U_Float_Point *normized_axis_cs);

   /********************************************************************************
    *	Name:	      	get_rot_speed_cs 	
    *	Description:	Get projected rotation speed for one given axis
    ********************************************************************************/
    IVP_DOUBLE get_rot_speed_cs(  const IVP_U_Float_Point *normized_axis_cs);


    /********************************************************************************
    *	Name:	      	xxx_all_async_pushes	
    *	Description:	handle xxx_change variables of core
    *	Note:		not really for public use, handle with care
    ********************************************************************************/
    void abort_all_async_pushes();			// clear all xxx_change variables
    void commit_all_async_pushes();			// make all asynchronous pushes permanent
    

    IVP_Environment *get_environment() { return this->environment; };
    
   /********************************************************************************
    *	Name:	     	calc_correct_virt_mass  	
    *	Description:	Get the virtual mass of a core space point,
    *			given a core direction and a world direction (e.g. of an assumed force vector).
    *                   Works like calc_virt_mass but slower and more accurately.
    ********************************************************************************/
    IVP_DOUBLE calc_correct_virt_mass(const IVP_U_Float_Point *position_cs,const IVP_U_Float_Point *direction_core,const IVP_U_Float_Point *direction_world) const;

    
   /********************************************************************************
    *	Name:	     	calc_virt_mass  	
    *	Description:	Estimates the virtual mass of a core space point,
    *			given a core direction (e.g. of an assumed force vector).
    ********************************************************************************/
    IVP_DOUBLE calc_virt_mass(const IVP_U_Float_Point *core_point, const  IVP_U_Float_Point *direction)const;	// calc the virtual mass of a point 

    
   /********************************************************************************
    *	Name:	      	calc_virt_mass_worst_case 	
    *	Description:	Same as above, coarse calculation of minimal virt. mass,
    *                   regarding all directions.
    ********************************************************************************/
    IVP_DOUBLE calc_virt_mass_worst_case(const IVP_U_Float_Point *core_point)const;
    

    /*******************************************************************************************
     *	section:	Calculate core matrix/quaternion/position at a user defined time between
     *			last and next PSI
     *******************************************************************************************/
           void calc_at_matrix(IVP_Time current_time, IVP_U_Matrix *m_world_f_core_out) const;  		// calc a matrix at any time
    inline void inline_calc_at_quaternion(IVP_Time time, IVP_U_Quat *quat_out) const; 	
    inline void inline_calc_at_position(IVP_Time time, IVP_U_Point *position_out) const; 	

    
   /********************************************************************************
    *	Name:	       	get_m_world_f_core_PSI
    *	Description:	Get the core matrix at PSI time
    ********************************************************************************/
    const IVP_U_Matrix *get_m_world_f_core_PSI()  { return &m_world_f_core_last_psi; };
    const IVP_U_Point *get_position_PSI() { return m_world_f_core_last_psi.get_position(); };

   /********************************************************************************
    *	Name:	       	values_changed_recalc_redundants
    *	Description:	The mass, rotation inertia, or material of associated object
    *                   changed. Recalculate some internal variables. Pretty fast.
    *                   Call at any time.
    ********************************************************************************/
    void values_changed_recalc_redundants();

    //lwss added
    void apply_velocity_limit();
    //lwss end
};


#endif /* _IVP_CORE_INCLUDED */


