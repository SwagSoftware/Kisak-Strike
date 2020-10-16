// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

#if !defined(IVP_ACTUATOR_INCLUDED)
#	define IVP_ACTUATOR_INCLUDED

class IVP_Template_Check_Dist;
class IVP_Template_Force;
class IVP_Template_Extra;
class IVP_Template_Rot_Mot;
class IVP_Template_Spring;
class IVP_Template_Stabilizer;
class IVP_Template_Two_Point;
class IVP_Template_Four_Point;
class IVP_Template_Anchor;

#ifndef _IVP_ACTUATOR_INFO_INCLUDED
#	include "ivp_actuator_info.hxx"
#endif

#ifndef _IVP_CONTROLLER_INCLUDED
#	include "ivp_controller.hxx"
#endif

#ifndef _IVP_ACTIVE_VALUES_INCLUDED
#	include "ivu_active_value.hxx"
#endif

#ifndef _IVP_LISTENER_HULL_INCLUDED
#	include <ivp_listener_hull.hxx>
#endif



/********************************************************************************
 *	Name:	       	IVP_Template_Anchor
 *	Description:	A fixed point in an objects coordinate system.
 *			Normally used by actuators.
 *	Note:		Anchors may be used by more than one actuator.
 *			At the beginning of every PSI, the world coordinates of
 *			all anchors of the objects are calculated.
 *			To create an anchor, call IVP_Real_Object::create_anchor()
 ********************************************************************************/
class IVP_Template_Anchor {
	friend class IVP_Real_Object;
    friend class IVP_Anchor;
private:
    IVP_Real_Object *object;
    IVP_U_Point coords_world;	// coordinates in world space
public:
    IVP_Real_Object *get_object(){ return object; };
    IVP_U_Point *get_anchor_point_ws(){ return &coords_world; };
    
    // unit for all conversion methods: meters
	void set_anchor_position_ws(IVP_Real_Object *obj, const IVP_U_Point *coords_ws);
	void set_anchor_position_ws(IVP_Real_Object *obj, const IVP_DOUBLE x, const IVP_DOUBLE y, const IVP_DOUBLE z);
	void set_anchor_position_os(IVP_Real_Object *obj, const IVP_U_Float_Point *coords_os);
	void set_anchor_position_os(IVP_Real_Object *obj, const IVP_DOUBLE x, const IVP_DOUBLE y, const IVP_DOUBLE z);
#if 1
	void set_anchor_position_cs(IVP_Real_Object *obj, const IVP_U_Float_Point *coords_cs);
	void set_anchor_position_cs(IVP_Real_Object *obj, const IVP_DOUBLE x, const IVP_DOUBLE y, const IVP_DOUBLE z);
#endif
    const IVP_U_Point *get_anchor_position_ws() const { return & coords_world; };
};


/********************************************************************************
 *	Name:	       	IVP_Template_Two_Point
 *	Description:	Two anchors
 ********************************************************************************/
class IVP_Template_Two_Point {
public:
    void *client_data;		// user definable data
    IVP_Template_Anchor *anchors[2];
    IVP_Template_Two_Point();
};

/********************************************************************************
 *	Name:	       	IVP_Template_Four_Point
 *	Description:	2 pair of anchors
 ********************************************************************************/
class IVP_Template_Four_Point {
public:
    void *client_data;		// user definable data
    IVP_Template_Anchor *anchors[4];
    IVP_Template_Four_Point();
};


/********************************************************************************
 *	Name:	       	IVP_Check_Dist
 *	Description:	A actuator used for comparing  the distance between two
 *			anchors against a fixed range
 *	Input:		range:	the distance to check
 *	Output:		result of the check,
 *			the result is stored either in the info block of the Actuator
 *			and optionally in the mod_len and mod_is_outside active_floats
 *			
 *	Version Info:	its implementation is very, very fast, so several
 *			ten thousands IVP_Actuator_Check_Dist can be used at the same time.
 *			
 ********************************************************************************/

class IVP_Template_Check_Dist {
public:
    void *client_data;
    IVP_Real_Object *objects[2];	// objects the end points of the check distance is connected
    IVP_U_Point position_world_space[2];	// world position of the end points (unit: meters)
    
    IVP_FLOAT range; // distance to check (unit: meters)

    // output section
    IVP_U_Active_Terminal_Int *mod_is_outside; // optional active_int to write the result to

    IVP_Template_Check_Dist();
};


/********************************************************************************
 *	Name:	      	IVP_Template_Force
 *	Description:	Simple force, either static or dynamic (using active_floats)
 ********************************************************************************/

class IVP_Template_Force: public IVP_Template_Two_Point {
public:
    IVP_FLOAT force;                        // the force (unit: Newton)
    IVP_U_Active_Float *active_float_force;	// optional override of force

    IVP_BOOL push_first_object;         // default IVP_TRUE, apply force on first object ?
    IVP_BOOL push_second_object;	// default IVP_FALSE, apply neg. force on second object
    IVP_Template_Force();
};


/********************************************************************************
 *	Name:	      	IVP_Template_Torque
 *	Description:	Simple torque, either static or dynamic (using active_floats)
 ********************************************************************************/
class IVP_Template_Torque: public IVP_Template_Two_Point {
public:
    IVP_FLOAT torque;                        // torque force
    IVP_U_Active_Float *active_float_torque;	// optional override of torque

	IVP_FLOAT max_rotation_speed;			// max fabs(rotation speed) the torque will be applied
    IVP_U_Active_Float *active_float_max_rotation_speed;	// optional override of max_rot_speed

    IVP_U_Active_Terminal_Double *active_float_rotation_speed_out; // optional
    IVP_Template_Torque();
};


/********************************************************************************
 *	Name:	       	IVP_Template_Rot_Mot
 *	Description:	Rotation Motor
 *                      This simulates a motor (with power and a gear)
 ********************************************************************************/
class IVP_Template_Rot_Mot: public IVP_Template_Two_Point {
public:
    IVP_FLOAT max_rotation_speed; // max absolute rotation speed [ angle/sec ]
    IVP_FLOAT power;              // the power which is applied to the object [J] = [N * m/sec]
                              // neg power means going backwards
    IVP_FLOAT max_torque;         // max absolute torque [n/(m*m)]
    
    IVP_U_Active_Float *active_float_max_rotation_speed;  // optional override of static value
    IVP_U_Active_Float *active_float_power;               // optional override of static value
    IVP_U_Active_Float *active_float_max_torque;          // optional override of static value

    IVP_U_Active_Terminal_Double *active_float_rotation_speed_out;
    IVP_Template_Rot_Mot();
};


/********************************************************************************
 *	Name:	      IVP_Template_Extra 	
 *	Description:  for ipion demos only (e.g. IVP_FLOAT cam)
 *	Attention:    forget this
 *	Version Info: will be deleted
 ********************************************************************************/
class IVP_Template_Extra: public IVP_Template_Two_Point {
public:
    IVP_Extra_Info info;
};


/********************************************************************************
 *	Name:	       	IVP_Template_Stabilizer
 *	Description:	Spring which calculates the differences between two
 *                      pairs of anchors and tries to keep the distances
 *                      of the points of each pair equal
 ********************************************************************************/
class IVP_Template_Stabilizer: public IVP_Template_Four_Point {
public:
    IVP_FLOAT stabi_constant;	/* in Newton/meter or (if stabi_values_are_relative) Newton/(meter*virtual_mass)
				 * (used to create mass independent systems) */
    IVP_U_Active_Float *active_float_stabi_constant;
    IVP_Template_Stabilizer();
};




/********************************************************************************
 *	Name:	      	IVP_Anchor 	
 *	Description:	An anchor is a position in an objects space
 *			It's used by actuators, which can share one anchor.
 *	Attention:	
 *	Version Info:
 ********************************************************************************/
class IVP_Anchor {
    friend class IVP_Real_Object;
    friend class IVP_Actuator_Two_Point;
    friend class IVP_Actuator_Four_Point;
    ~IVP_Anchor();
protected:
    IVP_Anchor *anchor_next_in_object;		// linked list of anchors per object
    IVP_Anchor *anchor_prev_in_object;
public:
    IVP_Real_Object *l_anchor_object;  // might be zero
    IVP_U_Float_Point object_pos;	// original position of object space
    IVP_U_Float_Point core_pos;		// core position of anchor
    class IVP_Actuator *l_actuator;
public:
    IVP_Anchor(){;};
    void object_is_going_to_be_deleted_event(IVP_Real_Object *obj);
    
    void init_anchor(IVP_Actuator *, IVP_Template_Anchor *);	// constructor, called by IVP_Real_Object::create_anchor
    IVP_Anchor *move_anchor(IVP_U_Point *coords_ws); 	// move anchor

    IVP_Anchor *get_next_anchor(){ return anchor_next_in_object; };
    IVP_Anchor *get_prev_anchor(){ return anchor_prev_in_object; };
    
    IVP_Real_Object *anchor_get_real_object() { return l_anchor_object; };
};



/********************************************************************************
 *	Name:	      	IVP_ACTUATOR_TYPE 	
 *	Description:	Needed for debugging	
 ********************************************************************************/
enum IVP_ACTUATOR_TYPE {
    IVP_ACTUATOR_TYPE_NONE,
    IVP_ACTUATOR_TYPE_SPRING,
    IVP_ACTUATOR_TYPE_STIFF_SPRING,
    IVP_ACTUATOR_TYPE_SUSPENSION,
    IVP_ACTUATOR_TYPE_FORCE,
    IVP_ACTUATOR_TYPE_ROT_MOT,
    IVP_ACTUATOR_TYPE_STABILIZER,
    IVP_ACTUATOR_TYPE_TORQUE,
    IVP_ACTUATOR_TYPE_ETC
};

/********************************************************************************
 *	Name:	      	IVP_Actuator 	
 *	Description:	All actuators are used to control forces on objects,
 *			Normally they are attached to several (mostly two) objects
 *			using anchors.
 *			All actuators of one type should normally by grouped
 *			by an actuator_manager. At every PSI all actuator_managers
 *		        execute their actuators code
 ********************************************************************************/
class IVP_Actuator : public IVP_Controller_Dependent
{
    friend class IVP_Actuator_Manager;
protected:
public:
    //Controller Section
    IVP_U_Vector<IVP_Core> actuator_controlled_cores;
    IVP_U_Vector<IVP_Core> *get_associated_controlled_cores() { return &actuator_controlled_cores; };
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_ACTUATOR; };

    IVP_Actuator(IVP_Environment *env);
    virtual void anchor_will_be_deleted_event(IVP_Anchor *del_anchor); // when an object is deleted it sends events to its connected actuators
    virtual void core_is_going_to_be_deleted_event(IVP_Core *my_core);
    virtual ~IVP_Actuator();
};


/********************************************************************************
 *	Name:	       	IVP_Actuator_Two_Point	
 *	Description:	For all standard cases, two anchors per actuator are
 *			sufficient. This is the base class for all such actuators
 *	Attention:	
 *	Version Info:
 ********************************************************************************/
class IVP_Actuator_Two_Point : public IVP_Actuator
{
    friend class IVP_Actuator_Four_Point;
protected:
    IVP_Anchor anchors[2];	// the two anchors
    void unlink_from_objects();
public:    
    IVP_Actuator_Two_Point( IVP_Environment *, IVP_Template_Two_Point *two_point_templ,IVP_ACTUATOR_TYPE ac_type);
    virtual ~IVP_Actuator_Two_Point();

    IVP_Anchor *get_actuator_anchor(int i) { return &anchors[i]; }

    void *client_data;
    void ensure_actuator_in_simulation();
    IVP_DOUBLE calc_len(); 				// calc the distance between both anchors, it's only valid during the PSI  
};


/********************************************************************************
 *	Name:	       	IVP_Actuator_Four_Point	
 *	Description:	For stabilizer e.g.
 ********************************************************************************/
class IVP_Actuator_Four_Point : public IVP_Actuator
{
protected:
    IVP_Anchor anchors[4];	// the four anchors
public:
    void *client_data;
    

    IVP_Actuator_Four_Point( IVP_Environment *, IVP_Template_Four_Point *four_point_templ,
			     IVP_ACTUATOR_TYPE ac_type);
    virtual ~IVP_Actuator_Four_Point();
    
    IVP_Anchor *get_actuator_anchor(int i) { return &anchors[i]; }
};




/********************************************************************************
 *	Name:	     	IVP_Actuator_Force
 *	Description:	Forces push one or two objects at the anchor position.
 *			The force value can be controlled by an IVP_Active_Float 
 *	Attention:	To create a force use the IVP_Environment->create_force function
 *                      Constantly changing the force value will cause that
 *			the involved objects are never taken out of simulation.
 *			(What means unnessary CPU consumption when e.g.
 *                      invisible trees are rattled unnecessesarily.)
 ********************************************************************************/

class IVP_Actuator_Force: public IVP_Actuator_Two_Point
{
    friend class IVP_Force_Manager;

    IVP_FLOAT force;                        // the force 
    IVP_BOOL push_first_object:1;
    IVP_BOOL push_second_object:1;
protected:
    friend class IVP_Environment;

    // To create a new force actuator, use the method IVP_Environment->create_force
    IVP_Actuator_Force(IVP_Environment *env, IVP_Template_Force *templ);
public:
    void set_force(IVP_DOUBLE new_force);
    virtual ~IVP_Actuator_Force();

    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
};


class IVP_Actuator_Force_Active: public IVP_Actuator_Force, IVP_U_Active_Float_Listener
{
    IVP_U_Active_Float *active_float_force;	// optional override of force
protected:
    void active_float_changed(IVP_U_Active_Float *calling_mod); // for wakup
    friend class IVP_Environment;
    IVP_Actuator_Force_Active(IVP_Environment *env, IVP_Template_Force *templ);
public:
    // to create a new use the IVP_Environment->create_force function
    virtual ~IVP_Actuator_Force_Active();
};


/********************************************************************************
 *	Name:	      	IVP_Actuator_Rot_Mot	
 *	Description:	rotation motors allows to rotate objects around a given
 *			axis. (E.g. car wheels, wind mills)
 *	Note:		Values can be changed on the fly using IVP_U_Active_Floats
 ********************************************************************************/
class IVP_Actuator_Rot_Mot: public IVP_Actuator_Two_Point
{
    IVP_FLOAT max_rotation_speed; // max absolute rotation speed [ angle/sec ]
    IVP_FLOAT power;              // the power which is applied to the object [J] = [N * m/sec]
                              // neg power means going backwards
    IVP_FLOAT max_torque;         // max absolute torque [n/(m*m)]
    
    
    friend class IVP_Rot_Mot_Manager;

    IVP_U_Float_Point axis_in_core_coord_system;
    IVP_DOUBLE rot_inertia;
    
    //void do_rot_mot(IVP_DOUBLE d_time);
    IVP_U_Active_Terminal_Double *active_float_rotation_speed_out;
protected:
    friend class IVP_Environment;
    IVP_Actuator_Rot_Mot(IVP_Environment *env, IVP_Template_Rot_Mot *templ);
public:
    IVP_FLOAT rot_speed_out;		// used to export the last rotation speed
    void set_max_rotation_speed( IVP_DOUBLE );
    void set_power(IVP_DOUBLE);
    IVP_FLOAT get_power() { return power; };
    void set_max_torque(IVP_DOUBLE);

    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
  
    virtual ~IVP_Actuator_Rot_Mot();
};


class IVP_Actuator_Rot_Mot_Active: public IVP_Actuator_Rot_Mot, public IVP_U_Active_Float_Listener {
    IVP_U_Active_Float *active_float_max_rotation_speed;
    IVP_U_Active_Float *active_float_power;
    IVP_U_Active_Float *active_float_max_torque;
    void active_float_changed(IVP_U_Active_Float *calling_mod); // for wakup
public:
    IVP_Actuator_Rot_Mot_Active(IVP_Environment *env, IVP_Template_Rot_Mot *templ);
    ~IVP_Actuator_Rot_Mot_Active();
};


/********************************************************************************
 *	Name:	      	IVP_Actuator_Torque	
 *	Description:	torques allows to rotate objects around a given
 *			axis. (E.g. car wheels, wind mills)
 *	Note:		Values can be changed on the fly [e.g. by using IVP_U_Active_Floats]
 ********************************************************************************/
class IVP_Actuator_Torque: public IVP_Actuator_Two_Point
{
    IVP_FLOAT max_rotation_speed; // max absolute rotation speed [ angle/sec ]
    IVP_FLOAT torque;              // the torque which is applied to the object = [N*m]
                              // neg torque means going backwards
    IVP_U_Float_Point axis_in_core_coord_system;
    IVP_DOUBLE rot_inertia;
    
    IVP_U_Active_Terminal_Double *active_float_rotation_speed_out;
protected:
    friend class IVP_Environment;

    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);

    IVP_Actuator_Torque(IVP_Environment *env, IVP_Template_Torque *templ);
public:
    IVP_FLOAT rot_speed_out;		// used to export the last rotation speed
    void set_max_rotation_speed( IVP_DOUBLE );
    void set_torque(IVP_DOUBLE);
    IVP_FLOAT get_torque() { return torque; };
    
    virtual ~IVP_Actuator_Torque();
};


class IVP_Actuator_Torque_Active: public IVP_Actuator_Torque, public IVP_U_Active_Float_Listener {
    IVP_U_Active_Float *active_float_max_rotation_speed;
    IVP_U_Active_Float *active_float_torque;
    void active_float_changed(IVP_U_Active_Float *calling_mod); // for wakup
public:
    IVP_Actuator_Torque_Active(IVP_Environment *env, IVP_Template_Torque *templ);
    ~IVP_Actuator_Torque_Active();
};

/********************************************************************************
 *	Name:	    	IVP_Listener_Check_Dist_Event
 *	Description:	Callback for check dists events
 ********************************************************************************/
class IVP_Listener_Check_Dist_Event {
public:
    virtual void check_dist_event(IVP_Actuator_Check_Dist *, IVP_BOOL distance_shorter_than_range)=0;
    virtual void check_dist_is_going_to_be_deleted_event(IVP_Actuator_Check_Dist *)=0;	// has to remove itself from IVP_Actuator_Check_Dist
};


/********************************************************************************
 *	Name:	     	IVP_Anchor_Check_Dist  	
 *	Description:	A fixed object space point needed for check dists
 *	Attention:	
 ********************************************************************************/
class IVP_Anchor_Check_Dist : public IVP_Listener_Hull
{
public:
    IVP_HULL_ELEM_TYPE get_type(){ return IVP_HULL_ELEM_ANCHOR; };
    virtual void hull_limit_exceeded_event(IVP_Hull_Manager *, IVP_HTIME);
    virtual void hull_manager_is_going_to_be_deleted_event(IVP_Hull_Manager *);

    IVP_Real_Object *real_object;
    IVP_Actuator_Check_Dist *l_actuator_check_dist;
    IVP_U_Float_Point object_pos;	// original position of object space

    IVP_Anchor_Check_Dist(){; };
    void init_anchor_check_dist(IVP_Real_Object *object, IVP_U_Point *position_world_space, IVP_Actuator_Check_Dist *my_act_check_dist);
    virtual ~IVP_Anchor_Check_Dist();
};

/********************************************************************************
 *	Name:	       	IVP_Actuator_Check_Dist
 *	Description:	compares the distance of two check anchors against a predefined
 *			distance.
 *	Note:		the implementation of this check_dist actuator is pretty
 *			fast as long as anchors between check dists are shared.
 *	Version Info:	In future releases check dists may not subclass IVP_Actuator_Two_Point
 ********************************************************************************/
class IVP_Actuator_Check_Dist
{
    IVP_Anchor_Check_Dist anchors[2];	// the two anchors
    IVP_FLOAT range;
    
    /** Listeners */    
    IVP_U_Vector<IVP_Listener_Check_Dist_Event> listeners_check_dist_event;
    IVP_U_Active_Terminal_Int *mod_is_outside; // optional active_int to write the result to

    void fire_check_dist_event(IVP_BOOL distance_shorter_than_range);
    void fire_check_dist_is_going_to_be_deleted_event( );
protected:
    friend class IVP_Anchor_Check_Dist;
    void hull_limit_exceeded_event();		// internal use
    friend class IVP_Environment;
    IVP_Actuator_Check_Dist(IVP_Environment *env, IVP_Template_Check_Dist *check_dist);
public:    
    void *client_data;		// user definable data
    IVP_BOOL is_outside;

    void set_range(IVP_DOUBLE value);
    
    void add_listener_check_dist_event(IVP_Listener_Check_Dist_Event *listener);
    void remove_listener_check_dist_event(IVP_Listener_Check_Dist_Event *listener);
    
    ~IVP_Actuator_Check_Dist();
};



// INTERN_START
/********************************************************************************
 *	Name:	       	IVP_Actuator_Extra
 *	Description:	not for public use yet
 *			used for various demo purposes
 *	Version Info:	beautification is needed, 
 ********************************************************************************/
class IVP_Actuator_Extra: public IVP_Actuator_Two_Point, public IVP_U_Active_Float_Listener
{
    friend class IVP_Extra_Manager;
    IVP_Extra_Info info;
public:
    IVP_U_Point current_float_cam_pos;
    
    IVP_U_Point current_look_point;
    IVP_BOOL current_vals_are_set;  // for first time

    IVP_U_Point *get_float_cam_props(IVP_DOUBLE *fc_height_out, IVP_DOUBLE *fc_target_height_out, IVP_DOUBLE *fc_dist_out, IVP_DOUBLE *fc_speed_out); // returns &current_float_cam_pos
    
    IVP_DOUBLE get_force();	// needed by bomb

    void do_float_cam(IVP_DOUBLE d_time);
    void do_puck_force(IVP_DOUBLE d_time);
    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
  
    void active_float_changed(IVP_U_Active_Float *calling_mod); // for bombs

    void calc_float_cam_matrix(IVP_U_Matrix *cam_matrix_out);
    IVP_Actuator_Extra(IVP_Environment *env, IVP_Template_Extra *templ);
    virtual ~IVP_Actuator_Extra();
};
// INTERN_END





/********************************************************************************
 *	Name:	       	IVP_Stabilizer_Info
 *	Description:	Values to IVP_Actuator_Stabilizer,
 *			tries to eliminate differences between two distances,
 *			useable for stabilization of a car axis.
 *	Attention:	Too high stabi constants result in higher frequency than
 *			PSI rate and will cause high unnatural speeds of objects
 ********************************************************************************/
class IVP_Actuator_Stabilizer: public IVP_Actuator_Four_Point
{
    friend class IVP_Stabilizer_Manager;
    IVP_Environment *l_environment;

protected:
    IVP_FLOAT stabi_constant;
    friend class IVP_Environment;
    IVP_Actuator_Stabilizer(IVP_Environment *env, IVP_Template_Stabilizer *stabi);
    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
public:
    void set_stabi_constant(IVP_DOUBLE);
    
    virtual ~IVP_Actuator_Stabilizer();
};

#if !defined( IVP_ACTUATOR_SPRING_INCLUDED )
#	include <ivp_actuator_spring.hxx>
#endif

#endif /* ifndef IVP_ACTUATOR_INCLUDED */
