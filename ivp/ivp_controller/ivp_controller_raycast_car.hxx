// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_CONTROLLER_RAYCAST_CAR_INCLUDED
#define IVP_CONTROLLER_RAYCAST_CAR_INCLUDED


#ifndef WIN32
#	pragma interface
#endif

#define IVP_RAYCAST_CAR_MAX_WHEELS 12

/********************************************************************************
 *	Name:	     	IVP_Controller_Raycast_Car  	
 *	Description:	implementation of the IVP_Car_System
 *			This class uses no real wheels but simulates them based
 *			on a ray caster.
 *	Note:		The driving behaviour of this car is nearly identical
 *			of the real car, but CPU time needed is significant lower
 *			(1/3 - 1/5)
 *	Note:		This is an abstract class,
 *			you need to subclass it and implement the do_raycasts function
 *			(example implementation: IVP_Example_Controller_Raycast_Car)
 *	Attention:	This car does influence moving objects using it's wheels
 *			This feature will be implemented soon
 ********************************************************************************/

class IVP_Raycast_Car_Wheel {
public:
    // static section
    IVP_U_Float_Point hp_cs;	// hard point core system projected on y plane
    IVP_U_Float_Point spring_direction_cs;

    IVP_FLOAT distance_orig_hp_to_hp;   // distance hp is moved by projecting it onto the y - plane
    IVP_FLOAT spring_len;  		// == pretension + distance_orig_hp_to_hp
    IVP_FLOAT spring_constant;
    IVP_FLOAT spring_damp_relax;
    IVP_FLOAT spring_damp_compress;
    IVP_FLOAT max_rotation_speed;
    
    IVP_FLOAT wheel_radius;
    IVP_FLOAT inv_wheel_radius;
    IVP_FLOAT friction_of_wheel;
    
    // dynamic section
    IVP_FLOAT torque;
    IVP_BOOL  wheel_is_fixed; // eg. handbrake
    IVP_U_Float_Point axis_direction_cs;
    IVP_FLOAT angle_wheel;    // to simulate rotation
    IVP_FLOAT wheel_angular_velocity;
    
    // out
    IVP_U_Float_Point surface_speed_of_wheel_on_ground_ws;
    IVP_FLOAT pressure;
    IVP_FLOAT raycast_dist;
};


class IVP_Raycast_Car_Wheel_Temp {
public:
    IVP_FLOAT friction_value;
    IVP_FLOAT stabilizer_force;
    IVP_Real_Object *moveable_object_hit_by_ray;
    IVP_U_Float_Point ground_normal_ws;
    
    IVP_U_Point ground_hit_ws;
    IVP_U_Float_Point spring_direction_ws;
    IVP_U_Float_Point surface_speed_wheel_ws;
    IVP_U_Float_Point projected_surface_speed_wheel_ws;
    IVP_U_Float_Point axis_direction_ws;
    IVP_U_Float_Point projected_axis_direction_ws;

    IVP_FLOAT forces_needed_to_drive_straight;
    IVP_FLOAT inv_normal_dot_dir;
};

class IVP_Raycast_Car_Axis {
public:
    IVP_FLOAT stabilizer_constant;
};

class IVP_Ray_Solver_Template;
class IVP_Ray_Hit;
class IVP_Event_Sim;

class IVP_Controller_Raycast_Car_Vector_of_Cores_1: public IVP_U_Vector<IVP_Core> {
    void *elem_buffer[1];
public:
    IVP_Controller_Raycast_Car_Vector_of_Cores_1(): IVP_U_Vector<IVP_Core>( &elem_buffer[0],1 ){;};
};

class IVP_Controller_Raycast_Car : public IVP_Car_System, protected IVP_Controller_Dependent  {
private:
    IVP_Controller_Raycast_Car_Vector_of_Cores_1 vector_of_cores;
protected:
    short n_wheels;
    short n_axis;
    short wheels_per_axis;
    IVP_Raycast_Car_Wheel wheels_of_car[IVP_RAYCAST_CAR_MAX_WHEELS];
    IVP_Raycast_Car_Axis  axis_of_car[IVP_RAYCAST_CAR_MAX_WHEELS/2];
    
    IVP_Real_Object *car_body;
    IVP_FLOAT gravity_y_direction; //  +/-1
    IVP_U_Float_Point normized_gravity_ws;
    
    IVP_FLOAT max_speed;
    IVP_COORDINATE_INDEX index_x;
    IVP_COORDINATE_INDEX index_y;
    IVP_COORDINATE_INDEX index_z;
    IVP_BOOL is_left_handed;
    IVP_FLOAT extra_gravity;

    IVP_FLOAT down_force;
    IVP_FLOAT down_force_vertical_offset;

    // booster
    IVP_FLOAT booster_force;
    IVP_FLOAT booster_seconds_to_go;
    IVP_FLOAT booster_seconds_until_ready;

    // steering
    IVP_FLOAT steering_angle;

	// debug
	IVP_CarSystemDebugData_t	m_CarSystemDebugData;
    
    IVP_Raycast_Car_Wheel *get_wheel( IVP_POS_WHEEL i) { return & wheels_of_car[i]; };
    IVP_Raycast_Car_Axis *get_axis( IVP_POS_AXIS i) { return & axis_of_car[i]; };

    virtual void core_is_going_to_be_deleted_event(IVP_Core *){ P_DELETE_THIS(this); };
    virtual IVP_U_Vector<IVP_Core> *get_associated_controlled_cores(){ return &vector_of_cores; };
    
    virtual void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
    virtual IVP_CONTROLLER_PRIORITY get_controller_priority();

	// Initialization.
	void InitRaycastCarEnvironment( IVP_Environment *pEnvironment, const IVP_Template_Car_System *pCarSystemTemplate );
	void InitRaycastCarBody( const IVP_Template_Car_System *pCarSystemTemplate );
	void InitRaycastCarWheels( const IVP_Template_Car_System *pCarSystemTemplate );
	void InitRaycastCarAxes( const IVP_Template_Car_System *pCarSystemTemplate );

	// Simulation.
	void SetupWheelRaycasts( IVP_Ray_Solver_Template *pRaySolverTemplates, const IVP_U_Matrix *m_world_f_core, IVP_Raycast_Car_Wheel_Temp *pTempWheels );
	
	bool DoSimulationWheels( IVP_Ray_Solver_Template *pRaySolverTemplates, const IVP_U_Matrix *m_world_f_core, IVP_Raycast_Car_Wheel_Temp *pTempWheels,
							 IVP_Ray_Hit *pRayHits, IVP_FLOAT *pFrictions, IVP_Core *pCarCore );
	void DoSimulationStabilizers( IVP_Raycast_Car_Wheel_Temp *pTempWheels );
	void DoSimulationShocks( IVP_Raycast_Car_Wheel_Temp *pTempWheels, IVP_Ray_Hit *pRayHits, IVP_Event_Sim *pEventSim, IVP_Core *pCarCore );
	void DoSimulationBooster( IVP_Event_Sim *pEventSim, IVP_Core *pCarCore );
	void DoSimulationSteering( IVP_Raycast_Car_Wheel_Temp *pTempWheels, IVP_Core *pCarCore, IVP_Event_Sim *pEventSim );

	void CalcSteeringForces( IVP_Raycast_Car_Wheel_Temp *pTempWheels, IVP_Core *pCarCore, IVP_Event_Sim *pEventSim, IVP_FLOAT *pForcesNeededToDriveStraight );
	void ApplySteeringForces( IVP_Raycast_Car_Wheel_Temp *pTempWheels, IVP_Core *pCareCore, IVP_Event_Sim *pEventSim, IVP_FLOAT *pForcesNeededToDriveStraight );

protected:
    
    virtual void do_raycasts( IVP_Event_Sim *, int n_wheels, IVP_Ray_Solver_Template* t_in,
			      class IVP_Ray_Hit* hits_out,
			      IVP_FLOAT *friction_of_object_out )=0;
    
public:
    void do_steering_wheel(IVP_POS_WHEEL wheel_pos, IVP_FLOAT s_angle); // called by do_steering()

    // Car Adjustment
    void change_spring_constant(IVP_POS_WHEEL pos, IVP_FLOAT spring_constant); // [Newton/meter]
    void change_spring_dampening(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening); // when spring is relaxing spring
    void change_spring_dampening_compression(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening); // [Newton/meter] for compressing spring
    void change_max_body_force(IVP_POS_WHEEL , IVP_FLOAT /*mforce*/){;};
    void change_spring_pre_tension(IVP_POS_WHEEL pos, IVP_FLOAT pre_tension_length);
	void change_spring_length(IVP_POS_WHEEL pos, IVP_FLOAT spring_length);

    void change_stabilizer_constant(IVP_POS_AXIS pos, IVP_FLOAT stabi_constant); // [Newton/meter]
	void change_fast_turn_factor( IVP_FLOAT fast_turn_factor_ );  // not implemented for raycasts
    void change_wheel_torque(IVP_POS_WHEEL pos, IVP_FLOAT torque);
    
	void update_throttle( IVP_FLOAT flThrottle ) {}

    void update_body_countertorque(){;};
    
    void change_body_downforce(IVP_FLOAT force);        // extra force to keep flipped objects flipped over

    void fix_wheel( IVP_POS_WHEEL, IVP_BOOL stop_wheel ); // stop wheel completely (e.g. handbrake )
    
    // Car Info
    IVP_DOUBLE get_body_speed(IVP_COORDINATE_INDEX idx_z = IVP_INDEX_Z); // km/h in 'z' direction
    IVP_DOUBLE get_wheel_angular_velocity(IVP_POS_WHEEL);
    IVP_DOUBLE get_orig_front_wheel_distance();
    IVP_DOUBLE get_orig_axles_distance();
	void get_skid_info( IVP_Wheel_Skid_Info *array_of_skid_info_out);
    
    void get_wheel_position(IVP_U_Point *position_ws_out, IVP_U_Quat *direction_ws_out);
  
    /**** Methods: 2nd Level, based on primitives ****/
    /**** Methods: 2nd Level, based on primitives ****/
    virtual void do_steering(IVP_FLOAT steering_angle_in); // default implementation updates this->steering_angle
    
    virtual void set_booster_acceleration( IVP_FLOAT acceleration);
    virtual void activate_booster(IVP_FLOAT thrust, IVP_FLOAT duration, IVP_FLOAT delay);
    virtual void update_booster(IVP_FLOAT /*delta_time*/){;};
	//virtual IVP_FLOAT IVP_Controller_Raycast_Car::get_booster_delay();
	virtual IVP_FLOAT get_booster_delay(); //lwss remove extra qualification

    IVP_Controller_Raycast_Car(IVP_Environment *environment, const IVP_Template_Car_System *);
    virtual ~IVP_Controller_Raycast_Car();

	// Debug
	void SetCarSystemDebugData( const IVP_CarSystemDebugData_t &carSystemDebugData );
	void GetCarSystemDebugData( IVP_CarSystemDebugData_t &carSystemDebugData );
};

#endif
