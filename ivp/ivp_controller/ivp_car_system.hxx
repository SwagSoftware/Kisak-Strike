// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

#if !defined(IVP_CAR_SYSTEM_INCLUDED)
#	define IVP_CAR_SYSTEM_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_LISTENER_PSI_INCLUDED
#	include <ivp_listener_psi.hxx>
#endif

enum IVP_POS_WHEEL{
  IVP_FRONT_LEFT  = 0,
  IVP_FRONT_RIGHT = 1,
  IVP_REAR_LEFT   = 2,
  IVP_REAR_RIGHT  = 3,
  IVP_CAR_SYSTEM_MAX_WHEELS = 10
};

enum IVP_POS_AXIS {
    IVP_FRONT  = 0,
    IVP_REAR = 1,
    IVP_CAR_SYSTEM_MAX_AXIS = 5
};

class IVP_Real_Object;
class IVP_Actuator_Torque;
class IVP_Constraint_Solver_Car;


class IVP_Template_Car_System {
public:
    // Fill in template to build up a functional
    // car out of given (and set up) IVP_Real_Objects.
    int n_wheels;
    int n_axis;
    
    /*** Coordinate System Data ***/
    /*** Coordinate System Data ***/
    IVP_COORDINATE_INDEX index_x;
    IVP_COORDINATE_INDEX index_y;
    IVP_COORDINATE_INDEX index_z;
    IVP_BOOL is_left_handed;

    /*** Instance Data ***/
    /*** Instance Data ***/

    IVP_Real_Object *car_body;

   /********************************************************************************
    *	Name:		car_wheel
    *	Description:	the wheels of the car
    *   Note:		Not needed of IVP_Controller_Raycast_Car
    ********************************************************************************/    
    IVP_Real_Object *car_wheel[IVP_CAR_SYSTEM_MAX_WHEELS]; // index according to IVP_POS_WHEEL

    IVP_FLOAT friction_of_wheel[IVP_CAR_SYSTEM_MAX_WHEELS]; // to be set for IVP_Controller_Raycast_Car
    

    IVP_FLOAT wheel_radius[IVP_CAR_SYSTEM_MAX_WHEELS];
    
    IVP_FLOAT wheel_reversed_sign[IVP_CAR_SYSTEM_MAX_WHEELS];  // Default: 1.0f. Use -1.0f when wheels are turned by 180 degrees.
    
  
    IVP_FLOAT body_counter_torque_factor; // (e.g. 0.3f) for nose dive etc. produced by wheel torque

    IVP_FLOAT extra_gravity_force_value;		// additional gravity force
	IVP_FLOAT extra_gravity_height_offset;      // force anchor height offset relative to center of mass

    IVP_FLOAT body_down_force_vertical_offset;  // vertical offset to mass center for tilted object
	IVP_FLOAT fast_turn_factor;
    /*** Archetype Data ***/
    /*** Archetype Data ***/

    // Standard wheel position
    IVP_U_Float_Point wheel_pos_Bos[IVP_CAR_SYSTEM_MAX_WHEELS]; // standard position of wheel centers in body's object system
    IVP_U_Float_Point trace_pos_Bos[IVP_CAR_SYSTEM_MAX_WHEELS]; // standard position of trace centers in body's object system

    // Springs
    IVP_FLOAT spring_constant[IVP_CAR_SYSTEM_MAX_WHEELS]; // [Newton/m]
    IVP_FLOAT spring_dampening[IVP_CAR_SYSTEM_MAX_WHEELS]; // for releasing springs
    IVP_FLOAT spring_dampening_compression[IVP_CAR_SYSTEM_MAX_WHEELS]; // for compressing springs
    IVP_FLOAT max_body_force[IVP_CAR_SYSTEM_MAX_WHEELS];    // clipping of spring forces. Reduces the jumpy behaviour of cars with heavy wheels
    IVP_FLOAT spring_pre_tension[IVP_CAR_SYSTEM_MAX_WHEELS]; // [m] used to keep the wheels at the original position
	IVP_FLOAT raycast_startpoint_height_offset;

    // Stabilizer
    IVP_FLOAT stabilizer_constant[IVP_CAR_SYSTEM_MAX_AXIS]; // [Newton/m]

    // Etc
    IVP_FLOAT wheel_max_rotation_speed[IVP_CAR_SYSTEM_MAX_AXIS];

	// Car System Setup
    IVP_Template_Car_System( int n_wheels_, int n_axis_ ) 
	{
		P_MEM_CLEAR( this );
		
		n_wheels = n_wheels_;
		n_axis = n_axis_;
		
		for( int i = 0; i < n_wheels; i++ )
		{
			// not reversed by default
			this->wheel_reversed_sign[i] = 1.0f; 
			
			// default coordinate system definition
			index_x = IVP_INDEX_X;
			index_y = IVP_INDEX_Y;
			index_z = IVP_INDEX_Z;
			is_left_handed = IVP_FALSE;
		}
		
		fast_turn_factor = 1.0f;
    };
};

class IVP_Wheel_Skid_Info {
	public:
		IVP_U_Float_Point		last_contact_position_ws;
		IVP_FLOAT				last_skid_value;  // 0 means no skidding, values > 0 means skidding, check yourself for reasonable ranges
		IVP_Time				last_skid_time;
};

struct IVP_CarSystemDebugData_t
{
	IVP_U_Point	wheelRaycasts[IVP_CAR_SYSTEM_MAX_WHEELS][2];		// wheels, start = 0, end = 1
	IVP_FLOAT	wheelRaycastImpacts[IVP_CAR_SYSTEM_MAX_WHEELS];		// wheels, impact raycast floating point min distance

	// 4-wheel vehicle.
	IVP_FLOAT	wheelRotationalTorque[4][3];
	IVP_FLOAT	wheelTranslationTorque[4][3];
};

class IVP_Car_System {
public:
    virtual ~IVP_Car_System();
    IVP_Car_System();

    virtual void do_steering_wheel(IVP_POS_WHEEL wheel_pos, IVP_FLOAT s_angle)=0; // called by do_steering()

    // Car Adjustment
    virtual void change_spring_constant(IVP_POS_WHEEL pos, IVP_FLOAT spring_constant)=0; // [Newton/meter]
    virtual void change_spring_dampening(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening)=0; // when spring is relaxing spring
    virtual void change_spring_dampening_compression(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening)=0; // [Newton/meter] for compressing spring
    virtual void change_max_body_force(IVP_POS_WHEEL wheel_nr, IVP_FLOAT mforce) = 0;
    virtual void change_spring_pre_tension(IVP_POS_WHEEL pos, IVP_FLOAT pre_tension_length)=0;
	virtual void change_spring_length(IVP_POS_WHEEL wheel_nr, IVP_FLOAT spring_length) = 0;

    virtual void change_stabilizer_constant(IVP_POS_AXIS pos, IVP_FLOAT stabi_constant)=0; // [Newton/meter]
	virtual void change_fast_turn_factor( IVP_FLOAT fast_turn_factor ) = 0;
  
    virtual void change_wheel_torque(IVP_POS_WHEEL pos, IVP_FLOAT torque)=0;
    virtual void update_body_countertorque()=0;		// rotate the body in the opposite direction of the wheels
    
	virtual void update_throttle( IVP_FLOAT flThrottle ) = 0;

    virtual void change_body_downforce(IVP_FLOAT force)=0; // extra force to keep flipped objects flipped over

    virtual void fix_wheel( IVP_POS_WHEEL, IVP_BOOL stop_wheel )=0; // stop wheel completely (e.g. handbrake )
    
    // Car Info
    virtual IVP_DOUBLE get_body_speed(IVP_COORDINATE_INDEX idx_z = IVP_INDEX_Z)=0; // km/h in 'z' direction
    virtual IVP_DOUBLE get_wheel_angular_velocity(IVP_POS_WHEEL)=0;
    virtual void update_wheel_positions()=0;   // move graphical wheels to correct position
    virtual IVP_DOUBLE get_orig_front_wheel_distance()=0;
    virtual IVP_DOUBLE get_orig_axles_distance()=0;
	virtual void get_skid_info( IVP_Wheel_Skid_Info *array_of_skid_info_out) = 0;

    // Tools
    static IVP_FLOAT calc_ackerman_angle(IVP_FLOAT alpha, IVP_FLOAT dx, IVP_FLOAT dz); // alpha refers to innermost wheel

    
    /**** Methods: 2nd Level, based on primitives ****/
    /**** Methods: 2nd Level, based on primitives ****/
    //lwss hack, this airboat crap is not defined for some reason
    //virtual void do_steering(IVP_FLOAT steering_angle_in) = 0; // updates this->steering_angle
    //lwss end
    virtual void set_booster_acceleration(IVP_FLOAT acceleration) = 0;    // set an additional accerleration force
    virtual void activate_booster(IVP_FLOAT thrust, IVP_FLOAT duration, IVP_FLOAT recharge_time) =0; // set a temporary acceleration force as a factor of gravity
    virtual void update_booster(IVP_FLOAT delta_time)=0; // should be called every frame to allow the physics system to deactivate a booster
	virtual IVP_FLOAT get_booster_delay() = 0;

	// Debug (Getting debug data out to vphysics and the engine to be rendered!)
	virtual void SetCarSystemDebugData( const IVP_CarSystemDebugData_t &carSystemDebugData ) = 0;
	virtual void GetCarSystemDebugData( IVP_CarSystemDebugData_t &carSystemDebugData ) = 0;
};


class IVP_Car_System_Real_Wheels : public IVP_Car_System {
private:
    IVP_Environment *environment;
    int n_wheels;      // number of wheels
    int n_axis;
protected:
    IVP_Real_Object *car_body;
    IVP_Real_Object *car_wheel[IVP_CAR_SYSTEM_MAX_WHEELS]; // index according to IVP_POS_WHEEL

    IVP_Constraint_Solver_Car *car_constraint_solver;

    IVP_Actuator_Torque *car_act_torque_body;
    IVP_Actuator_Torque *car_act_torque[IVP_CAR_SYSTEM_MAX_WHEELS];
    
    IVP_Actuator_Suspension *car_spring[IVP_CAR_SYSTEM_MAX_WHEELS];
    IVP_Actuator_Stabilizer *car_stabilizer[IVP_CAR_SYSTEM_MAX_AXIS];

    IVP_Actuator_Force  *car_act_down_force;
    IVP_Actuator_Force  *car_act_extra_gravity;

    IVP_Constraint *fix_wheel_constraint[IVP_CAR_SYSTEM_MAX_WHEELS];

    IVP_FLOAT wheel_reversed_sign[IVP_CAR_SYSTEM_MAX_WHEELS]; // Default: 1.0f. Use -1.0f when wheels are turned by 180 degrees.  
    IVP_FLOAT wheel_radius[IVP_CAR_SYSTEM_MAX_WHEELS];

    IVP_FLOAT body_counter_torque_factor; // response to wheel torque, e.g. 0.3f (for nose dive etc.), call update_body_countertorque() when all wheel torques are changed

    IVP_FLOAT max_speed;
	IVP_FLOAT fast_turn_factor;   // factor to angular accelerates the car if the wheels are steered ( 0.0f no effect 1.0f strong effect )
    // booster
    IVP_Actuator_Force *booster_actuator[2];
    IVP_FLOAT booster_seconds_to_go;
    IVP_FLOAT booster_seconds_until_ready;

    
    IVP_FLOAT steering_angle;

	// Debug
	IVP_CarSystemDebugData_t	m_CarSystemDebugData;

    virtual void environment_will_be_deleted(IVP_Environment *);
public:
    /**** Methods: Primitives ****/
    /**** Methods: Primitives ****/

    // Car Basics
    IVP_Car_System_Real_Wheels(IVP_Environment *environment,	 IVP_Template_Car_System *);
    virtual ~IVP_Car_System_Real_Wheels();
    
    void do_steering_wheel(IVP_POS_WHEEL wheel_pos, IVP_FLOAT s_angle); // called by do_steering()

    // Car Adjustment
    void change_spring_constant(IVP_POS_WHEEL pos, IVP_FLOAT spring_constant); // [Newton/meter]
    void change_spring_dampening(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening); // when spring is relaxing spring
    void change_spring_dampening_compression(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening); // [Newton/meter] for compressing spring
    void change_max_body_force(IVP_POS_WHEEL wheel_nr, IVP_FLOAT mforce);
    void change_spring_pre_tension(IVP_POS_WHEEL pos, IVP_FLOAT pre_tension_length);
	void change_spring_length(IVP_POS_WHEEL wheel_nr, IVP_FLOAT spring_length);

    void change_stabilizer_constant(IVP_POS_AXIS pos, IVP_FLOAT stabi_constant); // [Newton/meter]
	void change_fast_turn_factor( IVP_FLOAT );  
    void change_wheel_torque(IVP_POS_WHEEL pos, IVP_FLOAT torque);
	void change_wheel_speed_dampening( IVP_POS_WHEEL wheel_nr, IVP_FLOAT dampening );
    void update_body_countertorque();		// rotate the body in the opposite direction of the wheels

   	void update_throttle( IVP_FLOAT /*flThrottle*/ ) {}
 
    void change_body_downforce(IVP_FLOAT force); // extra force to keep flipped objects flipped over

    void fix_wheel( IVP_POS_WHEEL, IVP_BOOL stop_wheel ); // stop wheel completely (e.g. handbrake )
    
    // Car Info
    IVP_DOUBLE get_body_speed(IVP_COORDINATE_INDEX idx_z = IVP_INDEX_Z); // km/h in 'z' direction
    IVP_DOUBLE get_wheel_angular_velocity(IVP_POS_WHEEL);
    void update_wheel_positions(){;};
    IVP_DOUBLE get_orig_front_wheel_distance();
    IVP_DOUBLE get_orig_axles_distance();
	void get_skid_info( IVP_Wheel_Skid_Info *array_of_skid_info_out);
  
    /**** Methods: 2nd Level, based on primitives ****/
    /**** Methods: 2nd Level, based on primitives ****/
    virtual void do_steering(IVP_FLOAT steering_angle_in); // updates this->steering_angle
    
    void set_booster_acceleration(IVP_FLOAT acceleration);
    void activate_booster(IVP_FLOAT thrust, IVP_FLOAT duration, IVP_FLOAT recharge_time);
    void update_booster(IVP_FLOAT delta_time);
	virtual IVP_FLOAT get_booster_delay();

	// Debug
	void SetCarSystemDebugData( const IVP_CarSystemDebugData_t &carSystemDebugData );
	void GetCarSystemDebugData( IVP_CarSystemDebugData_t &carSystemDebugData );
};


#endif /* defined IVP_CAR_SYSTEM_INCLUDED */
