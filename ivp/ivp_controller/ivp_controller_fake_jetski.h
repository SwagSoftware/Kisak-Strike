//========= Copyright © 1996-2002, Valve LLC, All rights reserved. ============
//
// Purpose: 4-Wheel Vehicle attempt at a jetski/waverunner!
//
//=============================================================================

// Some code.
// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef IVP_CONTROLLER_FAKE_JETSKI_H
#define IVP_CONTROLLER_FAKE_JETSKI_H
#ifndef WIN32
#pragma interface
#endif

#include "ivp_car_system.hxx"
#include "ivp_controller.hxx"

#define IVP_RAYCAST_FAKE_JETSKI_MAX_WHEELS		4

//=============================================================================
//
// Fake Jetski Classes
//

//-----------------------------------------------------------------------------
// Fake Jetski - Fake Wheel
//-----------------------------------------------------------------------------
class IVP_Raycast_Fake_Jetski_Wheel 
{
public:

    // static section
    IVP_U_Float_Point	hp_cs;										// hard point core system projected on y plane
	IVP_U_Float_Point	raycast_start_cs;							// ray cast start position

    IVP_U_Float_Point	spring_direction_cs;						// spring direction in "jetski-space"	(cs = "car-space")
    IVP_FLOAT			distance_orig_hp_to_hp;						// distance hp is moved by projecting it onto the y - plane
    IVP_FLOAT			spring_len;  								// == pretension + distance_orig_hp_to_hp
    IVP_FLOAT			spring_constant;							// shock at wheel spring constant
    IVP_FLOAT			spring_damp_relax;							// shock at wheel spring dampening during relaxation
    IVP_FLOAT			spring_damp_compress;						// shock at wheel spring dampening during compression

    IVP_FLOAT			max_rotation_speed;							// max rotational speed of the wheel
    
    IVP_FLOAT			wheel_radius;								// wheel radius
    IVP_FLOAT			inv_wheel_radius;							// inverse wheel radius
    IVP_FLOAT			friction_of_wheel;							// wheel friction
    
    // dynamic section
    IVP_FLOAT			torque;										// torque applied to wheel
    IVP_BOOL			wheel_is_fixed;								// eg. handbrake (fixed = stationary)
    IVP_U_Float_Point	axis_direction_cs;							// axle direction in "jetski-space" (cs = "car-space")
    IVP_FLOAT			angle_wheel;								// wheel angle
    IVP_FLOAT			wheel_angular_velocity;						// angular velocity of wheel
    
    // out
    IVP_U_Float_Point	surface_speed_of_wheel_on_ground_ws;		// actual speed in world-space
    IVP_FLOAT			pressure;									// force from gravity, mass of car, stabilizers, etc. on wheel
    IVP_FLOAT			raycast_dist;								// raycast distance to impact for wheel
};

//-----------------------------------------------------------------------------
// Fake Jetski - Fake Wheel Temp
//-----------------------------------------------------------------------------
class IVP_Raycast_Fake_Jetski_Wheel_Temp 
{
public:

    IVP_FLOAT			friction_value;								// combined (multiply) frictional value of impact surface and wheel
    IVP_FLOAT			stabilizer_force;							// force on wheel due to axle stabilization
    IVP_Real_Object		*moveable_object_hit_by_ray;				// moveable physics object hit by raycast
    IVP_U_Float_Point	ground_normal_ws;							// impact normal in world-space
    
    IVP_U_Point			ground_hit_ws;								// impact point in world-space
    IVP_U_Float_Point	spring_direction_ws;						// spring direction (raycast for impact direction) in world-space
    IVP_U_Float_Point	surface_speed_wheel_ws;						// wheel speed in world-space
    IVP_U_Float_Point	projected_surface_speed_wheel_ws;			// ???
    IVP_U_Float_Point	axis_direction_ws;							// axle direction in world-space
    IVP_U_Float_Point	projected_axis_direction_ws;				// ???

    IVP_FLOAT			forces_needed_to_drive_straight;			// forces need to keep the vehicle driving straight (attempt and directional wheel friction)
    IVP_FLOAT			inv_normal_dot_dir;							// ???
};

//-----------------------------------------------------------------------------
// Fake Jetski - Fake Axle
//-----------------------------------------------------------------------------
class IVP_Raycast_Fake_Jetski_Axle 
{
public:

    IVP_FLOAT			stabilizer_constant;						// axle (for wheels) stabilizer constant
};

class IVP_Ray_Solver_Template;
class IVP_Ray_Hit;
class IVP_Event_Sim;

//-----------------------------------------------------------------------------
// Fake Jetski - Core List
//-----------------------------------------------------------------------------
class IVP_Controller_Raycast_Fake_Jetski_Vector_of_Cores_1: public IVP_U_Vector<IVP_Core> 
{
    void *elem_buffer[1];

public:

    IVP_Controller_Raycast_Fake_Jetski_Vector_of_Cores_1(): IVP_U_Vector<IVP_Core>( &elem_buffer[0],1 ) { ; };
};

//-----------------------------------------------------------------------------
// Fake Jetski
//-----------------------------------------------------------------------------
class IVP_Controller_Raycast_Fake_Jetski : public IVP_Car_System, protected IVP_Controller_Dependent  

{
public:

	// Constructor/Deconstructor.
    IVP_Controller_Raycast_Fake_Jetski( IVP_Environment *environment, const IVP_Template_Car_System* );
    virtual ~IVP_Controller_Raycast_Fake_Jetski();

	// Steering
    void							do_steering_wheel(IVP_POS_WHEEL wheel_pos, IVP_FLOAT s_angle);						// called by do_steering()

    // Car Adjustment
    void							change_spring_constant(IVP_POS_WHEEL pos, IVP_FLOAT spring_constant);				// [Newton/meter]
    void							change_spring_dampening(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening);				// when spring is relaxing spring
    void							change_spring_dampening_compression(IVP_POS_WHEEL pos, IVP_FLOAT spring_dampening); // [Newton/meter] for compressing spring
    void							change_max_body_force(IVP_POS_WHEEL , IVP_FLOAT /*mforce*/){;};
    void							change_spring_pre_tension(IVP_POS_WHEEL pos, IVP_FLOAT pre_tension_length);
	void							change_spring_length(IVP_POS_WHEEL pos, IVP_FLOAT spring_length);

    void							change_stabilizer_constant(IVP_POS_AXIS pos, IVP_FLOAT stabi_constant);				// [Newton/meter]
	void							change_fast_turn_factor( IVP_FLOAT fast_turn_factor_ );								// not implemented for raycasts
    void							change_wheel_torque(IVP_POS_WHEEL pos, IVP_FLOAT torque);
    
    void							update_body_countertorque(){;};
    
    void							change_body_downforce(IVP_FLOAT force);												// extra force to keep flipped objects flipped over

    void							fix_wheel( IVP_POS_WHEEL, IVP_BOOL stop_wheel );									// stop wheel completely (e.g. handbrake )
	void							change_friction_of_wheel( IVP_POS_WHEEL pos, IVP_FLOAT friction );

    // Car Info
    IVP_DOUBLE						get_body_speed(IVP_COORDINATE_INDEX idx_z = IVP_INDEX_Z);							// km/h in 'z' direction
    IVP_DOUBLE						get_wheel_angular_velocity(IVP_POS_WHEEL);
    IVP_DOUBLE						get_orig_front_wheel_distance();
    IVP_DOUBLE						get_orig_axles_distance();
	void							get_skid_info( IVP_Wheel_Skid_Info *array_of_skid_info_out);
    
    void							get_wheel_position(IVP_U_Point *position_ws_out, IVP_U_Quat *direction_ws_out);
  
    /**** Methods: 2nd Level, based on primitives ****/
    /**** Methods: 2nd Level, based on primitives ****/
    virtual void					do_steering(IVP_FLOAT steering_angle_in);											// default implementation updates this->steering_angle
    
    virtual void					set_booster_acceleration( IVP_FLOAT acceleration);
    virtual void					activate_booster(IVP_FLOAT thrust, IVP_FLOAT duration, IVP_FLOAT delay);
    virtual void					update_booster(IVP_FLOAT /*delta_time*/){;};    
	virtual IVP_FLOAT				get_booster_delay();
    
	// Debug
	void							SetCarSystemDebugData( const IVP_CarSystemDebugData_t &carSystemDebugData );
	void							GetCarSystemDebugData( IVP_CarSystemDebugData_t &carSystemDebugData );

protected:

	// Jetski body.
    IVP_Real_Object					*m_pJetskiBody;			// *car_body

	// Wheels/Axles.
    short							n_wheels;
    short							n_axis;
    short							wheels_per_axis;
    IVP_Raycast_Fake_Jetski_Wheel	m_aJetskiWheels[IVP_RAYCAST_FAKE_JETSKI_MAX_WHEELS];		// wheel_of_car
    IVP_Raycast_Fake_Jetski_Axle	m_aJetskiAxles[IVP_RAYCAST_FAKE_JETSKI_MAX_WHEELS/2];		// axis_of_jetski

	// Gravity.
    IVP_FLOAT						gravity_y_direction; //  +/-1
    IVP_U_Float_Point				normized_gravity_ws;
    IVP_FLOAT						extra_gravity;
    
	// Orientation.
    IVP_COORDINATE_INDEX			index_x;
    IVP_COORDINATE_INDEX			index_y;
    IVP_COORDINATE_INDEX			index_z;
    IVP_BOOL						is_left_handed;

	// Speed.
    IVP_FLOAT						max_speed;

	//
    IVP_FLOAT						down_force;
    IVP_FLOAT						down_force_vertical_offset;

    // Booster.
    IVP_FLOAT						booster_force;
    IVP_FLOAT						booster_seconds_to_go;
    IVP_FLOAT						booster_seconds_until_ready;

    // Steering
    IVP_FLOAT						m_SteeringAngle;
	IVP_FLOAT						m_SteeringRate;

	// Debugging!
	IVP_CarSystemDebugData_t		m_CarSystemDebugData;
    
protected:

    IVP_Raycast_Fake_Jetski_Wheel	*get_wheel( IVP_POS_WHEEL i )						{ return &m_aJetskiWheels[i]; };
    IVP_Raycast_Fake_Jetski_Axle		*get_axle( IVP_POS_AXIS i )							{ return &m_aJetskiAxles[i]; };

    virtual void					core_is_going_to_be_deleted_event( IVP_Core * )		{ P_DELETE_THIS(this); };
    virtual IVP_U_Vector<IVP_Core>  *get_associated_controlled_cores( void )			{ return &vector_of_cores; };
    
    virtual void                    do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
    virtual IVP_CONTROLLER_PRIORITY get_controller_priority();

	// Initialization.
	void							InitRaycastCarEnvironment( IVP_Environment *pEnvironment, const IVP_Template_Car_System *pCarSystemTemplate );
	void							InitRaycastCarBody( const IVP_Template_Car_System *pCarSystemTemplate );
	void							InitRaycastCarWheels( const IVP_Template_Car_System *pCarSystemTemplate );
	void							InitRaycastCarAxes( const IVP_Template_Car_System *pCarSystemTemplate );

	// Simulation.
	void							SetupWheelRaycasts( IVP_Ray_Solver_Template *pRaySolverTemplates, const IVP_U_Matrix *m_world_f_core, IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels );	
	bool							DoSimulationWheels( const IVP_U_Matrix *m_world_f_core, IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels, IVP_Ray_Hit *pRayHits, IVP_FLOAT *pFrictions, IVP_Core *pCarCore );
	void							DoSimulationStabilizers( IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels );
	void							DoSimulationShocks( IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels, IVP_Ray_Hit *pRayHits, IVP_Event_Sim *pEventSim, IVP_Core *pCarCore );
	void							DoSimulationBooster( IVP_Event_Sim *pEventSim, IVP_Core *pCarCore );
	void							DoSimulationSteering( IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels, IVP_Core *pCarCore, IVP_Event_Sim *pEventSim );

	void							CalcSteeringForces( IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels, IVP_Core *pCarCore, IVP_Event_Sim *pEventSim, IVP_FLOAT *pForcesNeededToDriveStraight );
	void							ApplySteeringForces( IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels, IVP_Core *pCareCore, IVP_Event_Sim *pEventSim, IVP_FLOAT *pForcesNeededToDriveStraight );
	IVP_DOUBLE						GetSteeringForceRotational( int iWheel, IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels, IVP_Core *pCarCore, IVP_Event_Sim *pEventSim, IVP_FLOAT *pForcesNeededToDriveStraight );

	void							TurnInPlaceHack( IVP_Raycast_Fake_Jetski_Wheel_Temp *pTempWheels, IVP_Core *pCarCore, IVP_Event_Sim *pEventSim );

protected:
    
	// Pure Virtual
    virtual void do_raycasts( IVP_Event_Sim *, int n_wheels, IVP_Ray_Solver_Template* t_in, class IVP_Ray_Hit* hits_out, IVP_FLOAT *friction_of_object_out ) = 0;
    
private:

    IVP_Controller_Raycast_Fake_Jetski_Vector_of_Cores_1 vector_of_cores;

};

#endif	// IVP_CONTROLLER_FAKE_JETSKI_H

