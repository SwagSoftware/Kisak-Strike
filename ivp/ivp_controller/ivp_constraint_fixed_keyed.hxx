// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_CONSTRAINT_LOCAL_INCLUDED
//#	include <ivp_constraint_local.hxx>
#endif
#ifndef IVP_CONTROLLER_MOTION_INCLUDED
#	include <ivp_controller_motion.hxx>
#endif


#ifndef IVP_CONSTRAINED_FIXED_KEYFRAMED_INCLUDED
#define IVP_CONSTRAINED_FIXED_KEYFRAMED_INCLUDED

class IVP_Template_Constraint_Fixed_Keyframed:public IVP_Template_Controller_Motion {
public:
    IVP_Template_Constraint_Fixed_Keyframed();
};

class IVP_Constraint_Fixed_Keyframed: public IVP_Controller_Dependent {
protected:
	// static values
    IVP_U_Float_Point max_translation_force; // defined in ws
    IVP_U_Float_Point max_torque;	// defined per core axis

    IVP_FLOAT force_factor;	// [ 0.. 1 ] 
    IVP_FLOAT damp_factor;	// [ 0..1 ] 

    IVP_FLOAT torque_factor;
    IVP_FLOAT angular_damp_factor;

	IVP_Environment *l_environment;
	// dynamic values
	IVP_Time time_of_prime_position;

	IVP_Time time_of_prime_orientation_0;
	IVP_FLOAT i_delta_prime_orientation_time;

	IVP_U_Quat prime_orientation_0;
	IVP_U_Quat prime_orientation_1;
	IVP_BOOL   angular_velocity_set;

	IVP_U_Point prime_position_Ros;
	IVP_U_Float_Point velocity_Ros;

	IVP_Real_Object *reference_obj;
	IVP_Real_Object *attached_obj;
    IVP_Vector_of_Cores_2 cores_of_constraint_system;

	void core_is_going_to_be_deleted_event(IVP_Core *);
    IVP_DOUBLE get_minimum_simulation_frequency();
    IVP_U_Vector<IVP_Core> *get_associated_controlled_cores() { return &cores_of_constraint_system; };
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_CONSTRAINTS; };
	void ensure_in_simulation();
	void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *);
public:
	IVP_Environment *get_environment(){ return l_environment; };
	void set_prime_position_Ros( const IVP_U_Point * position_Ros, const IVP_U_Float_Point *velocity_Ros, const IVP_Time &current_time );
	void set_prime_orientation_Ros( const IVP_U_Quat * orientation0_Ros, const IVP_Time & time0, const IVP_U_Quat * orientation1 = NULL, IVP_FLOAT dt = 1.0f );

	IVP_Constraint_Fixed_Keyframed(IVP_Real_Object *reference_object, IVP_Real_Object *attached_object,
					const IVP_Template_Constraint_Fixed_Keyframed *);
	virtual ~IVP_Constraint_Fixed_Keyframed();
};

#endif
