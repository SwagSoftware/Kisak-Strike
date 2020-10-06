// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_CONTROLLER_MOTION_INCLUDED
#	include <ivp_controller_motion.hxx>
#endif


#ifndef IVP_CONTROLLER_GOLEM_INCLUDED
#define IVP_CONTROLLER_GOLEM_INCLUDED


enum IVP_GOLEM_PROBLEM {
	IVP_GP_FAR_DISTANCE,
        IVP_GP_BIG_ANGLE,
	IVP_GP_TIME_BY_DISTANCE_TOO_LONG
};

class IVP_Template_Controller_Golem:public IVP_Template_Controller_Motion {
public:
	IVP_FLOAT max_delta_position;
	IVP_FLOAT max_delta_orientation;
	IVP_FLOAT max_integrated_delta_position; //not used at the moment
	IVP_FLOAT max_golem_force;               //not used at the moment
	IVP_FLOAT filter_dtime;                  //not used at the moment

	IVP_Template_Controller_Golem();
};

class IVP_Controller_Golem : public IVP_Controller_Motion {
protected:
	// dynamic values
	IVP_FLOAT integrated_delta_position;    
	IVP_Time time_of_prime_position;

	IVP_Time time_of_prime_orientation_0;
	IVP_FLOAT i_delta_prime_orientation_time;
	IVP_BOOL   angular_velocity_set;

	IVP_U_Quat prime_orientation_0;
	IVP_U_Quat prime_orientation_1;

	IVP_U_Point prime_position_ws;
	IVP_U_Float_Point velocity_ws;

protected:
	// static values
	IVP_FLOAT max_delta_position;
	IVP_FLOAT max_delta_orientation;
	IVP_FLOAT max_integrated_delta_position;
	IVP_FLOAT max_golem_force;
	IVP_FLOAT filter_dtime;

    void reset_time( IVP_Time offset);
    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *);

    void beam_object_to_target_position(IVP_Event_Sim *es);  // may be called from resolve_for_problem to beam object
public:
	virtual IVP_RETURN_TYPE resolve_for_problem( IVP_Event_Sim *event_sim, IVP_GOLEM_PROBLEM problem ) = 0;


	void set_prime_position( const IVP_U_Point * position, const IVP_U_Float_Point * velocity , const IVP_Time & time );
	void set_prime_orientation( const IVP_U_Quat * orientation0, const IVP_Time & time0,const IVP_U_Quat * orientation1 = NULL, IVP_FLOAT dt = 0.0f);
	IVP_Controller_Golem(IVP_Real_Object *, const IVP_Template_Controller_Golem *);
	virtual ~IVP_Controller_Golem();
};

#endif
