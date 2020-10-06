// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_CONTROLLER_INCLUDED
#	include <ivp_controller.hxx>
#endif

#ifndef IVP_CONTROLLER_MOTION_INCLUDED
#define IVP_CONTROLLER_MOTION_INCLUDED

class IVP_Template_Controller_Motion {
public:
    // --------------------------------------------------------
    // The function controlling the behaviour of the controller
    // if force_factor > damp_factor you get a springy behaviour
    // if force_factor < damp_factor ->smooth movement
    // --------------------------------------------------------
    IVP_FLOAT force_factor;	// force factor   [ 0.. 1.0 .. 1.5f ]     default: 0.8
    IVP_FLOAT damp_factor;	// damp:   [ 0 ...1.0f ..1.5f ]		default: 1.2

    IVP_FLOAT torque_factor;		// torque factor   [ 0.. 1.0 .. 1.5f ]     default: 0.8
    IVP_FLOAT angular_damp_factor;	// damp:   [ 0 ...1.0f ..1.5f ]		default: 1.2

    IVP_U_Float_Point max_translation_force;
    IVP_FLOAT max_torque;

    IVP_Template_Controller_Motion();
};



class IVP_Controller_Motion : public IVP_Controller_Independent
{
protected:
    // --------------------------------------------------------
    // The following settings may be changed in the mainloop using the
    // public functions
    // --------------------------------------------------------
    IVP_U_Point target_pos_ws;         // position of core to reach in next PSI
    IVP_U_Quat target_q_world_f_core;  // orientation to reach in next PSI

    IVP_U_Float_Point max_translation_force; // defined in ws
    IVP_U_Float_Point max_torque;	// defined per core axis

    IVP_FLOAT force_factor;	// [ 0.. 1 ] 
    IVP_FLOAT damp_factor;	// [ 0..1 ] 

    IVP_FLOAT torque_factor;
    IVP_FLOAT angular_damp_factor;
protected:
    IVP_Environment *l_environment;
    IVP_Real_Object *real_object;
    IVP_Core *core;

    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *);
    
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_MOTION; };
    void core_is_going_to_be_deleted_event(IVP_Core *core);

    friend class IVP_Environment;
public:

	/********************************************************************************
	*   get values
	********************************************************************************/
    const IVP_U_Float_Point *get_max_torque() const { return &max_torque; };
    const IVP_U_Float_Point *get_max_translation_force() const { return &max_translation_force; };
	IVP_FLOAT get_force_factor() const { return force_factor; };
	IVP_FLOAT get_damp_factor() const { return damp_factor; };
	IVP_FLOAT get_torque_factor() const { return torque_factor; };
	IVP_FLOAT get_angular_damp_factor() const { return angular_damp_factor; };
    const IVP_U_Point *get_target_position_ws() const { return &target_pos_ws; };
	const IVP_U_Quat *get_target_orientation() const { return &target_q_world_f_core; }; 

	/********************************************************************************
	*   change some values on the fly
	********************************************************************************/
    void set_max_torque(const IVP_U_Float_Point *max_t);
    void set_max_translation_force(const IVP_U_Float_Point *max_tf);
	void set_force_factor(IVP_FLOAT new_val) { force_factor=new_val; };
	void set_damp_factor(IVP_FLOAT new_val) { damp_factor=new_val; };
	void set_torque_factor(IVP_FLOAT new_val) { torque_factor=new_val; };
	void set_angular_damp_factor(IVP_FLOAT new_val) { angular_damp_factor=new_val; };

    /********************************************************************************
     *	Name:	       	set_target_position_ws
     *	Description:	set the desired position of the mass center 
     *	Note:		very fast
     ********************************************************************************/
    void set_target_position_ws(const IVP_U_Point * position_of_mass);

    /********************************************************************************
     *	Name:	       	set_target_object_position_ws
     *	Description:	set the desired position of the object center
     *  Note:		significant slower than set_target_position_ws, so
     *			if your mass center is equal to the objects space origin.
     *			use set_target_position_ws
     ********************************************************************************/
    void set_target_object_position_ws(IVP_Real_Object *obj, const IVP_U_Quat * desired_orientation,const IVP_U_Point *position_of_object_space_center_ws);


    void set_target_q_world_f_core(const IVP_U_Quat * orientation);

    // --------------------------------------------------------
    // Methods
    // --------------------------------------------------------
    IVP_Controller_Motion(IVP_Real_Object *object, const IVP_Template_Controller_Motion *templ);
    virtual ~IVP_Controller_Motion();
};


#endif

