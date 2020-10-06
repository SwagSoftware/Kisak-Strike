// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_CONTROLLER_INCLUDED
#define IVP_CONTROLLER_INCLUDED


#ifndef WIN32
#	pragma interface
#endif

/**********************************************************************************************
 *	Name:	  	IVP_CONTROLLER_PRIORITY   	
 *    controllers give forces to objects (e.g. gravity, friction)
 *    they are sorted, the controller with the highest priority manipulates the object first
 *
 *	Note:		Only values between >IVP_CP_STATIC_FRICTION, and <IVP_CP_ENERGY_FRICTION
 * 			are allowed (Note: >= and <= is not allowed)
 *
 *			All controllers with priority > IVP_CP_GRAVITY must do async pushes
 *			all controllers with priority < IVP_CP_GRAVITY do real pushes
 *
 *			The order of controllers with priority > IVP_CP_GRAVITY is not important
 **********************************************************************************************/										 
enum IVP_CONTROLLER_PRIORITY {
	IVP_CP_NONE				= -1,

	IVP_CP_STATIC_FRICTION  = 0,    //this must always be of least priority


    IVP_CP_CONSTRAINTS_MIN  = 400,
    IVP_CP_CONSTRAINTS      = 405,
    IVP_CP_CONSTRAINTS_MAX  = 410,

    IVP_CP_STIFF_SPRINGS    = 460,
    IVP_CP_FLOATING         = 470,
    
    IVP_CP_MOTION           = 500,
    
    IVP_CP_DYNAMIC_FRICTION = 600,
    
    IVP_CP_GRAVITY          = 1000,
    
    IVP_CP_SPRING           = 1400,
    IVP_CP_ACTUATOR         = 1500,
    IVP_CP_FORCEFIELDS      = 1600,
    
    IVP_CP_ENERGY_FRICTION  = 2000 // this must be of highest priority
};

class IVP_Core;


class IVP_Event_Sim {
public:
    IVP_DOUBLE delta_time;
    IVP_DOUBLE i_delta_time;
    IVP_Environment *environment;
    class IVP_Simulation_Unit *sim_unit;
    
    IVP_Event_Sim(IVP_Environment *env, IVP_DOUBLE dtime){
	    environment = env; delta_time = dtime;if (dtime > P_FLOAT_EPS){
	    i_delta_time = 1.0f / dtime;
	}else{
	    i_delta_time = 1.0f/P_FLOAT_EPS;
	}
    }

    IVP_Event_Sim(IVP_Environment *env){
	environment = env; delta_time = env->get_delta_PSI_time(); i_delta_time = env->get_inv_delta_PSI_time();
    }
};

/********************************************************************************
 *	Name:	  	IVP_Controller   	
 *	Description:	super-class for physical controllers
 *                      Implement all of it's functions to controll one object
 ********************************************************************************/
class IVP_Controller {
public:
    /********************************************************************************
     *	Name:	  	core_is_going_to_be_deleted_event   	
     *	Description:	Callback, called by IVP_Core::~IVP_CORE()
     *  Note:		Controller may delete itself
     ********************************************************************************/
    virtual void core_is_going_to_be_deleted_event(IVP_Core *) {; };
    
    /********************************************************************************
     *	Name:	  	get_minimum_simulation_frequency   	
     *	Description:	Returns the minimum simulation frequency of this controller
     *			The result of this function is heavily cached, so changing
     *			it's return value might not have any effect
     ********************************************************************************/
    virtual IVP_DOUBLE get_minimum_simulation_frequency() { return 1.0f; };
    
    /********************************************************************************
     *	Name:	  	get_associated_controlled_cores   	
     *	Description:	Returns the cores that are treated this controller
     *  Note:           This vector is used only temporarily, so you may return a pointer
     *                  to a thread global vector.
     *	Note:           Must return only movable cores
     *  Version Info:   In future releases IVP_U_Vector might be replaced by IVP_U_Set.
     ********************************************************************************/
    virtual IVP_U_Vector<IVP_Core> *get_associated_controlled_cores() = 0;
  
    /********************************************************************************
     *	Name:	  	reset_time   	
     *	Description:	called when the time of the whole physics system is resetted
     * 			offset is old last_PSI_time
     ********************************************************************************/
    virtual void reset_time(IVP_Time /*offset*/){;};

    /********************************************************************************
     *	Name:	  	do_simulation_controller   	
     *	Description:	Perform the simulation, apply the forces or change the speed, rot_speed
     ********************************************************************************/
    virtual void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list)=0;
    
    /********************************************************************************
     *	Name:	  	get_controller_priority   	
     *	Description:	Get the priority of the controller, see IVP_CONTROLLER_PRIORITY
     ********************************************************************************/
    virtual IVP_CONTROLLER_PRIORITY get_controller_priority() = 0;

    virtual ~IVP_Controller() { ; };

    //lwss add
    virtual const char *get_controller_name()
    {
        return "CONTROLLER_UNNAMED";
    }
    //lwss end
};

class IVP_Controller_Independent: public IVP_Controller {
    static IVP_U_Vector<IVP_Core> empty_list;
    IVP_U_Vector<IVP_Core> *get_associated_controlled_cores() { return &empty_list; };
};

class IVP_Controller_Dependent: public IVP_Controller {
    
};


class IVP_Standard_Gravity_Controller : public IVP_Controller_Independent {
public:
    IVP_U_Float_Point grav_vec;

    void set_standard_gravity(IVP_U_Point *new_gravity);

    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_GRAVITY; };
    virtual ~IVP_Standard_Gravity_Controller() { ; };
    void core_is_going_to_be_deleted_event(IVP_Core *) { ; }
};



/********************************************************************************
 *	Name:	  	IVP_Controller_Manager   	
 *	Description:	Toolkit to manage controllers,
 *	Note:		To get an instance of this class call IVP_Environment::get_controller_manager()
 ********************************************************************************/
class IVP_Controller_Manager {
public:
    IVP_Environment *l_environment;
    
    /********************************************************************************
     *	Name:	  	announce_controller_to_environment   	
     *	Description:	inform the system that a new controller is now controlling
     *			some cores
     ********************************************************************************/
    void announce_controller_to_environment(IVP_Controller_Dependent *cntrl);

    /********************************************************************************
     *	Name:	  	add_controller_to_core   	
     *	Description:	inform the system that a new controller is now controlling
     *			an additional single core
     ********************************************************************************/
    static void add_controller_to_core(IVP_Controller_Independent *cntrl, IVP_Core *core);

    
    /********************************************************************************
     *	Name:	  	remove_controller_from_environment   	
     *	Description:	remove controller from all cores
     ********************************************************************************/
    static void remove_controller_from_environment(IVP_Controller_Dependent *cntrl,IVP_BOOL silently);


    /********************************************************************************
     *	Name:	  	remove_controller_from_core   	
     *	Description:	remove single controller from one cores
     ********************************************************************************/
    static void remove_controller_from_core(IVP_Controller_Independent *cntrl, IVP_Core *core);
    
    
    /********************************************************************************
     *	Name:	  	ensure_controller_in_simulation   	
     *	Description:	forces all objects to wake up at next PSI
     ********************************************************************************/
    void ensure_controller_in_simulation(IVP_Controller_Dependent *cntrl);

    /********************************************************************************
     *	Name:	  	ensure_controller_in_simulation   	
     *	Description:	forces one core to wake up at next PSI
     ********************************************************************************/
    void ensure_core_in_simulation(IVP_Core *core);

    IVP_Controller_Manager(IVP_Environment *env) { l_environment=env; };
};

#endif
