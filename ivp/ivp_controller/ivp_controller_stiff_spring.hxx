// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#if !defined( IVP_CONTROLLER_STIFF_SPRING_INCLUDED )
#	define IVP_CONTROLLER_STIFF_SPRING_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#if !defined(IVP_ACTUATOR_INCLUDED)
#	include <ivp_actuator.hxx>
#endif


/********************************************************************************
 *	Name:	       	IVP_Template_Stiff_Spring
 *	Description:	Values to IVP_Controller_Stiff_Spring, 
 *			a springs with a constant spring constant,
 *			useable for spring-mass simulations
 *	Note:		Works like a real spring, however
 *			- spring constant [0.001f .. 1]
 *			- does not suffer from integrator problems ->very stable
 *			- can be made stiffer than real springs
 *			- dampening [0 .. 1]
 *			It's a always a good idea to keep the spring_damp value in the same range
 *			as the spring_constant ( e.g. s_constant 0.1f -> s_dampening [0.05f .. 0.2f]
 *	Attention:	- avoid short spring lens (use ball socket constraints instead)
 *			(A short spring len is: if the differential speed of the
 *			connected objects is higher than spring_len * 10 /PSI_RATE:
 *			current settings:  speed [m/s] < spring_len * 6
 ********************************************************************************/

class IVP_Template_Stiff_Spring: public IVP_Template_Two_Point {
public:
    IVP_FLOAT spring_len;		/* the len of the spring (resulting in no force),
					 * unit: meters */
    
    IVP_FLOAT spring_constant;	// [ 0.. 1 ] 
    IVP_FLOAT spring_damp;	// [ 0.. 1 ]
    
    IVP_SPRING_FORCE_EXCEED max_len_exceed_type; 
    IVP_FLOAT break_max_len;	// causes a fire_event_spring_broken 


    IVP_Template_Stiff_Spring();
};


/********************************************************************************
 *	Name:	       	IVP_Template_Spring
 *	Description:	Values to IVP_Actuator_Spring, 
 *			a springs with a constant spring constant,
 *			useable for spring-mass simulations
 *	Attention:	Too high spring constants result in higher frequency than
 *			PSI rate and will cause high unnatural speeds of objects
 ********************************************************************************/
class IVP_Template_Stiff_Spring_Active: public IVP_Template_Stiff_Spring {
public:
    IVP_U_Active_Float *active_float_spring_len;           // optional override of spring len
    IVP_U_Active_Float *active_float_spring_constant;      // dito
    IVP_U_Active_Float *active_float_spring_damp;          // dito
    IVP_Template_Stiff_Spring_Active();
};

/********************************************************************************
 *	Name:	     		IVP_Listener_Spring_Event 	
 *	Description:		a callback to watch the status of a spring
 ********************************************************************************/
class IVP_Listener_Stiff_Spring {
public:
    virtual void event_stiff_spring_broken(class IVP_Controller_Stiff_Spring *)=0;
};


/********************************************************************************
 *	Name:	       	IVP_Actuator_Spring
 *	Description:	the standard spring used for spring-mass simulationns
 *	Attention:	Too high spring constant can result in higher frequency
 *			than the simulation can handle and therefore should
 *			be avoided
 ********************************************************************************/
class IVP_Controller_Stiff_Spring: public IVP_Actuator_Two_Point
{
    IVP_Environment *l_environment;
protected:
    IVP_FLOAT spring_len;

    IVP_FLOAT spring_constant;	           // already multiplied with factor
    IVP_FLOAT spring_damp;		   // the spring damp constant
    IVP_FLOAT break_max_len;	           // when spring length exceeds this value, fire_event_spring_broken is called
    IVP_SPRING_FORCE_EXCEED max_len_exceed_type; 
    
    /** Listeners */
    IVP_U_Vector<IVP_Listener_Stiff_Spring> listeners_spring_event;
    void fire_event_spring_broken();

    friend class IVP_Environment;
    
    virtual void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *);

    virtual IVP_CONTROLLER_PRIORITY get_controller_priority();
 public:
    void set_constant(IVP_DOUBLE value); // will be multiplied with factor
    IVP_FLOAT get_constant(){ return spring_constant;};
    void set_damp(IVP_DOUBLE value);
    IVP_FLOAT get_damp_factor(){ return spring_damp;};

    void set_len(IVP_DOUBLE value);
    void set_break_max_len(IVP_DOUBLE value);

    IVP_FLOAT get_spring_length_zero_force(){ return spring_len; };
  
    void add_listener_stiff_spring(IVP_Listener_Stiff_Spring *listener);	// NOT_IMPLEMENTED_YET
    void remove_listener_stiff_spring(IVP_Listener_Stiff_Spring *listener);	// NOT_IMPLEMENTED_YET

    IVP_Controller_Stiff_Spring(IVP_Environment *,IVP_Template_Stiff_Spring *);
    virtual ~IVP_Controller_Stiff_Spring();
};



class IVP_Controller_Stiff_Spring_Active: public IVP_Controller_Stiff_Spring, IVP_U_Active_Float_Listener {
protected:

    IVP_U_Active_Float *active_float_spring_len;           // optional override of spring len
    IVP_U_Active_Float *active_float_spring_constant;      // dito
    IVP_U_Active_Float *active_float_spring_damp;          // dito
    
    void active_float_changed(IVP_U_Active_Float *af);

    friend class IVP_Environment;
 public:    
    IVP_Controller_Stiff_Spring_Active(IVP_Environment *env, IVP_Template_Stiff_Spring_Active *spring); // constructor of the spring
    virtual ~IVP_Controller_Stiff_Spring_Active();
};

#endif /* ifdef IVP_CONTROLLER_STIFF_SPRING_INCLUDED */
