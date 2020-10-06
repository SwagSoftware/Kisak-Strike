// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#if !defined( IVP_ACTUATOR_SPRING_INCLUDED )
#	define IVP_ACTUATOR_SPRING_INCLUDED

#ifndef WIN32
#	pragma interface
#endif



enum IVP_SPRING_FORCE_EXCEED {  // Do not use this enum yet.
    IVP_SFE_NONE =0 ,
    IVP_SFE_BREAK = 1      // destroy constraint if force is too great
    //IVP_SFE_CLIP,       // clip force 
    //IVP_CFE_BEND        // bends itself if force is too great -- not yet implemented
};

/********************************************************************************
 *	Name:	       	IVP_Template_Spring
 *	Description:	Values to IVP_Actuator_Spring, 
 *			a springs with a constant spring constant,
 *			useable for spring-mass simulations
 *	Attention:	Too high spring constants result in higher frequency than
 *			PSI rate and will cause high unnatural speeds of objects
 ********************************************************************************/

class IVP_Template_Spring: public IVP_Template_Two_Point {
public:
    IVP_FLOAT spring_len;		/* the len of the spring (resulting in no force),
				 * unit: meters
				 * hint: to fix in object in another objects space, use zero spring
				 * 	 len and rel_pos_damp */
    
    IVP_BOOL spring_values_are_relative;	/* set this to IVP_TRUE, if spring values should be multiplied
						 * with the average virtual mass of both objects */
    IVP_BOOL spring_force_only_on_stretch;	// set this to IVP_TRUE, if spring values should only be applied when the length exceeds spring_len

    IVP_FLOAT spring_constant;	/* in Newton/meter of (if spring_values_are_relative) Newton/(meter*virtual_mass)
				 * (used to create mass independent spring-mass systems) */
    IVP_FLOAT spring_damp;		// the spring damp constant
    IVP_FLOAT rel_pos_damp;		/* a damp factor which includes dampening of spring rotation. This factor is very usefull
				 * to decrease the number of springs */
    
    IVP_SPRING_FORCE_EXCEED max_len_exceed_type; 
    IVP_FLOAT break_max_len;	// causes a fire_event_spring_broken 

    IVP_U_Active_Float *active_float_spring_len;           // optional override of spring len
    IVP_U_Active_Float *active_float_spring_constant;      // dito
    IVP_U_Active_Float *active_float_spring_damp;          // dito
    IVP_U_Active_Float *active_float_spring_rel_pos_damp;  // dito
    IVP_Template_Spring();
};

/********************************************************************************
 *	Name:	       	IVP_Template_Suspension
 *	Description:	Values for IVP_Actuator_Suspension, 
 *			which is a special spring implementation
 *                      used for vehicle simulation.
 ********************************************************************************/

class IVP_Template_Suspension: public IVP_Template_Spring {
public:
    IVP_FLOAT spring_dampening_compression; // damp factor when spring is compressing
    IVP_FLOAT max_body_force; // maximum force that is exerted on body from the spring
                              // Value must be positive.
    IVP_Template_Suspension();
};


/********************************************************************************
 *	Name:	     		IVP_Listener_Spring_Event 	
 *	Description:		a callback to watch the status of a spring
 ********************************************************************************/
class IVP_Listener_Spring {
public:
    virtual void event_spring_broken(IVP_Actuator_Spring *spring)=0;
};


/********************************************************************************
 *	Name:	       	IVP_Actuator_Spring
 *	Description:	the standard spring used for spring-mass simulationns
 *	Attention:	Too high spring constant can result in higher frequency
 *			than the simulation can handle and therefore should
 *			be avoided
 ********************************************************************************/
class IVP_Actuator_Spring: public IVP_Actuator_Two_Point
{
    friend class IVP_Spring_Manager;
    IVP_Environment *l_environment;

protected:
    IVP_FLOAT spring_len;

    IVP_FLOAT spring_values_factor;        // the following four values are multiplied with this factor 
    IVP_FLOAT spring_constant;	           // already multiplied with factor
    IVP_FLOAT spring_damp;		   // the spring damp constant
    IVP_FLOAT rel_pos_damp;		   // A damp factor which includes dampening of spring rotation. This factor is useful
                                           // to minimize the number of springs
    IVP_FLOAT break_max_len;	           // when spring length exceeds this value, fire_event_spring_broken is called
    IVP_SPRING_FORCE_EXCEED max_len_exceed_type;
    IVP_BOOL spring_force_only_on_stretch;

    /** Listeners */
    IVP_U_Vector<IVP_Listener_Spring> listeners_spring;
    void fire_event_spring_broken();

    friend class IVP_Environment;
    
    IVP_Actuator_Spring(IVP_Environment *, IVP_Template_Spring *, IVP_ACTUATOR_TYPE);
 public:

    void set_constant(IVP_DOUBLE value); // will be multiplied with factor
    IVP_FLOAT get_constant(){ return spring_constant;};
    void set_damp(IVP_DOUBLE value);
    IVP_FLOAT get_damp_factor(){ return spring_damp;};
    void set_rel_pos_damp(IVP_DOUBLE value);
    IVP_FLOAT get_rel_pos_damp() { return rel_pos_damp; }
    void set_len(IVP_DOUBLE value);
    void set_break_max_len(IVP_DOUBLE value);

    IVP_FLOAT get_spring_length_zero_force(){ return spring_len; };

	IVP_BOOL get_only_stretch() { return spring_force_only_on_stretch; }
  
    void add_listener_spring(IVP_Listener_Spring *listener);	// NOT_IMPLEMENTED_YET
    void remove_listener_spring(IVP_Listener_Spring *listener);	// NOT_IMPLEMENTED_YET

    virtual void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
  
    virtual ~IVP_Actuator_Spring();
};

class IVP_Actuator_Spring_Active: public IVP_Actuator_Spring, IVP_U_Active_Float_Listener {

protected:

    IVP_U_Active_Float *active_float_spring_len;           // optional override of spring len
    IVP_U_Active_Float *active_float_spring_constant;      // dito
    IVP_U_Active_Float *active_float_spring_damp;          // dito
    IVP_U_Active_Float *active_float_spring_rel_pos_damp;  // dito
    
    void active_float_changed(IVP_U_Active_Float *af);

    friend class IVP_Environment;
    IVP_Actuator_Spring_Active(IVP_Environment *env, IVP_Template_Spring *spring); // constructor of the spring
 public:    
    virtual ~IVP_Actuator_Spring_Active();
};

/********************************************************************************
 *	Name:	       	IVP_Actuator_Suspension
 *	Description:	Special spring used for vehicle suspension. Derived from IVP_Actuator_Spring.
 ********************************************************************************/
class IVP_Actuator_Suspension: public IVP_Actuator_Spring {
protected:
    IVP_FLOAT spring_dampening_compression;    // In vehicles, the dampening is less when wheels are pushed upwards.
                                               // is multiplied by spring_values_factor
                                               // The inherited damp_factor is used for downward movement.
    IVP_FLOAT max_body_force;                  // You can restrict the force exerted at one of the objects
                                               // This is used to prevent the body from jumping too high on side walk borders 
public:    
    void set_spring_damp_compression(IVP_FLOAT value); // will be multiplied with factor
    void set_max_body_force(IVP_FLOAT value);
    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);  
    ~IVP_Actuator_Suspension();

    // To create a new suapension, use IVP_Environment->create_suspension
    IVP_Actuator_Suspension(IVP_Environment *env, IVP_Template_Suspension *templ);
};

#endif  /* if defined(IVP_ACTUATOR_SPRING_INCLUDED) */
