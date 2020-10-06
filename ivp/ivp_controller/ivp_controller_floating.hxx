// Copyright (C) 2000 Ipion Software GmbH. All rights reserved.

// IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_CONTROLLER_INCLUDED
#	include <ivp_controller.hxx>
#endif

class IVP_Template_Controller_Floating 
{
public:

    IVP_Template_Controller_Floating();

    void set_position_ws( IVP_Real_Object *obj, const IVP_U_Point *pos_ws );
    void set_ray_direction_ws( IVP_Real_Object *obj, const IVP_U_Point *dir_ws );

public:

    IVP_FLOAT	max_repulsive_force;
    IVP_FLOAT	max_adhesive_force;
    IVP_U_Point position_os;					// position where force is exerted (and current_distance is measured from)
    IVP_U_Point ray_direction_ws;				// is also force direction, must be normized!
    IVP_FLOAT	target_distance;				// distance to reach in next PSI must be >> 0.0f!
    IVP_FLOAT	current_distance;				// initial value, later to be fed from outside (e.g. from ray caster)
};


class IVP_Controller_Floating : public IVP_Controller_Independent 
{
public:

    virtual IVP_RETURN_TYPE do_ray_casting(IVP_Event_Sim *) = 0;

public:

    IVP_Controller_Floating(IVP_Real_Object *obj, const IVP_Template_Controller_Floating *templ);
    virtual ~IVP_Controller_Floating();

    void set_current_distance(IVP_DOUBLE cd);
    IVP_DOUBLE get_current_distance(){ return current_distance; };
    
    const IVP_U_Float_Point *get_position_os()const { return &position_os; };
    
    void set_target_distance(IVP_DOUBLE td);
    IVP_DOUBLE get_target_distance(){ return target_distance; };

    void set_ray_direction_ws( const IVP_U_Float_Point *new_dir );
    const IVP_U_Float_Point *get_ray_direction_ws();
    
protected:

    void do_simulation_controller( IVP_Event_Sim *, IVP_U_Vector<IVP_Core> *core_list );
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_FLOATING; };
    void core_is_going_to_be_deleted_event( IVP_Core *core );

    friend class IVP_Environment;    

    IVP_Real_Object		*object;    

    // --------------------------------------------------------
    // The following settings may be changed in the mainloop e.g. using the following public functions
    // --------------------------------------------------------
    IVP_FLOAT			max_repulsive_force;
    IVP_FLOAT			max_adhesive_force;

    IVP_U_Float_Point	position_os;			// position where force is exerted (and distances are measured from)
    
    IVP_U_Float_Point	ray_direction_ws;		// (is also force direction)
    IVP_DOUBLE			target_distance;		// distance to reach in next PSI
    IVP_DOUBLE			current_distance;		// to be fed from outside (e.g. from ray caster)
};
