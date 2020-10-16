// Copyright (C) 2000 Ipion Software GmbH. All rights reserved.

// IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_CONTROLLER_INCLUDED
#	include <ivp_controller.hxx>
#endif

class IVP_Template_Controller_World_Friction {
public:
    IVP_U_Point desired_speed_ws; //desired speed greater zero could be used on e.g. moving platforms
    IVP_U_Point desired_rot_speed_cs;
    IVP_U_Point friction_value_translation; //one value per axis
    IVP_U_Point friction_value_rotation; //one value per axis
    
    //warning: these friction values are different to friction values of normal objects.
    //         normally a friction value is the tan( alpha ) , were alpha is degree of steepest
    //         angle were object does not slide
    //         here the friction value is the maximum speed (m/s) that is allowed to be reduced per second
    //         9.81 m/s means that gravity is totally neutralized
        
    IVP_Template_Controller_World_Friction();
};


class IVP_Controller_World_Friction : public IVP_Controller_Independent {
protected:
    IVP_Real_Object *real_obj;

    void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list);
    
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_CONSTRAINTS_MAX; };
    void core_is_going_to_be_deleted_event(IVP_Core *core);

    friend class IVP_Environment;    
    
    // --------------------------------------------------------
    // The following settings may be changed in the mainloop e.g. using the following public functions
    // --------------------------------------------------------
    IVP_U_Float_Point desired_speed_ws; 
    IVP_U_Float_Point desired_rot_speed_cs;
    IVP_U_Float_Point friction_value_translation; //one value per axis
    IVP_U_Float_Point friction_value_rotation; //one value per axis
    IVP_BOOL clip_manhattan;
    
public:
    void set_desired_speed_ws( IVP_U_Float_Point *speed_ws ) { desired_speed_ws.set( speed_ws ); };
    void set_desired_rot_speed_cs( IVP_U_Float_Point *rot_speed_cs ) { desired_rot_speed_cs.set( rot_speed_cs ); };
    const IVP_U_Float_Point *get_desired_speed_ws()const { return &desired_speed_ws; };
    const IVP_U_Float_Point *get_desired_rot_speed_cs()const { return &desired_rot_speed_cs; };

    void set_friction_value_translation( IVP_U_Point value_trans ) { friction_value_translation.set(&value_trans); };  
    void set_friction_value_rotation( IVP_U_Point value_rot ) { friction_value_rotation.set(&value_rot); };  
    const IVP_U_Float_Point *get_friction_value_rotation() { return &friction_value_rotation; };
    const IVP_U_Float_Point *get_friction_value_translation() { return &friction_value_translation; };
    
    // --------------------------------------------------------
    // Special Methods
    // --------------------------------------------------------
    IVP_Controller_World_Friction(IVP_Real_Object *obj, const IVP_Template_Controller_World_Friction *templ);
    virtual ~IVP_Controller_World_Friction();
};
