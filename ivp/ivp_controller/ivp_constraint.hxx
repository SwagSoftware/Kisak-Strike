// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_CONSTRAINT_INCLUDED
#define IVP_CONSTRAINT_INCLUDED

#ifndef IVP_CONTROLLER_INCLUDED
#include <ivp_controller.hxx>
#endif

#include "ivp_constraint_types.hxx"

class IVP_Real_Object;
class IVP_Core;
class IVP_Environment;

enum IVP_TRANSROT_INDEX {
    IVP_TR_INDEX_TX = 0,
    IVP_TR_INDEX_TY = 1,
    IVP_TR_INDEX_TZ = 2,
    IVP_TR_INDEX_RX = 3,
    IVP_TR_INDEX_RY = 4,
    IVP_TR_INDEX_RZ = 5,
    IVP_TR_INDEX_MAX = 6
};

class IVP_Constraint : public IVP_Controller_Dependent {
protected:
    IVP_BOOL is_enabled:2;
public:
    IVP_Constraint();

    /******************************************************************************************
     *  Feature:        deactivate()
     *  Description:    You can deactivate this constraint temporarily if you call this function.
     *                  The constraint is removed from simulation until you activate() it again.
	 *  Attention:		If a constraint is deactivated, it will not get deleted when you delete the 
	 *					environment
     *****************************************************************************************/
    void deactivate();
    void activate();

    /******************************************************************************************
     *  Feature:        Destruktor
     *  Description:    You can delete a constraint whenever you want to permanently remove
     *                  it from simulation.
     *****************************************************************************************/
    virtual ~IVP_Constraint();

    /******************************************************************************************
     *  Feature:        Constraint changing functions
     *  Description:    You can change this constraint's parameter by using these functions.
     *                  The constraint changing functions are used exactly the same way as the
     *                  constraint setting functions in class IVP_Template_Constraint.
     *                  Look there for a complete description.
     *  Note:           Constraint changing functions are rather expensive. Try not to use them
     *                  some thousand times per second. But creating a new constraint is much
     *                  more expensive than changing one parameter of an existing one.
     *			Spaces:
     *			R Reference Object
     *			A Attached Object
     *			os object system
     *			fs system for translation
     *			rs system for rotation (may be equal to fs)
     *
     *  Attention:      The constraint changing functions are not yet fully tested.
     *****************************************************************************************/

    // constraint changing functions that refer to translation
    virtual void change_fixing_point_Ros(const IVP_U_Point *anchor_Ros);		    // just set a new fixing point at both objects
    virtual void change_target_fixing_point_Ros(const IVP_U_Point *anchor_Ros);
    virtual void change_translation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rfs);   // just changes the translation axes. Normally has no effect
    virtual void change_target_translation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rfs); // move attached object to a new postion
    virtual void fix_translation_axis(IVP_COORDINATE_INDEX which);
    virtual void free_translation_axis(IVP_COORDINATE_INDEX which);
    virtual void limit_translation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right);
    virtual void change_max_translation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse);

    // constraint changing functions that refer to rotation
    virtual void change_rotation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rrs);
    virtual void change_target_rotation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rrs);
    
    virtual void fix_rotation_axis(IVP_COORDINATE_INDEX which);
    virtual void free_rotation_axis(IVP_COORDINATE_INDEX which);
    virtual void limit_rotation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right);
    virtual void change_max_rotation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse);

    // constraint changing functions that explain differences
    virtual void change_Aos_to_relaxe_constraint();
    virtual void change_Ros_to_relaxe_constraint();
    
    // some functions are missing yet.
public:
    // functions to get the values of the constraint are missing (should be feedable to changing values)

    // change_angular_limitation(IVP_INDEX_X, alpha, alpha);
    // get_angular_shift_rs(IVP_U_Point *p);
protected:
    IVP_Environment *get_environment();    
protected:
    IVP_Vector_of_Cores_2 cores_of_constraint_system;

    void core_is_going_to_be_deleted_event(IVP_Core *core);

    IVP_DOUBLE get_minimum_simulation_frequency();
    IVP_CONTROLLER_PRIORITY get_controller_priority() { return IVP_CP_CONSTRAINTS; };
public:
    IVP_U_Vector<IVP_Core> *get_associated_controlled_cores() { return &cores_of_constraint_system; };
    static IVP_Constraint *create_constraint_any_solver(IVP_Template_Constraint *constraint_template);

public:
    void *client_data; //not used by physics, provided for customer
};

#endif
