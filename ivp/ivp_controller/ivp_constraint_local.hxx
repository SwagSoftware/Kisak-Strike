// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_CONSTRAINT_LOCAL_INCLUDED
#define IVP_CONSTRAINT_LOCAL_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

// Constraints
// missing:
// global constraints
// force borders
// runtime-changeable constraints
// Version 0.6: local linear constraints with borders, maxforces and API
// added axis[] == mapping_userRfs_in_internRfs
// enhanced internal structure and algorithm
// Version 0.5: local linear constraints with borders and API
// removed constraintmanager
// enhanced API
// enhanced internal structure
// added possibility of providing only one object
// added initial difference between Rfs and Afs
// Version 0.4: local linear constraints with borders and API
// enhanced API
// enhanced internal structure and added rs
// added borders
// Version 0.3: local linear constraints with API
// added API
// organized internal structure and added Afs and fs
// Version 0.2: local linear constraints
// added most linear constraints
// Version 0.1: local linear ball-and-socket constraints


#ifndef IVP_CONSTRAINT_INCLUDED
#include <ivp_constraint.hxx>
#endif

#ifndef _IVP_U_MAPPING_INCLUDED
#include <ivu_mapping.hxx>
#endif

class IVP_Real_Object;
class IVP_Core;
class IVP_Template_Constraint_Anchor;

// The Anchor is stored in a certain coords system and it describes from its view the axles and the center (fs)
// vv of m_as_f_bs is the origin of as, stored in bs
class IVP_Constraint_Local_Anchor : public IVP_U_Matrix {
public:
    IVP_Real_Object *object; // remember object (not core) since cores may be split up in objects
    IVP_U_Matrix3   *rot;
    
    IVP_Constraint_Local_Anchor();
    ~IVP_Constraint_Local_Anchor();
};

class IVP_Constraint_Local_MaxImpulse {
    friend class IVP_Constraint_Local;
private:
    IVP_FLOAT halfimpulse [IVP_TR_INDEX_MAX];
    IVP_CONSTRAINT_FORCE_EXCEED type [IVP_TR_INDEX_MAX];
};

class IVP_Constraint_Local : public IVP_Constraint {
    friend class IVP_Controller;
    friend class IVP_Environment;
private: // describing variables
    IVP_FLOAT force_factor;  // factor to balance different simulation frequencies
    IVP_FLOAT damp_factor_div_force;  // dampening / force 
    
    IVP_CONSTRAINT_AXIS_TYPE fixed[IVP_TR_INDEX_MAX]; // fixed axles, maybe bitfield
    IVP_FLOAT borderleft_Rfs[IVP_TR_INDEX_MAX];
    IVP_FLOAT borderright_Rfs[IVP_TR_INDEX_MAX];
    IVP_FLOAT limited_axis_stiffness; // a stiffness factor ( 0..1 ) for all limited axis (default 0.3f)
    IVP_Constraint_Local_MaxImpulse *maxforce;
    //IVP_CONSTRAINT_FORCE_EXCEED maxforce_type, maxtorque_type;
    
    IVP_Constraint_Local_Anchor m_Rfs_f_Rcs; // matrizes, that descibe the relative rotation and translation systems
    IVP_Constraint_Local_Anchor m_Afs_f_Acs;
private: // useful variables for calculation

    IVP_U_Mapping mapping_uRfs_f_Rfs, mapping_uRrs_f_Rrs;
    unsigned char fixedtrans_dim, fixedrot_dim;
    unsigned char limitedtrans_dim, limitedrot_dim;
    unsigned char matrix_size; // number of (sometimes temporary) fixed axles
    IVP_NORM norm:8;
private: // functions
    //void set_orientation(const IVP_U_Matrix3 &m_relatedsystem_f_rotatedsystem, IVP_U_Point *orientation_out); // calculates orientation vector for one matrix

    IVP_Constraint_Local();
    void init(const IVP_Template_Constraint &tmpl);
    void sort_translation_mapping();
    void sort_rotation_mapping();
    void constraint_changed();
protected:
    void core_is_going_to_be_deleted_event(IVP_Core *core);
    void do_simulation_controller(IVP_Event_Sim *es,IVP_U_Vector<IVP_Core> *core_list);
    
public: // constraint changing functions
    IVP_Real_Object *get_objectR();
    IVP_Real_Object *get_objectA();

    // functions that refer to translation
    void change_fixing_point_Ros(const IVP_U_Point *anchor_Ros);
    void change_target_fixing_point_Ros(const IVP_U_Point *anchor_Ros);
    void change_translation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rfs);
    void change_target_translation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rfs);
    void fix_translation_axis(IVP_COORDINATE_INDEX which);
    void free_translation_axis(IVP_COORDINATE_INDEX which);
    void limit_translation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right);
    void change_max_translation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse);

    // functions that refer to rotation
    
    void change_rotation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rrs);   // set a new rotation axis in both object
    void change_target_rotation_axes_Ros(const IVP_U_Matrix3 *m_Ros_f_Rrs); // set a new angular target position of attached object
    void fix_rotation_axis(IVP_COORDINATE_INDEX which);
    void free_rotation_axis(IVP_COORDINATE_INDEX which);
    void limit_rotation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right);
    void change_max_rotation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse);

    // functions that explain differences
    void change_Aos_to_relaxe_constraint();
    void change_Ros_to_relaxe_constraint();

    IVP_Constraint_Local(const IVP_Template_Constraint &tmpl);
    ~IVP_Constraint_Local();
};
#endif
