// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif

/********************************************************************************
 *	Name:	       	ivp_object_attach.hxx
 *	Description:	simple functions that allow to attach an object to another
 *			the merged entity will use the physical values of the 
 *			parent object (mass, rot_inertia, speed, rot_speed ....)
 ********************************************************************************/

class IVP_Object_Attach {

public:
    /********************************************************************************
    *	Name:	       	attach_object
    *	Parameter:	max_distance_attached_object_to_parent is used for the collision 
    *			detection engine and should be set to a reasonable value (not e.g. 1e6)
    *			the smaller this value is set, the faster the collision detection
    ********************************************************************************/
    static void attach_object( IVP_Real_Object *parent, IVP_Real_Object *attached_object, IVP_DOUBLE max_distance_attached_object_to_parents_mass_center = -1.0f );

    /********************************************************************************
    *	Name:	       	detach_object
    *	Description:	creates a new physical instance for a attached object
    *	Note:		maybe used to change the physical parameters for an object
    *			as it is allowed to even call this function when the object is not
    *			attached to a parent
    *	Parameter	tmpl:	the physical parameters for this object
    ********************************************************************************/
    static void detach_object( IVP_Real_Object *attached_object, IVP_Template_Real_Object *tmpl );
    
    /********************************************************************************
    *	Name:	       	reposition_object_Ros
    *	Parameter:	q_Ros_f_Aos is a quat that transforms from Attached object space
    *			into Reference object space == matrix defining the attached object in Ros space
    *	Parameter:      check_before_moving if set movement will be limited if there
    *			might be penetration problems
    ********************************************************************************/
    static IVP_RETURN_TYPE reposition_object_Ros( IVP_Real_Object *parent,  // optional parent, if NULL than Ros == ws
						IVP_Real_Object *attached_object,
						const IVP_U_Quat *q_Ros_f_Aos,
						const IVP_U_Point *shift_Ros_f_Aos,
						IVP_BOOL check_before_moving);
};
