// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// Do not misuse this code for bad purpose.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_CONSTRAINT_TEMPLATE_INCLUDED
#define _IVP_CONSTRAINT_TEMPLATE_INCLUDED

#include "ivp_constraint_types.hxx"

class IVP_Real_Object;

/********************************************************************************
 *  Name:           IVP_Template_Constraint
 *  Description:    an interface structure for constraints
 *                  A constraint limits one (attached) object relative to another
 *		    (reference) object due to possible movement.
 *  Attention:      The "Attention:" sections of ivp_templates.hxx applies to this
 *		    template, too.
 *******************************************************************************/

class IVP_Template_Constraint {
public:
    friend class IVP_Environment;
public:
    /********************************************************************************
     *  Attention:      You may change the public variables in this class.
     *                  It is recommended to use the functions though.
     *  Note:           Like everywhere, the matrices have to be right-hand matrices.
     *******************************************************************************/

    IVP_CONSTRAINT_FLAGS flags; // not yet used

    /*******************************************************************************
     *  Name:           objectR   ("Reference object")
     *  Description:    The constraint's coordinate space definition is relative to
     *			the reference object's coordinate space.
     *                  When the objectR moves, the coordinate system of the 
     *			constraint moves accordingly.
     *  Default value:  NULL  (means: the reference space is world space)
     ******************************************************************************/
    IVP_Real_Object *objectR; // reference object
    void set_reference_object(IVP_Real_Object *objR);

    /*******************************************************************************
     *  Feature:        m_Ros_f_Rfs
     *  Description:    This matrix defines the translation axes of the constraint in reference object space
     *  Note:           If the rotation axes are not given, this matrix also defines the rotation axes.
     *                  The first column describes the x translation axis of the constraint.
     *  Note:           You can simply set the m_Ros_f_Rfs pointer to any applicable matrix.
     *  Note:           Or, you can use the buffer mm_Ros_f_Rfs and set: m_Ros_f_Rfs = &mm_Ros_f_Rfs;
     *  Default value:  NULL  (means: the constraint translation coordinate system is set to the reference object space)
     ******************************************************************************/
    IVP_U_Matrix  mm_Ros_f_Rfs;   // internal buffer
    IVP_U_Matrix  *m_Ros_f_Rfs;   // if NULL than Ros == Rfs
    
    void set_fixing_point_Ros    (const IVP_U_Point *anchor); // set anchor point, given in reference object space.
    void set_translation_axes_Ros(const IVP_U_Matrix3 *trans_axes);  // set direction of axle
    void set_translation_axes_as_object_space(); // this sets m_Ros_f_Rfs to NULL

    /******************************************************************************
     *  Feature:        m_Ros_f_Rrs
     *  Description:    This matrix defines the rotation axes of the constraint in reference object space.
     *  Note:           You can set the m_Ros_f_Rrs pointer to any applicable matrix.
     *  Note:           Or, you can use the buffer mm_Ros_f_Rfs and set: m_Ros_f_Rrs = &mm_Ros_f_Rrs;
     *  Default value:  NULL  (means: the rotation axes are equal to the translation axes)
     *  Note:           Setting m_Ros_f_Rrs to NULL results in faster constraints.
     *****************************************************************************/
    IVP_U_Matrix3 mm_Ros_f_Rrs;  // internal buffer
    IVP_U_Matrix3*m_Ros_f_Rrs;   // NULL means: Rrs == Rfs
    void set_rotation_axes_Ros(const IVP_U_Matrix3 *rot_axes);
    void set_rotation_axes_as_translation_axes(); // this sets m_Ros_f_Rrs to NULL

    /******************************************************************************
     *  Name:           objectA  ("Attached object")
     *  Description:    This is the object that is attached to the Reference object by the constraint.
     *                  When objectR and objectA move, the constraint axes move with objectR - not with objectA.
     *  Default value:  NULL  (means: an imaginary "world object"; its object coordinate system is world coordinate space)
     *	Note:		If the attached object is NULL (i.e. is the world space),
     *			moving the Reference object won't be possible.
     *****************************************************************************/
    IVP_Real_Object *objectA;
    void set_attached_object(IVP_Real_Object *objA);

    /******************************************************************************
     *  Feature:        m_Aos_f_Afs
     *  Description:    This matrix defines where the translation axes of the constraint should be in a relaxed constraint,
     *                  seen from the object system of the *Attached* object.
     *                  You will need this variable only if you are generating a constraint that is *not* relaxed.
     *                  The first time the constraint is simulated, it will give the objects a big push that moves them into a relaxed position.
     *  Note:           There is no need to set any "m_Aos_f_Ars" (rotation axes) since
     *                  the constraint calculates from m_Aos_f_Afs and m_Fos_f_Frs where the rotation axes should be.
     *  Default value:  NULL  (this means that the constraint is relaxed.)
     *****************************************************************************/
    IVP_U_Matrix  mm_Aos_f_Afs, *m_Aos_f_Afs;   // NULL means: Afs == Rfs
    void set_attached_fixing_point_Aos(const IVP_U_Point *trans_attached); // where the anchor should be on the attached object, seen in attached object system.
    void set_attached_translation_axes_Aos(const IVP_U_Matrix3 *rot_attached);
    void set_constraint_is_relaxed(); // this sets m_Aos_f_Afs to NULL (default)

    // --------------------------------------------------------
    // The function controlling the behaviour of the controller
    // if force_factor > damp_factor you get a springy behaviour
    // if force_factor < damp_factor ->smooth movement
    // --------------------------------------------------------
    IVP_FLOAT force_factor;	// force factor   [ 0.. 1.0 .. 1.5f ]     default: 1.0
    IVP_FLOAT damp_factor;	// damp:   [ 0 ...1.0f ..1.5f ]		default: 1.0


    /******************************************************************************
     *  Feature:        limited_axis_stiffness
     *  Description:    The same as relaxation_rate_per_dtime, but defined for all limited axis only
     *  Note		Small values should be used for limited constraints to remove
     *                  the effect of jumping constraints
     *****************************************************************************/
    IVP_FLOAT limited_axis_stiffness;
    
    /******************************************************************************
     *  Feature:        axis_type[6]
     *  Description:    You can define here what type of constraint you want to have.
     *                  axis_type[0] is the X translation axis.
     *                  axis_type[1] is the Y translation axis.
     *                  axis_type[2] is the Z translation axis.
     *                  axis_type[3] is the X rotation axis.
     *                  axis_type[4] is the Y rotation axis.
     *                  axis_type[5] is the Z rotation axis.
     *                  You can set any of these to any of the following values:
     *                  IVP_CONSTRAINT_FIXED   --  the objects must not move in this direction.
     *                  IVP_CONSTRAINT_FREE    --  the objects should be able to move freely in this direction.
     *                  IVP_CONSTRAINT_LIMITED --  the objects are allowed to move freely only within a given limitation.
     *                                             Set borderleft_Rfs and borderright_Rfs accordingly to these borders.
     *  Note:           If you set borders, always set them so that the left border value is smaller than the right border value.
     *  Note:           To determine what is "left" and "right" in terms of rotation, look into the direction of the axis.
     *                  Clockwise is right, anti-clockwise is left.
     *  Note:           Try to set up constraints in a way that borderleft < 0 < borderright and that
     *                  borderright has about the same value as -borderleft. You will get more accurate constraints.
     *  Default value:  {IVP_CONSTRAINT_FIXED, IVP_CONSTRAINT_FIXED, IVP_CONSTRAINT_FIXED,
     *                   IVP_CONSTRAINT_FREE,  IVP_CONSTRAINT_FREE,  IVP_CONSTRAINT_FREE}
     *                  (this is a ball-and-socket constraint)
     *****************************************************************************/
    IVP_CONSTRAINT_AXIS_TYPE axis_type[6];  // [author's note: think about a bitfield]
    IVP_FLOAT borderleft_Rfs[6], borderright_Rfs[6];
    //IVP_U_Point center_trans_Ros, center_rot_Ros;
    void fix_translation_axis(IVP_COORDINATE_INDEX which);  // make the constraint unmoveable in a specific direction
    void free_translation_axis(IVP_COORDINATE_INDEX which); // make the constraint moveable in a specific direction
    void limit_translation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right);  // Note: limitations are not yet fully tested.
    void fix_rotation_axis(IVP_COORDINATE_INDEX which);  // tell which rotation axis cannot be rotated around.
    void free_rotation_axis(IVP_COORDINATE_INDEX which); // tell which roation axis can be freely rotated around.
    void limit_rotation_axis(IVP_COORDINATE_INDEX which, IVP_FLOAT border_left, IVP_FLOAT border_right);  // Note: limitations are not yet fully tested.

    /******************************************************************************
     *  Feature:        maximpulse_type[6]
     *  Description:    You can set how strong the constraints are.
     *                  The maximpulse parameter(s) describe the maximum impulse that the constraint should endure,
     *                  described in N*s (Newton second, a metric impulse unit).
     *                  The maximpulse_type values have the following meanings:
     *                  IVP_CFE_NONE   --  the constraint has infinite force, no matter what maximpulse says.
     *                  IVP_CFE_CLIP   --  clip the constraint force to be maximal the given impulse (makes waygiving constraints). (not yet implemented)
     *                  IVP_CFE_BREAK  --  the constraint breaks (deletes itself) when the constraints' burden exceeds the given maximpulse.
     *                  IVP_CFE_BEND   --  not yet implemented.
     *  Attention:      Currently, you HAVE TO set all six maximpulse values to the *same* value
     *                  and all six maximpulse_type types to the *same* type since that concept is
     *                  implemented only partially yet.
     *  Attention:      the IVP_CFE_CLIP is still under construction, limiting rotation axes can be dangerous
     *  Note:           Currently, the maximpulse value is effectively internally very random
     *                  between (0.5 * maximpulse) and (1.0 * maximpulse), the latter being more probable.
     *  Default value:  {IVP_CFE_NONE, IVP_CFE_NONE, IVP_CFE_NONE, IVP_CFE_NONE, IVP_CFE_NONE, IVP_CFE_NONE},
     *                  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
     *****************************************************************************/
    IVP_CONSTRAINT_FORCE_EXCEED maximpulse_type[6];
    IVP_FLOAT maximpulse[6];
    void set_max_translation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse);
    void set_max_translation_impulse(IVP_COORDINATE_INDEX coord, IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse);    // not yet implemented:
    void set_max_rotation_impulse(IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse);
    void set_max_rotation_impulse(IVP_COORDINATE_INDEX coord, IVP_CONSTRAINT_FORCE_EXCEED impulsetype, IVP_FLOAT impulse);   // not yet implemented:

private: // internal functions
    void sort_coordinates(const IVP_Template_Constraint &tmpl);
public:
    /******************************************************************************
     *  Feature:        All-in-one constraint setting functions
     *  Description:    You may use one of the following functions if you prefer one single big
     *                  function call instead of many small function calls.
     *                  Each of the following functions set the constraint completely.
     *                  These functions do not provide the complete functionality,
     *                  however you can easily combine them with the above functions.
     *  Note:           Some of these functions provide some alternative ways of thinking of constraints.
     *  Attention:      All previous settings of Template_Constraint will be overwritten by each one of the following functions.
     *****************************************************************************/

    /******************************************************************************
     * Names of some special constraints:
     * ----------------------------------
     *
     * the first digit is nr of fixed translation dimensions, the second digit is nr of fixed rotation dimensions
     * english and german descriptions
     *                         _  _  _  _  _  _
     * 00 free, no limitation                   keine Verbindung
     * 01
     * 02
     * 03 always heading the same orientation _ Raumverschiebung
     * 10 rolling on a plane                    auf einer Ebene bewegbar
     * 11 a cardanjoint moving on a plane
     * 12 hinge that can be moved on a plane    drehbares Objekt auf Ebene
     * 13 movable on a plane   _  _  _  _  _  _ Doppelschiene
     * 20 direction                             verschiebbarer Punkt, Kugelbahn
     * 21 a cardanjoint moving on a rail        verschiebbares Kardangelenk
     * 22 a hinge that can be moved on a rail   Schiebescharnier
     * 23 slider, rail   _  _  _  _  _  _  _  _ Schieber, Schiene
     * 30 ball and socket                       Kugelgelenk
     * 31 cardanjoint                           Kardangelenk
     * 32 hinge                                 Scharnier
     * 33 fixed                                 fest verbunden
     *****************************************************************************/
    
    /******************************************************************************
     *  Parameters and their meanings:
     *  ------------------------------
     *
     *  objR:           The reference object pointer. NULL means 'no object' with the world system as object system.
     *  objA:           The attached object pointer. NULL means 'no object' with the world system as object system.
     *  anchor_ws:      The anchor point defines where the translation axes have their origin.
     *                  The point is given in world system.
     *  anchor_Ros:     The anchor point defines where the translation axes have their origin.
     *                  The point is given in the object system of the Reference object.
     *  fixed_axis_ws:  A vector representing the axis the rotation is fixed, given in world space.
     *                  This vector will be the X axis of the rotation axes.
     *  fixed_axis_Ros: A vector representing the axis the rotation is fixed, given in reference object space
     *                  This vector will be the X axis of the rotation axes.
     *  free_axis_ws:   A vector representing the axis the rotation is allowed, given in world space.
     *                  This vector will be the Z axis of the rotation axes.
     *  free_axis_Ros:  A vector representing the axis the rotation is allowed, given in the reference object space
     *                  This vector will be the Z axis of the rotation axes.
     *  known_axis_ws:  The axis that is given:
     *                  if fixedrotdim == 2 or fixedtransdim == 2: known_axis is the only free axis
     *                  if fixedrotdim == 1 or fixedtransdim == 1: known_axis is the only fixed axis
     *                  else: known_axis is not used.
     *  borderleft:     left rotation border of the hinge
     *  borderright:    right rotation border of the hinge
     *  distanceAR_Ros: a vector that is pointing from the anchor (of the reference object) to the point on the attached object
     *                  where the anchor should be. Seen in the object system of the Reference object.
     *  fixedtransdim:  How many of the translation coordinates are fixed (unmoveable for objects).
     *                  Ranging from 0 to 3. With this parameter, fixed coordinates are always beforen free coordinates,
     *                  if you think of {IVP_INDEX_X, IVP_INDEX_Y, IVP_INDEX_Z} as an enumeration.
     *  fixedrotdim:    How many of the rotation coordinates are fixed (unrotable for objects).
     *                  Ranging from 0 to 3. With this parameter, fixed coordinates are always beforen free coordinates,
     *                  if you think of {IVP_INDEX_X, IVP_INDEX_Y, IVP_INDEX_Z} as an enumeration.
     *  m_Rfs_f_Afs:    This matrix describes where the constraint translation axes should be on the attached object (Afs),
     *                  in the coordinate system of the translation axes of the Reference object (Rfs).
     *                  Use a NULL pointer if the attached object is in the correct position (the constraint is relaxed).
     *  m_Ros_f_Rfs:    the same as the variable with the same name.
     *  m_Ros_f_Rrs:    the same as the variable with the same name.
     *  type[6]:        IVP_CONSTRAINT_FIXED, IVP_CONSTRAINT_LIMITED, IVP_CONSTRAINT_FREE - just like the corresponding variable, look above.
     *  range_Rfs[6]:   how far the attached object is allowed to move or rotate in both directions from origin
     *                  when IVP_CONSTRAINT_LIMITED is set for this coordinate.
     *****************************************************************************/
    // maybe m_Rfs_f_Afs should be called transrot_displacementA_Rfs? [Author's note]


    /******************************************************************************
     *  Function:       set_constraint
     *  Note:           This function is one of the most complicated of all. If you want easier functions, look down to the inline functions.
     *  Description:    You can completely describe the linear components of a constraint with this function.
     *                  The parameters are described above. The anchor point and the known_axis vector are given in world coords.
     *****************************************************************************/
    void set_constraint_ws(IVP_Real_Object *objR, const IVP_U_Point *anchor_ws, const IVP_U_Point *known_axis_ws, const unsigned fixedtransdim, const unsigned fixedrotdim,
			   IVP_Real_Object *objA, const IVP_U_Matrix *m_Rfs_f_Afs);

    /******************************************************************************
     *  Function:       set_constraint
     *  Note:           This function is one of the most complicated of all. If you want easier functions, look down to the inline functions.
     *  Description:    You can completely describe the linear components of a constraint with this function.
     *                  The parameters are described above. The anchor point and the known_axis vector are given in objects coords of the Reference object.
     *****************************************************************************/
    void set_constraint_Ros(IVP_Real_Object *objR, const IVP_U_Point *anchor_Ros, const IVP_U_Point *known_axis_Ros, const unsigned fixedtransdim, const unsigned fixedrotdim,
			    IVP_Real_Object *objA, const IVP_U_Matrix *m_Rfs_f_Afs);


    /******************************************************************************
     *  Function:       set_stiffness_for_limited_axis
     *  Description:    allows to set for all limited axis a new stiffness.
     *                  Values between 0.0f ( == no force )
     *                         and     1.0f ( very stiff )
     *  Note:           Allows to remove the springy behaviour of limited axis
     *****************************************************************************/
    void set_stiffness_for_limited_axis(IVP_FLOAT stiffness);

    
    /******************************************************************************
     *  Function:       set_orientation
     *  Description:    Defines a constraint that enforces the objects to always have the same orientation.
     *****************************************************************************/
    void set_orientation(IVP_Real_Object *objR, IVP_Real_Object *objA) {
	set_constraint_ws(objR, NULL, NULL, 0, 3, objA, NULL);
    }

    /******************************************************************************
     *  Function:       set_ballsocket_ws
     *  Description:    Defines a constraint that allows objA to rotate around a point that is given in world coords and that is always in the same position relative to objR.
     *****************************************************************************/
    void set_ballsocket_ws(IVP_Real_Object *objR, const IVP_U_Point *anchor_ws, IVP_Real_Object *objA) {
	set_constraint_ws(objR, anchor_ws, NULL, 3, 0, objA, NULL);
    }

    /******************************************************************************
     *  Function:       set_ballsocket_Ros
     *  Description:    Defines a constraint that allows the objects to rotate around a point that is given in object coords of the Reference object.
     *****************************************************************************/
    void set_ballsocket_Ros(IVP_Real_Object *objR, const IVP_U_Point *anchor_Ros, IVP_Real_Object *objA) {
	set_constraint_Ros(objR, anchor_Ros, NULL, 3, 0, objA, NULL);
    }

    /******************************************************************************
     *  Function:       set_ballsocket_tense_Ros
     *  Description:    Defines a constraint that allows the objects to rotate around a point that is given in reference object space;
     *                  the vector that is pointing from the anchor to the point where the anchor should be on the Attached object
     *                  can be given in distanceAR_Ros. (the vector is given in reference  object space)
     *****************************************************************************/
    void set_ballsocket_tense_Ros(IVP_Real_Object *objR, const IVP_U_Point *anchor_Ros, IVP_Real_Object *objA, const IVP_U_Point *distanceAR_Ros) {
	IVP_U_Matrix m_displacement_Ros;
	m_displacement_Ros.init3();
	m_displacement_Ros.vv.set(distanceAR_Ros);
	set_constraint_Ros(objR, anchor_Ros, NULL, 3, 0, objA, &m_displacement_Ros);
    }

    /******************************************************************************
     *  Function:       set_cardanjoint_ws
     *  Description:    Defines a constraint that allows the objects to rotate around the anchor, but does not allow them to rotate
     *                  around the fixed axis. Anchor and axis are given in world coords.
     *****************************************************************************/
    void set_cardanjoint_ws(IVP_Real_Object *objR, const IVP_U_Point *anchor_ws, const IVP_U_Point *fixed_axis_ws, IVP_Real_Object *objA) {
        set_constraint_ws(objR, anchor_ws, fixed_axis_ws, 3, 1, objA, NULL);
    }

    /******************************************************************************
     *  Function:       set_cardanjoint_Ros
     *  Description:    Defines a constraint that allows the objects to rotate around the anchor, but does not allow them to rotate
     *                  around the fixed axis. Anchor and axis are given in object coords of the Reference object.
     *****************************************************************************/
    void set_cardanjoint_Ros(IVP_Real_Object *objR, const IVP_U_Point *anchor_Ros, const IVP_U_Point *fixed_axis_Ros, IVP_Real_Object *objA) {
	set_constraint_Ros(objR, anchor_Ros, fixed_axis_Ros, 3, 1, objA, NULL);
    }

    /******************************************************************************
     *  Function:       set_hinge_ws
     *  Description:    Defines a hinge constraint that allows the objects to rotate around the free axis which is mounted on the anchor.
     *                  anchor and axis are given in world coords.
     *****************************************************************************/
    void set_hinge_ws(IVP_Real_Object *objR, const IVP_U_Point *anchor_ws, const IVP_U_Point *free_axis_ws, IVP_Real_Object *objA) {
	set_constraint_ws(objR, anchor_ws, free_axis_ws, 3, 2, objA, NULL);
    }

    /******************************************************************************
     *  Function:       set_hinge_Ros
     *  Description:    Defines a hinge constraint that allows the objects to rotate around the free axis which is mounted on the anchor.
     *                  anchor and axis are given in object coords of the Reference object.
     *****************************************************************************/
    void set_hinge_Ros(IVP_Real_Object *objR, const IVP_U_Point *anchor_Ros, const IVP_U_Point *free_axis_Ros, IVP_Real_Object *objA) {
	set_constraint_Ros(objR, anchor_Ros, free_axis_Ros, 3, 2, objA, NULL);
    }

    /******************************************************************************
     *  Function:       set_hinge_Ros
     *  Description:    Defines a hinge constraint that allows the objects to rotate around the free axis which is mounted on the anchor,
     *                  but not to rotate beyond borderleft and borderright.
     *                  anchor and axis are given in object coords of the Reference object.
     *****************************************************************************/
    void set_hinge_Ros(IVP_Real_Object *objR, const IVP_U_Point *anchor_Ros, const IVP_U_Point *free_axis_Ros, IVP_Real_Object *objA,
		       IVP_FLOAT borderleft, IVP_FLOAT borderright);

    /******************************************************************************
     *  Function:       set_fixed
     *  Description:    Defines a completely fixed constraint. The attached object is fixed in its position to the reference object.
     *****************************************************************************/
    void set_fixed(IVP_Real_Object *objR, IVP_Real_Object *objA) {
	set_constraint_ws(objR, NULL, NULL, 3, 3, objA, NULL);
    }
    
public: // other functions
    /******************************************************************************
     *  Feature:        Constructor
     *  Description:    
     *****************************************************************************/
    IVP_Template_Constraint();

    //void set_target_angular_shift_os(const IVP_U_Quat &rot_attached);
    //void set_target_shift_os(const IVP_U_Point &trans_attached);
};

/********************************************************************************
 * How to finish a constraint:
 *
 * After you have defined your IVP_Environment *environment and filled up a
 * IVP_Template_Constraint template_constraint with values, do:
 * IVP_Constraint *constraint = environment->create_constraint(&template_constraint);
 * This will copy all values of template_constraint into the constraint. You can
 * reuse the template and change any values after you created a constraint from it;
 * these changes will not have any effect on the created constraint.
 * If you want to run-time-change a constraint, use the functions that are implemented
 * within the constraint. (see IVP_Constraint, ivp_constraint.hxx)
 * You can delete the constraint if you don't want it any more. (it was created by new())
 * Note: if any of the two objects of the constraints are deleted, the constraints deletes itself
 *******************************************************************************/

#endif
