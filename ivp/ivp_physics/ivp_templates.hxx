// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	Name:	       	ivp_templates
 *	Description:	interface structures, needed to send the world representation
 *			to the physics engine
 *      See also:       ivp_template_constraint.hxx for a constraint template
 *	Attention:	All IVP_*_Template structures are used to transport information
 *			and therefore are only needed for a limited period of time:
 *			after usage, the user should free the memory of the structures.
 *	Attention:	All surfaces have to share the same lines!
 *			All lines have to share the same points!
 *			This means that no two point instances may exist with the same coordinates.
 *	Attention:	Filling the structures incorrectly may result in program hangup or crash
 *			(in the current version there is no plausible check).
 *	Attention:     	The physical simulation is optimized to handle normal sized ojects
 *			(0.1f - 100 meter diameter)
 *			Objects which are much larger may result in unnecessary CPU usage.
 *			Smaller objects cause simulation inaccuracies to become visible.
 *			Therefore, you should rescale your world according to whether
 *			you want to simulate colliding planets or jumping insects
 *			(this may change in the future versions).
 *	Attention:	Units: Meters (m), Kilo (kg), Newtons (N)
 ********************************************************************************/

class IVP_Real_Object;
class IVP_Material;

#ifndef _IVP_U_LINEAR_INCLUDED
#	include "ivu_linear.hxx"
#endif


/********************************************************************************
 *	Name:	       	IVP_Template_Object
 *	Description:	An object
 ********************************************************************************/
class IVP_Template_Object {
    char *name;
public:
    void set_name(const char *name);
    const char *get_name() const { return name;}
    IVP_Template_Object();
    ~IVP_Template_Object();
};



/********************************************************************************
 *	Name:	       	IVP_Template_Cluster
 *	Description	A container for IVP_Objects
 *	Attention:	There is no use for clusters in the current implementation
 *			of the physics engine.
 ********************************************************************************/
class IVP_Template_Cluster : public IVP_Template_Object {
public:
    // ...
};



/********************************************************************************
 *	Name:	     	IVP_Template_Real_Object
 *	Description:	A real object with mass and rotation inertia properties.
 ********************************************************************************/
class IVP_Template_Real_Object : public IVP_Template_Object {
    char nocoll_group_ident[IVP_NO_COLL_GROUP_STRING_LEN];		 // Optional (malloced) string used by the IVP_Collision_Filter_Coll_Group_Ident (use set ... to set this)
public:
    IVP_BOOL physical_unmoveable;	 // If set to TRUE, the object is unmoveable (cannot be changed later)
    IVP_BOOL enable_piling_optimization; // Objects get more wobbly but simulation of object piles is faster (dominos)
	IVP_BOOL pinned;
    // IVP_BOOL low_simulation_forbidden;   // not used anymore
    IVP_Material *material;		 // Pointer to the material (used only if no surface material is set)
    void set_nocoll_group_ident(const char *nocoll_group);        // Optional string used by the IVP_Collision_Filter_Coll_Group_Ident, maxlen = IVP_NO_COLL_GROUP_STRING_LEN
    const char *get_nocoll_group_ident() const { return &nocoll_group_ident[0]; };
    IVP_DOUBLE mass;			 // Mass of object (unit: kilogram), default 1.0

/********************************************************************************
 *	Feature:       	rot_inertia
 *	Description:	To set the rotation inertia, you may either:
 *			1. set the rotation inertia directly (rot_inertia_is_factor = IVP_FALSE)
 *			2. set a factor, which is multiplied with the automatically calculated rotation inertia
 *			   (by the Ipion ViPE). (rot_inertia_is_factor = IVP_TRUE)
 *	Note:		Increasing rot_inertia helps to stabilize objects.
 *	Attention:	To define rotation inertia, only the diagonal values of the rotation inertia 3x3 matrix
 *			can be set. This is no restriction since every object can be
 *			rotated to a main rotation axis coordinate system in which only the
 *			diagonal values of the rotation inertia matrix are not zero.
 *	Note for non-physicians: To avoid dealing with rotation inertia, try to design
 *			your object mass symetrically to the objects coordinate system.
 *			Since small errors will hardly be visible later (as long as you are
 *			not designing a thin slanted pole), there is no need to
 *			worry about the exact values. Simply let the Ipion Virtual
 *			Physics Engine do the job for you (rot_inertia_is_factor= IVP_TRUE).
 *			The engine calculates the rot_inertia assuming homogenous objects
 *			(entirely made out of the same material).
 *			To simulate hollow objects, simply increase the rot_inertia values slightly:
 *			(rot_inertia_is_factor=IVP_TRUE, rot_inertia= (>1.0f, >1.0f, >1.0f)
 *      Auto check:     Default is to auto check rotation inertia to avoid artefacts (like jumping like crazy)
 *                      When set to <1.0f auto check is on and values are clipped against
 *                      old_rot_inertia.length() * auto_check_rot_inertia,
 *						e.g. 0.0 -> no clipping (dangerous for very thin objects, however very realistic),
 *                           0.1 -> medium
 *                           0.5 -> strong (safe and unrealistic)
 *						default is 0.03
 ********************************************************************************/
    IVP_BOOL rot_inertia_is_factor;			// default: IVP_TRUE, see above
    IVP_U_Float_Point rot_inertia;				/* the diagonal values of the rotation inertia matrix
							 *  default: (1.0f,1.0f,1.0f)
							 */
    IVP_FLOAT auto_check_rot_inertia;
    
/********************************************************************************
 *	Feature:	   	Dampening    	
 *	Description:		Object may be dampened in many ways. The following
 *				two values allow dampening of the movement of
 *				an object in world space. This method works
 *				extremely well, is numerically  stable, very fast
 *				and easy to use.
 *	Usage:			0.0f means no dampening
 *				>0.0f means dampening
 *				1.0f means decrease the speed every second to 1/e of
 *				its original value.
 *	Mathematics:		new_speed = old_speed * e**( -damp_value * time)
 *
 *	speed_damp_factor:	used to damp the speed
 *	rot_speed_damp_factor:	used to damp the rotation of the object.
 *				Every axis has its own value.
 *	Attention:		Only values >=0 make sense, otherwise the objects
 *				will GAIN speed very rapidly.
 *	Note:			Values between 0 and 0.05f are hardly noticeable.
 *				Values greater 1 cause the objects to appear as if
 *				swimming in liquid.
 *		      		Experiment with different values.
 ********************************************************************************/
    IVP_DOUBLE speed_damp_factor;		// default 0.01f
    IVP_U_Point rot_speed_damp_factor; //  default 0.01f,0.01f,0.01f

    IVP_FLOAT extra_radius;                // extra radius around each object

/********************************************************************************
 *	Feature:	Rotation axis and mass center
 *	Description:	By default (mass_center_override=NULL), the Ipion ViPE
 *			calculates the center of mass automatically and assumes
 *			that the main rotation axes are the axes of the objects
 *			coordinate system.
 *			By defining a mass_center_override matrix,
 *			the user may set the rotation axis directly
 *			(rows of the mass_center_override matrix are the new rotation
 *			axes in object space. The mass_center_override.vv (shift value)
 *			should be set to the new mass center of the object in
 *			object space).
 *	Hint:		Simply design your objects symmetrically to the object coordinate system
 *			and forget mass_center_override.
 ********************************************************************************/
    IVP_U_Matrix *mass_center_override; // optional

    
/********************************************************************************
 *	Feature:	Rotation axis and mass center
 ********************************************************************************/
    void *client_data;			// user supplied client data
    IVP_Template_Real_Object();
    ~IVP_Template_Real_Object();       
};


/********************************************************************************
 *	Name:	       	IVP_Template_Ball
 *	Version Info:	
 ********************************************************************************/
class IVP_Template_Ball {
public:
        IVP_FLOAT radius; // unit: meters // add extra_radius in IVP_Template_Real_Object to get real radius
};



