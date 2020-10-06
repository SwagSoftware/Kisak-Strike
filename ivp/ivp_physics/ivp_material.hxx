// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef IVP_MATERIAL_INCLUDED
#define IVP_MATERIAL_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

class IVP_Real_Object;

#ifndef _IVP_CONTACT_SITUATION_INCLUDED
#	include <ivp_contact_situation.hxx>
#endif

/********************************************************************************
 *	Name:	       	P_MATERIAL_TYPE
 *	Description:	
 *	Attention:	only P_MATERIAL_TYPE_TERMINAL is supported now.
 *	Version Info:	Future releases will probably use virtual materials:
 *			needed when an object collides with one edge and both
 *			neighbour surface have different materials
 ********************************************************************************/
enum P_MATERIAL_TYPE
{
    P_MATERIAL_TYPE_TERMINAL,
    P_MATERIAL_TYPE_LAST
};


/********************************************************************************
 *	Name:	       	IVP_Material
 *	Description:	this material class has to be implemented by the user application.
 *			it serves the collision and friction parameters to the
 *			material manager, which then calculates the real object-object
 *			parameters for the physical simulation by [optionally] combining
 *			the two materials properties.
 *	Attention:	Its' functions are called every collision, so the implementation
 *			should be quite fast.
 *	Version Info:
 ********************************************************************************/
class IVP_Material
{
    // base class that may be implemented in different ways (e.g. IVP_Material_Simple);
public:
    P_MATERIAL_TYPE material_type; // indicates way of implementation
    IVP_BOOL        second_friction_x_enabled; // when we want two different friction values (e.g. for Skis) 
  
    virtual IVP_DOUBLE get_friction_factor()=0;
    virtual IVP_DOUBLE get_second_friction_factor()=0; // second friction factor when two friction values is enabled
    virtual IVP_DOUBLE get_elasticity()=0;
    // INTERN_START
    virtual IVP_DOUBLE get_adhesion()=0;  // for future releases
    // INTERN_END
    virtual const char *get_name() = 0;		// helps debugging
    virtual ~IVP_Material();
    IVP_Material() { second_friction_x_enabled = IVP_FALSE; }
};

/********************************************************************************
 *	Name:	       	IVP_Material_Simple
 *	Description:	very simple material
 *	Version Info:
 ********************************************************************************/
class IVP_Material_Simple : public IVP_Material {
protected:
    IVP_Material_Simple();
public:
    IVP_DOUBLE friction_value;
    IVP_DOUBLE second_friction_x;
    IVP_DOUBLE elasticity;
    // INTERN_START
    IVP_DOUBLE adhesion;
    virtual IVP_DOUBLE get_adhesion();
    // INTERN_END
    
    virtual IVP_DOUBLE get_friction_factor();
    virtual IVP_DOUBLE get_second_friction_factor() { return second_friction_x; };
    virtual IVP_DOUBLE get_elasticity();
    virtual const char *get_name();		// helps debugging
    IVP_Material_Simple(IVP_DOUBLE friction, IVP_DOUBLE elasticity);
    virtual ~IVP_Material_Simple();
};

/********************************************************************************
 *	Name:	       	IVP_Material_Manager
 *	Description:	the material manager serves the collision parameters to
 *			the Ipion Virtual Physics Engine.
 *			The user should provide an implementation which calculates the
 *			values given the materials of both objects.
 *	Note:		You may use the default implementation of IVP_Material_Manager
 *			which simply multiplies the values of both materials
 *			
 *	Attention:	The simulation does not handle static friction and sliding
 *			friction differently. (Different values for static and
 *			moving friction will easily lead to non linear behavior,
 *		        thus creating high frequencies of object movement (like
 *			chalk on a blackboard))
 ********************************************************************************/
class IVP_Material_Manager
{
    IVP_BOOL delete_on_env_delete; // see deconstructor
public:
  virtual IVP_Material *get_material_by_index(const IVP_U_Point *world_position, int index); // returns the material, if material index in compact_triangle != 0
    virtual IVP_DOUBLE get_friction_factor(IVP_Contact_Situation *situation);	// returns values >0, value of 1.0f means object stands on a 45 degres hill
    virtual IVP_DOUBLE get_elasticity(IVP_Contact_Situation *situation);		// range [0, 1.0f[, the relative speed after a collision compared to the speed before
    // INTERN_START
    virtual IVP_DOUBLE get_adhesion(IVP_Contact_Situation *situation);		//
    // INTERN_END
    IVP_Material_Manager(IVP_BOOL delete_on_env_delete);
    virtual ~IVP_Material_Manager(){;};
    
    virtual void environment_will_be_deleted(IVP_Environment *){
	if (delete_on_env_delete){
	    P_DELETE_THIS(this);
	}
    };
};

#endif



