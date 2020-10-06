// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_contact_situation.hxx	
 *	Description:	???
 ********************************************************************************/

#ifndef _IVP_CONTACT_SITUATION_INCLUDED
#define _IVP_CONTACT_SITUATION_INCLUDED

class IVP_Real_Object;
class IVP_Material;
#ifndef WIN32
#	pragma interface
#endif


/********************************************************************************
 *	Name:	      	IVP_Contact_Situation 	
 *	Description:	Contact situations are simple, condensed information
 *			about a collision.
 ********************************************************************************/
struct IVP_Contact_Situation
{
	// can be requested in a collision or friction situations
	IVP_U_Float_Point surf_normal; 		// 16bytes: the normized normal of the contact surface (world space), pointing to second obj
	IVP_U_Float_Point speed;			// 16bytes: the speed (world space) of contact surface1 relative to contact surface0
	IVP_U_Point contact_point_ws;		// 16bytes[ND]:
	IVP_Real_Object* objects[2];		// 8bytes: link to involved objects
	const class IVP_Compact_Edge* compact_edges[2]; // 8bytes: link to involved Edges (needs source code license for more info
	IVP_Material* materials[2];	        // 8bytes: materials (might be virtual materials for point and edge cases)
	IVP_Contact_Situation() {;};
};

#endif
