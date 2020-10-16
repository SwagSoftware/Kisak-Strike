// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/** Common Info types for ivp_physic and ivp_template */

#ifndef _IVP_ACTUATOR_INFO_INCLUDED
#define _IVP_ACTUATOR_INFO_INCLUDED

class IVP_U_Active_Float;


/********************************************************************************
 *			Info classes for some actuator examples
 *			See ivp_actuator.hxx for actuator details
 ********************************************************************************/

// INTERN_START
/********************************************************************************
 *	Name:	       	IVP_Extra_Info
 *	Description:	The actuator extras are a test assembly of special
 *			solutions needed for the Ipion demo
 *	Attention:	don't use them, they should only be used as an example
 *	Version Info:	might be removed later
 ********************************************************************************/
class IVP_Extra_Info {
public:
    void *client_data;		// P_Actuator_Extra

    IVP_BOOL is_float_cam;
    IVP_U_Active_Float *active_float_bomb;
    IVP_DOUBLE range; // e.g. for bomb
    
    IVP_U_Active_Float *mod_fc_height;
    IVP_U_Active_Float *mod_fc_target_height;
    IVP_U_Active_Float *mod_fc_dist;
    IVP_U_Active_Float *mod_fc_speed;
    IVP_U_Active_Float *active_float_force;		// the force

    int is_physic_cam;
    
    int is_puck_force;
    IVP_U_Active_Float *mod_pf_forward;
    IVP_U_Active_Float *mod_pf_sideward;
    
    IVP_Extra_Info();	// memclear(this)
};

// INTERN_END





#endif














