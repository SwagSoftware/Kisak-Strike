// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

/********************************************************************************
 *	File:	       	ivp_forcefield.hxx	
 *	Description:	IVP_ForceFields are convinient managers to Force Fields
 ********************************************************************************/

#ifndef IVP_FORCEFIELD_INCLUDED
#define IVP_FORCEFIELD_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#ifndef IVP_CONTROLLER_INCLUDED
#include <ivp_controller.hxx>
#endif

#ifndef IVP_SET_INCLUDED
#	include <ivu_set.hxx>
#endif

/********************************************************************************
 *	Name:	  	IVP_ForceField
 *	Description:	Force fields are a set of independent forces applied
 *			to objects.
 ********************************************************************************/
class IVP_Forcefield: protected IVP_Listener_Set_Active<IVP_Core>, protected IVP_Controller_Independent
{
protected:
    IVP_U_Set_Active<IVP_Core> *set_of_cores;
    IVP_BOOL i_am_owner_of_set_of_cores; // default: IVP_TRUE; if IVP_FALSE, set_of_cores will not be deleted by deconstructor!

    // IVP_Listener_Set_Active implementations
    virtual void element_added(IVP_U_Set_Active<IVP_Core> *set, IVP_Core *elem);
    virtual void element_removed(IVP_U_Set_Active<IVP_Core> *set, IVP_Core *elem);
    virtual void pset_is_going_to_be_deleted(IVP_U_Set_Active<IVP_Core> *set);   // Note: pset is not removing elements when deleted

    // IVP_Controller implementations (only some)
    virtual void core_is_going_to_be_deleted_event(IVP_Core *);   // default deletes this
    
    virtual void do_simulation_controller(IVP_Event_Sim *,IVP_U_Vector<IVP_Core> *core_list)=0;
    virtual IVP_CONTROLLER_PRIORITY get_controller_priority();
    IVP_Forcefield(IVP_Environment *env, IVP_U_Set_Active<IVP_Core> *set_of_cores, IVP_BOOL owner_of_set);
public:
    ~IVP_Forcefield();
};


#endif
