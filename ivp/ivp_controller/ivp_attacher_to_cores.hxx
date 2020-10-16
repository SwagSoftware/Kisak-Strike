// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_ATTACHER_TO_CORES_INCLUDED
#define _IVP_ATTACHER_TO_CORES_INCLUDED

#ifndef WIN32
#	pragma interface
#endif



#ifndef _IVP_VHASH_INCLUDED
#	include <ivu_vhash.hxx>
#endif

#ifndef IVP_SET_INCLUDED
#	include <ivu_set.hxx>
#endif

/********************************************************************************
 *	Name:	  	IVP_Attacher_To_Cores
 *	Description:	IVP_Attacher_To_Cores manages attachments to cores
 *			ATTACH_T must implement:
 *				ATTACH_T::ATTACH_T( IVP_Attacher_To_Cores *, IVP_Core *)
 *                              and call attachment_is_going_to_be_deleted in it's destructor
 *			ATTACH_T controlles just one core
 *      Note:           ATTACH_T is resonsible for adding itself to the core in the constructor
 *			and removing itself from the core at the destructor
 *			It also must listen to the core for IVP_Core::~IVP_Core
 *			Only one attachment per core is allowed
 *      Important Note: The destructor of ATTACH_T     M U S T   call
 *			IVP_Multi_Controller_Set::attachment_is_going_to_be_deleted
 ********************************************************************************/

template<class ATTACH_T>
class IVP_Attacher_To_Cores: protected IVP_Listener_Set_Active<IVP_Core>  {
    
public:
    IVP_VHash_Store core_to_attachment_hash;
protected:
    IVP_U_Set_Active<IVP_Core> *set_of_cores;

    virtual ~IVP_Attacher_To_Cores(){ // called by pset_is_going_to_be_deleted
	set_of_cores->remove_listener_set_active(this);
    }
     
    void element_added(IVP_U_Set_Active<IVP_Core> *, IVP_Core *elem){
	ATTACH_T *con = new ATTACH_T(this, elem);
	core_to_attachment_hash.add_elem( elem, con);
    }
    
    void element_removed(IVP_U_Set_Active<IVP_Core> *, IVP_Core *elem){
	ATTACH_T *con = (ATTACH_T *) core_to_attachment_hash.find_elem(elem);
	IVP_ASSERT(con);
	P_DELETE(con);
    }
    
    void pset_is_going_to_be_deleted(IVP_U_Set_Active<IVP_Core> *set){
	IVP_U_Set_Enumerator<IVP_Core> all_cores(set);
	while ( IVP_Core *core = all_cores.get_next_element(set) ){
	    this->element_removed( NULL, core );
	}
	IVP_ASSERT( core_to_attachment_hash.n_elems() == 0);
	P_DELETE_THIS(this);
    }

public:

    void attachment_is_going_to_be_deleted(ATTACH_T *, IVP_Core *attached_core){
	core_to_attachment_hash.remove_elem( attached_core);
    }

    IVP_Attacher_To_Cores(IVP_U_Set_Active<IVP_Core> *set_of_cores_in) : core_to_attachment_hash(16) {
	set_of_cores = set_of_cores_in;
	IVP_U_Set_Enumerator<IVP_Core> all_cores(set_of_cores);
	while ( IVP_Core *core = all_cores.get_next_element(set_of_cores) ){
	    this->element_added( NULL, core );
	}
	set_of_cores->add_listener_set_active(this);
    }
    
};

#endif
