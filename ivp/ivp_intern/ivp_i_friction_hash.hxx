// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef _IVP_FRICTION_HASH_INCLUDED
#define _IVP_FRICTION_HASH_INCLUDED


#ifndef _IVP_VHASH_INCLUDED
#	include  "ivu_vhash.hxx"
#endif


class IVP_Friction_Hash: protected IVP_VHash_Store
{
    friend class IVP_Core;
protected:
    
public:
    void add_friction_info(IVP_Friction_Info_For_Core *fs_info){
	add_elem( fs_info->l_friction_system , fs_info );
	IVP_IF(1){
	    IVP_Environment *env=fs_info->l_friction_system->l_environment;
	    if( env->get_debug_manager()->check_fs ) {
		IVP_ASSERT( find_friction_info(fs_info->l_friction_system) == fs_info );
	    }
	}
    };

    IVP_Friction_Info_For_Core *remove_friction_info(IVP_Friction_Info_For_Core *fs_info){
	return (IVP_Friction_Info_For_Core *)remove_elem( fs_info->l_friction_system );
    };

    IVP_Friction_Info_For_Core *find_friction_info(IVP_Friction_System *fs){
	return (IVP_Friction_Info_For_Core *)find_elem( fs );
    };
    
    int friction_hash_len(){ return size;};
    IVP_Friction_Info_For_Core *element_at(int i){ return (IVP_Friction_Info_For_Core *)elems_store[i].elem;};

    ~IVP_Friction_Hash();
    IVP_Friction_Hash(int size_):IVP_VHash_Store(size_){;};
    void print(){ IVP_VHash_Store::print();};
};


#endif





