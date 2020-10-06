// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#define IVP_MAX_CORE_STACK 6

class IVP_Environment;

// INTERN_START
/********************************************************************************
 *	Name:	    	IVP_Core_Merged  	
 *	Description:	temporary core needed to resolve jamed objects
 *	Version Info:   Only limited useage in SDK_1xx, will be used extensively in
 *                      future versions.
 *      Attention:      Not for public useage
 ********************************************************************************/
class IVP_Core_Merged: public IVP_Core {
public:
    void set_by_merge(IVP_Real_Object *first_object);
    void set_by_merge(IVP_Core *c0,IVP_Core *c1);
    IVP_U_Matrix m_world_f_core_when_created;	// matrix when core is created
    IVP_Core_Merged(IVP_Core *core0, IVP_Core *core1);
	IVP_Core_Merged(IVP_Real_Object *real_obj);
};

class IVP_Core_Collision: public IVP_Core_Merged {
public:
    IVP_Core_Collision(IVP_Core *core0, IVP_Core *core1);
    IVP_Core_Collision *next_collision_core;
    void split_collision_merged_core_next_PSI();			// split this and use next_PSI matrix
};
// INTERN_END


class IVP_Merge_Core {
	friend class IVP_Environment;

    IVP_Core *core_stack[IVP_MAX_CORE_STACK];

    int n_cores;
    IVP_Core **cores;

    IVP_Core *unmovable_core;	// if there is at least one unmoveable core, this pointer points to it

    IVP_Movement_Type movement_type;		// ored movement
    // destination core
    IVP_Core_Merged *mcore;

    void check_for_unmovable_core();
    void synchronize_motion();
    void find_main_axis();
    void find_mass_center();
    void find_damp_factors();
    void place_objects();
    void set_speed();
    void set_radius();

protected:
    friend class IVP_Core_Merged;
    void calc_calc();
    
public:
    IVP_Merge_Core();
    IVP_Merge_Core(IVP_Core_Merged *mcore, IVP_Real_Object *first_object);
    IVP_Merge_Core(IVP_Core_Merged *mcore, IVP_Core *first_core, IVP_Core *second_core);
    ~IVP_Merge_Core();
};

