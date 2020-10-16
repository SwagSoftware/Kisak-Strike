// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <ivp_i_collision_vhash.hxx>


IVP_Collision_Callback_Table_Hash::~IVP_Collision_Callback_Table_Hash() 
{
    for (int i=this->len()-1; i>=0; i--) {
	IVP_Collision_Callback_Table *table;
	table = (IVP_Collision_Callback_Table *)this->element_at(i);
	P_DELETE(table);
    }
}

int IVP_Collision_Callback_Table_Hash::object_to_index(IVP_Real_Object *real_object)
{
    IVP_Real_Object *real_obj = real_object;
    return hash_index( (char *)&real_obj, sizeof(real_obj) );
}

IVP_BOOL IVP_Collision_Callback_Table_Hash::compare(void *elem0, void *elem1) const
{
    IVP_Collision_Callback_Table *table0 = (IVP_Collision_Callback_Table *)elem0;
    IVP_Collision_Callback_Table *table1 = (IVP_Collision_Callback_Table *)elem1;

    if ( table0->real_object != table1->real_object) return(IVP_FALSE);
    
    return(IVP_TRUE);
}








