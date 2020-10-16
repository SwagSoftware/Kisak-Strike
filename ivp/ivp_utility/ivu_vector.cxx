// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>


void IVP_U_Vector_Base::increment_mem(){
    int i;
	IVP_ASSERT( memsize != 0xFFFF );
    void **new_elems = (void **)p_malloc(sizeof(void *) * 2* (memsize+1)); 
    int newMemsize = memsize * 2 + 1;
	if ( newMemsize > 0xFFFF )
	{
		memsize = 0xFFFF;
	}
	else
	{
		memsize = newMemsize;
	}

    for(i=0; i<n_elems; i++){
	new_elems[i] = elems[i];
    }
    if ( elems != (void **) (this+1)){
	P_FREE(elems);
    }
    elems = new_elems;
}

