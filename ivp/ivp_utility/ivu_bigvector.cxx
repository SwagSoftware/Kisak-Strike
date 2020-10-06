// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>


void IVP_U_BigVector_Base::increment_mem(){
    int i;
    void **new_elems = (void **)p_malloc(sizeof(void *) * 2* (memsize+1)); 
    memsize = memsize * 2 + 1;

    for(i=0; i<n_elems; i++){
	new_elems[i] = elems[i];
    }
    if ( elems != (void **) (this+1)){
	P_FREE(elems);
    }
    elems = new_elems;
}

