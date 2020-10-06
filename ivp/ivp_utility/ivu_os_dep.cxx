// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>

#if !defined(SUN) && !defined(SUN4)
#	include <string.h> 
#endif
#include <ivu_types.hxx> 

// WIN32 ---------------------------------------------------------------------

static int IVP_RAND_SEED = 1;

// returns 0 .. 1.0
IVP_FLOAT ivp_rand(){
    IVP_RAND_SEED *= 75;
    IVP_FLOAT res=(IVP_RAND_SEED & 0xffff) / (float) 0x10000;
    return res;
}

void ivp_srand(int seed){
    if (seed == 0) seed = 1;
    IVP_RAND_SEED = seed;
}

int ivp_srand_read(void)
{
	return IVP_RAND_SEED;
}

extern void *p_malloc(unsigned int size);




#if 0 
FILE *p_glob_fp;
void p_init_glob_fp()
{
#ifdef WIN32
	p_glob_fp=fopen(ERRORFILEPATH,"a");
#else
	p_glob_fp=stdout;
#endif
}
#endif




