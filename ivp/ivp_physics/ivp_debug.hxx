// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

class IVP_Mindist;

class IVP_Draw_Vector_Debug
{
public:
    IVP_Draw_Vector_Debug *next;
    IVP_U_Point first_point; 
    IVP_U_Point direction_vec;
    int color;
    char *debug_text; //may be null. must be freed afterwards
    IVP_Draw_Vector_Debug();
    ~IVP_Draw_Vector_Debug();
};

// returns IVP_TRUEif mindist is interesting
IVP_BOOL ivp_check_debug_mindist( IVP_Mindist *md );

#if 0
#define IVP_DEBUG_OBJECT0 "box0_5"
#define IVP_DEBUG_OBJECT1 "box2_4"
#define IVP_DEBUG_TIME 4.3
#else

#define IVP_DEBUG_OBJECT0 0
#define IVP_DEBUG_OBJECT1 0
#define IVP_DEBUG_TIME 0

#endif
