// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _IVP_CONSTRAINT_TYPES_INCLUDED
#define _IVP_CONSTRAINT_TYPES_INCLUDED

enum IVP_CONSTRAINT_AXIS_TYPE {
    IVP_CONSTRAINT_AXIS_FREE = 0x0,        // this axis is not constrained
    IVP_CONSTRAINT_AXIS_FIXED = 0x1,       // this axis is constrained
    IVP_CONSTRAINT_AXIS_LIMITED = 0x2,     // this axis has a limited area
    //IVP_CONSTRAINT_AXIS_NOT_LIMITED = 0xD, // Do not use this constant
    //IVP_CONSTRAINT_AXIS_NOT_FIXED = 0xE,    // Do not use this constant
    IVP_CONSTRAINT_AXIS_DUMMY = 0xffffffff
};

enum IVP_CONSTRAINT_FORCE_EXCEED {  // Do not use this enum yet.
    IVP_CFE_NONE,
    IVP_CFE_CLIP,       // clip force to scalar
    IVP_CFE_BREAK,      // destroy constraint if force is too great
    IVP_CFE_BEND        // Constraint bends itself if force is too great -- not yet implemented
    //IVP_CFE_PROCENTUAL,    // multiply force with that value
    //IVP_CFE_APPROXIMATION  // relaxation rate
};

enum IVP_CONSTRAINT_FLAGS {  // Do not use this enum yet
    IVP_CONSTRAINT_ACTIVATED = 0x10,
    //    IVP_CONSTRAINT_LOCAL = 0x0,
    IVP_CONSTRAINT_GLOBAL = 0xF,
    IVP_CONSTRAINT_FAST = 0x0,
    IVP_CONSTRAINT_SECURE = 0x10
};

enum IVP_NORM {
    IVP_NORM_MINIMUM,
    IVP_NORM_EUCLIDIC,
    IVP_NORM_MAXIMUM
};

#endif
