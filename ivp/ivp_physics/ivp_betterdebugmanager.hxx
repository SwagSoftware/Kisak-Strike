// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

#ifndef _IVP_BETTERDEBUGMANAGER_INCLUDED
#define _IVP_BETTERDEBUGMANAGER_INCLUDED


#define IVP_IFDEBUG(dci)	if (ivp_debugmanager.is_debug_enabled(dci))


// NOTE: The values from 0-1023 are reserved for internal
//       Ipion use. Feel free to define your own debug
//       identifiers using values from 1024-2047.
enum IVP_DEBUG_CLASS {
  IVP_DM_DUMMY,
  IVP_DM_SURBUILD_POINTSOUP,
  IVP_DM_SURBUILD_HALFSPACESOUP,
  IVP_DM_SURBUILD_LEDGESOUP,
  IVP_DM_SURBUILD_Q12,
  IVP_DM_CONVEX_DECOMPOSITOR,
  IVP_DM_QHULL,
  IVP_DM_GEOMPACK_LEVEL1,
  IVP_DM_GEOMPACK_LEVEL2,
  IVP_DM_GEOMPACK_LEVEL3,
  IVP_DM_CLUSTERING_SHORTRANGE_VISUALIZER,

  IVP_DEBUG_IPION_ERROR_MSG = 1024,

  IVP_DEBUG_MAX_N_CLASSES = 2048 // DO NOT CHANGE THIS VALUE!
};

class IVP_BetterDebugmanager {
    int initialized;
    int flag_list[IVP_DEBUG_MAX_N_CLASSES];

public:
    void     enable_debug_output(IVP_DEBUG_CLASS class_id);
    void     disable_debug_output(IVP_DEBUG_CLASS class_id);

    IVP_BOOL is_debug_enabled(IVP_DEBUG_CLASS class_identifier);
    void     dprint(IVP_DEBUG_CLASS class_id, const char *formatstring, ...);

    // feel free to override this method with your customized
    // output function (e.g. for redirecting all debug outputs
    // into a file)
    virtual void output_function(IVP_DEBUG_CLASS class_id, const char *string);

    IVP_BetterDebugmanager();
    virtual ~IVP_BetterDebugmanager();
};

// use global variable only in emergency
extern IVP_BetterDebugmanager ivp_debugmanager;


#endif
