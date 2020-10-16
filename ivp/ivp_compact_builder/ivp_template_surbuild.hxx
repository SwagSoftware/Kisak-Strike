#ifndef IVP_TEMPLATE_SURBUILD_LEDGE_SOUP_INCLUDED
#define IVP_TEMPLATE_SURBUILD_LEDGE_SOUP_INCLUDED

enum IVP_SURBUILD_LEDGE_SOUP_MERGE_POINT_TYPES {
    IVP_SLMP_NO_MERGE,
    IVP_SLMP_MERGE_AND_REALLOCATE,
    IVP_SLMP_MERGE_NO_REALLOCATE
};

class IVP_Template_Surbuild_LedgeSoup {
public:
    /********************************************************************************
     *  Parameter:    build_root_convex_hull
     *  Default:      IVP_FALSE
     *	Description:  if true than an extra convex hull around the whole object is generated
     *******************************************************************************/
    IVP_BOOL build_root_convex_hull;

    /********************************************************************************
     *  Parameter:    free_input_compact_ledges
     *  Default:      IVP_TRUE
     *	Description:  if true input data compact_ledges are freed using ivp_free_aligned()
     *******************************************************************************/
    IVP_BOOL free_input_compact_ledges;

    /********************************************************************************
     *  Parameter:    link_to_input_compact_ledges
     *  Default:      IVP_FALSE
     *	Description:  if true input data is reused by linking to it
     *  Note:	      if true than assert( !free_input_compact_ledges );
     *******************************************************************************/
    IVP_BOOL link_to_input_compact_ledges;

    /********************************************************************************
     *  Parameter:    merge_points
     *  Default:      IVP_SLMP_MERGE_AND_REALLOCATE
     *	Description:  true compile tries to merge all equal points
     *  Note:	      Only input ledges which are copied ( and not linked to ) are taken into account
     *	Note:	      For reasons of speed
     *  Attention:    The current implementation limits the number of points to 65k
     *		      If your resulting ledge contains more points, the engine will crash
     *	Values:	    IVP_SLMP_NO_MERGE	    result needs more memory but compile is fast
     *		    IVP_SLMP_MERGE_AND_REALLOCATE results needs less memory
     *		    IVP_SLMP_MERGE_NO_REALLOCATE result is placed in a chunk of memory
     *				same size as IVP_SLMP_NO_MERGE, but actually needs only
     *				same size as IVP_SLMP_MERGE_AND_REALLOCATE
     *				good for creating intermediate results
     *******************************************************************************/
    IVP_SURBUILD_LEDGE_SOUP_MERGE_POINT_TYPES merge_points;

    IVP_Template_Surbuild_LedgeSoup()
	{
	    build_root_convex_hull = IVP_FALSE;
		link_to_input_compact_ledges = IVP_FALSE;
		free_input_compact_ledges = IVP_TRUE;
#ifdef WIN32
	    merge_points = IVP_SLMP_MERGE_AND_REALLOCATE;
#else
		merge_points = IVP_SLMP_NO_MERGE;
#endif
	};
};

#endif // IVP_TEMPLATE_SURBUILD_LEDGE_SOUP_INCLUDED