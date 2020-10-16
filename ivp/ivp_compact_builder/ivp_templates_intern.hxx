// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	Name:	       	ivp_templates
 *	Description:	interface structures, needed to send the world representation
 *			to the physics engine
 *	Attention:	All IVP_*_Template structures are used to transport information
 *			and therefore are only needed for a limited period of time:
 *			after usage, the user should free the memory of the structures.
 *	Attention:	All surfaces have to share the same lines!
 *			All lines have to share the same points!
 *			This means that no two point instances may exist with the same coordinates.
 *	Attention:	Filling the structures incorrectly may result in program hangup or crash
 *			(in the current version there is no plausible check).
 *	Attention:     	The physical simulation is optimized to handle normal sized ojects
 *			(0.3f - 50 meter diameter)
 *			Objects which are much larger may result in unnececarry CPU usage.
 *			Smaller objects cause simulation inaccuracies to become visible.
 *			Therefore, you should rescale your world according to whether
 *			you want to simulate colliding planets or jumping insects
 *			(this may change in the future versions).
 *	Attention:	All values are: Meters, Kilo, Newtons
 ********************************************************************************/

class IVP_Template_Surface;
class P_Surface;		// debugging only


/********************************************************************************
 *	Name:	      	IVP_Template_Point 	
 *	Description:	a vertex in object space
 ********************************************************************************/
class IVP_Template_Point:public IVP_U_Point {
public:
};


/********************************************************************************
 *	Name:	      	IVP_Template_Line 	
 *	Description:	An edge of an polygonal object. 
 *	Attention:	Two surfaces have to reference a line.
 *			Line should be longer than 10 mm.
 ********************************************************************************/
class IVP_Template_Line {
public:
    ushort p[2];	// end points of a line stored as a point index (see point array)
    void set(ushort a, ushort b){ p[0] = a; p[1] = b;};
};


/********************************************************************************
 *	Name:	       	IVP_Template_Polygon
 *	Description:	a polygon
 *	Attention:	there is no plausible check for filled templates
 *			after usage the template is not needed any more
 ********************************************************************************/
class IVP_Template_Polygon {
public:
    int n_points;		 	// number of points
    IVP_Template_Point *points;	 	// array of all points
    
    int n_lines;			// number of lines
    IVP_Template_Line *lines;		// an array of all lines
    
    int n_surfaces;			// number of surfaces
    IVP_Template_Surface *surfaces;	// an array of all surfaces

    /** methods **/
    void scale(int scaling_factor);
    
    IVP_Template_Polygon();		// empty polygon
    IVP_Template_Polygon(int point_count,
			 int line_count,
			 int surface_count);	// polygon with n_points, n_lines, n_surfaces
    ~IVP_Template_Polygon();
};


/********************************************************************************
 *	Name:	       	IVP_Template_Triangle
 *	Description:	
 ********************************************************************************/
class IVP_Template_Triangle {
public:
    IVP_Template_Point tri_points[3]; // array of all 3 points of triangle
    IVP_Template_Triangle(){ ; }; // used by graph_lib
};


/********************************************************************************
 *	Name:	       	IVP_Template_Ledge_Polygon_Soup
 *	Description:	
 ********************************************************************************/
class IVP_Template_Ledge_Polygon_Soup {
public:
    IVP_BOOL ledge_is_open; // IVP_FALSE means: ledge is already closed (and convex!)
    
    int n_templ_triangles;
    IVP_Template_Triangle *templ_triangles_array;    
           // ATTENTION: right now,
           // n_templ_triangles must be 2
           // and ledge_is_open must be IVP_FALSE
};


/********************************************************************************
 *	Name:	      	IVP_Template_Surface
 *	Description:	A surface of an object
 *	Note:		Holes in the surface are allowed.
 *			The material referenced by the surface is assumed to be long term.
 *			Each surface uses a line index array which stores the line indices
 *			(see IVP_Template_Object::lines) and a revert_line array which
 *			stores flags about the direction of the lines.
 *		        The order of the lines is not important. 
 *
 *	Attention:	When walking from the start_point of a line to the end_point,
 *			the surface has to be on the left side (in a right hand
 *			coordinate system).
 *			Technically: for surface line number i (0<i<n_lines):
 *			walking from point nr
 *			templ_poly->lines[lines[i]].p[revert_line[i]] to 
 *			templ_poly->lines[lines[i]].p[1-revert_line[i]] the
 *			surface has to be on the left side
 ********************************************************************************/
class IVP_Template_Surface {
public:
    IVP_U_Point normal; 		// normal

    IVP_Template_Polygon *templ_poly;    
    
    int n_lines;	/* number of lines for this surface */
    ushort *lines;	// all lines of this surfaces: Stored in an array of indices which refer to elements in the templ_poly->lines array
    char *revert_line;	/* an array of flags, indicating whether a line referred to by the line index array
			 * should be reversed */
    
    void calc_surface_normal_template(int point_index0,int point_index1,int point_index2); // points have to be counter clockwise
    
    /** methods **/
    void set_line(int sur_l_i, int line_index, char revert){
	lines[sur_l_i] = line_index; revert_line[sur_l_i] = revert;
    }
    void init_surface(int line_count);	// malloc line_count lines
    void close_surface();		// free all values: 'lines' array and 'revert_line' array
    IVP_Template_Surface();
    ~IVP_Template_Surface();		// note: calls close_surface
    int get_surface_index();
};













