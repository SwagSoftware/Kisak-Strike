// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef _P_LIST_INCLUDED
#	include <ivu_list.hxx>
#endif

class IVP_Triangle;
class P_OPoint;
class P_Surface;
class P_Object_Polygon;
class P_Sur_2D_Line;
class IVP_Object_Polygon_Tetra;
class IVP_Template_Surface;
class P_Sur_2D_Point : public IVP_U_Point {
public:
    // 2d koords, 3rd koord unused!
    int point_num; // point number of object (o)points

    // calc section
    int was_reached; // used together with lines list for island checking
    P_Sur_2D_Line *line_ref; // the line with startpoint = this
    
    P_Sur_2D_Point(int i_point_num); // koords have to be computed afterwards!
    ~P_Sur_2D_Point(); // removes all line list entries
};

class P_Sur_2D_Line{
public:
    P_Sur_2D_Line *next;
    P_Sur_2D_Line *prev;
    
    P_Sur_2D_Point *start_point; // on the way from start to end:
    P_Sur_2D_Point *end_point;   // filled area lies to the left!

    IVP_DOUBLE delta_x;
    IVP_DOUBLE delta_y;

    P_Sur_2D_Line(P_Sur_2D_Point *start_p, P_Sur_2D_Point *end_p);
    ~P_Sur_2D_Line(); // end/start_points remain!
    
    int point_lies_to_the_left(IVP_U_Point *i_point);
    IVP_DOUBLE dist_to_point(IVP_U_Point *i_point);
    IVP_DOUBLE hesse_dist_to_point(IVP_U_Point *i_point);
    int is_crossing_line(P_Sur_2D_Line *i_line);
    int point_lies_in_interval(IVP_U_Point *i_point);
    int overlaps_with_line(P_Sur_2D_Line *line_v);
    int has_points(P_Sur_2D_Point *point_a, P_Sur_2D_Point *point_b = 0);
};

class P_Sur_2D_Triangle{
public:
    P_Sur_2D_Triangle *next;
    P_Sur_2D_Triangle *prev;
    
    int point_nums[3]; // order of point conn.: clockwise
    
    P_Sur_2D_Triangle(int pn0, int pn1, int pn2);
};

class P_Sur_2D{
    // one can reach all points of the surface by checking
    // all start_points of the lines list!
public:
    IVP_Object_Polygon_Tetra *orig_tetras;
    IVP_Template_Surface *orig_surface;

    // calc section
    P_List<P_Sur_2D_Line> lines; // line representation
    P_List<P_Sur_2D_Triangle> triangles; // full triangle representation
    P_Sur_2D_Line **line_array; // bookmarks for posterior deletion
    P_Sur_2D_Point **point_array; // bookmarks for posterior deletion
    
    P_Sur_2D(IVP_Object_Polygon_Tetra *tetras, IVP_Template_Surface *sur);
    ~P_Sur_2D();

    IVP_ERROR_STRING calc_line_representation(); // could be done in constructor
    IVP_ERROR_STRING calc_triangle_representation(); // -"-
};










