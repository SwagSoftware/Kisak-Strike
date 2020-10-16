// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#if defined(LINUX)
#	include <memory.h>
#endif

#include <ivu_memory.hxx>

#ifndef WIN32
#	pragma implementation "ivp_gridbuild_array.hxx"
#endif

#include <ivp_compact_grid.hxx>
#include <ivp_gridbuild_array.hxx>

#include <ivp_cache_object.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_compact_ledge_solver.hxx>


IVP_Template_Compact_Grid::IVP_Template_Compact_Grid(){
	P_MEM_CLEAR(this);
}

IVP_GridBuilder_Array::IVP_GridBuilder_Array(IVP_U_Memory *mm_, const IVP_Template_Compact_Grid *gp, IVP_FLOAT *height_field_)
{
    P_MEM_CLEAR(this);
    mm = mm_;
    this->height_field = height_field_;
    n_rows = gp->row_info.n_points;
    n_cols = gp->column_info.n_points;

    n_compact_poly_points_used = 0;
    height_points = (IVP_Compact_Poly_Point *)mm->get_mem(sizeof(IVP_Compact_Poly_Point) * n_rows * n_cols);
	unsigned int buffer_size=sizeof(IVP_Compact_Poly_Point) * n_rows * n_cols * 3;
    compact_poly_point_buffer = (IVP_Compact_Poly_Point *)mm->get_mem(buffer_size); // two extra points per square worst case
    ledge_reference_field = (IVP_Compact_Grid_Element *)mm->get_memc(sizeof(IVP_Compact_Grid_Element) * n_rows * n_cols); // last row and col not used

    grid_point_to_ledge_point_array = (int *) mm->get_mem(sizeof(int) * n_rows * n_cols);

    for (int j= n_rows * n_cols -1; j>=0;j--){
	grid_point_to_ledge_point_array[j] = -1;
    }
    
    IVP_U_Float_Point *dest = &this->height_points[0];
    IVP_FLOAT *source = &this->height_field[0];
    IVP_DOUBLE x = gp->position_origin_os.k[ gp->row_info.maps_to ];

    IVP_DOUBLE dx = (gp->row_info.invert_axis) ? -gp->grid_field_size:gp->grid_field_size;
    IVP_DOUBLE dy = (gp->column_info.invert_axis) ? -gp->grid_field_size:gp->grid_field_size;
    IVP_DOUBLE dz = (gp->height_invert_axis) ? -1.0f : 1.0f;
    for (int r=0; r<n_rows; r++) {
	IVP_DOUBLE y = gp->position_origin_os.k[ gp->column_info.maps_to ];
	for (int c=0; c<n_cols; c++) {
	    dest->k[ gp->row_info.maps_to ]    = (IVP_FLOAT)x;
	    dest->k[ gp->column_info.maps_to ] = (IVP_FLOAT)y;
	    dest->k[ gp->height_maps_to ]      = float(source[0] * dz);
	    source++;
	    dest++;
		IVP_ASSERT( ((intptr_t)dest & 15) == 0 );
	    y += dy;
	}
	x += dx;
    }
    templ = gp;	
}

int IVP_GridBuilder_Array::install_grid_point(int grid_point_index){
    int index = grid_point_to_ledge_point_array[grid_point_index];
    if (index >=0) return index;
    index = n_compact_poly_points_used++;	
    grid_point_to_ledge_point_array[grid_point_index] = index;
    compact_poly_point_buffer[ index ] = height_points[grid_point_index];
	IVP_IF(1) {
		IVP_Compact_Poly_Point *cpp=&compact_poly_point_buffer[ index ];
		//lwss - x64 fixes
		//int adress=(int)cpp;
		intptr_t adress=(intptr_t)cpp;
		//lwss end
		IVP_ASSERT( (adress & 15)==0 );
	}
    return index;
}

int IVP_GridBuilder_Array::install_point(const IVP_U_Float_Point *point){
    int index = n_compact_poly_points_used++;
	IVP_IF(1) {
		IVP_Compact_Poly_Point *cpp=&compact_poly_point_buffer[ index ];
		//lwss - x64 fixes
		//int adress=(int)cpp;
		intptr_t adress=(intptr_t)cpp;
		//lwss end
		IVP_ASSERT( (adress & 15)==0 );
	}
    compact_poly_point_buffer[ index ].set(point);
    compact_poly_point_buffer[index].hesse_val = 0.0f;
    return index;
}




IVP_GridBuilder_Array::~IVP_GridBuilder_Array(){
    //P_FREE(height_points);  // memory management system used
    //P_FREE(ledge_reference_field);

}


void IVP_GridBuilder_Array::add_triangle(int s_p0_off, int s_p1_off, int s_p2_off,
										 int opp0, int opp1, int opp2)
{
    int p0_off = c_point_to_point_index[s_p0_off];
    int p1_off = c_point_to_point_index[s_p1_off];
    int p2_off = c_point_to_point_index[s_p2_off];
    
    IVP_Compact_Triangle *tri = &((IVP_Compact_Triangle *)this->c_ledge)[this->triangle_count + 1];

    tri->set_tri_index(this->triangle_count++);

    tri->c_three_edges[0].set_start_point_index(p0_off);
    tri->c_three_edges[1].set_start_point_index(p1_off);
    tri->c_three_edges[2].set_start_point_index(p2_off);

    tri->c_three_edges[0].set_opposite_index(opp0);
    tri->c_three_edges[1].set_opposite_index(opp1);
    tri->c_three_edges[2].set_opposite_index(opp2);

    IVP_IF(1){
	// check for opposite and convexity consistency
	int opp[3];
	opp[0] = opp0;
	opp[1] = opp1;
	opp[2] = opp2;

	for(int e=0; e<3; e++){
	    const IVP_Compact_Edge *edge, *oppo;
	    edge = &tri->c_three_edges[e];
	    oppo = edge->get_opposite();

	    if((opp[e] < 0)&&(opp[e] > -1000) && (oppo->get_opposite_index() > -1000)){
		IVP_ASSERT(edge==oppo->get_opposite());

		// check for convexity
		IVP_U_Hesse hesse;
		IVP_U_Point p0, p1, p2;
		p0.set(edge->get_start_point(c_ledge));
		p1.set(edge->get_next()->get_start_point(c_ledge));
		p2.set(edge->get_prev()->get_start_point(c_ledge));

		hesse.calc_hesse(&p0, &p1, &p2);
		IVP_U_Point pp;
		pp.set(oppo->get_prev()->get_start_point(c_ledge));
		IVP_ASSERT(hesse.get_dist(&pp) >= -0.001f);//- P_RES_EPS);
	    }
	}
    }
}

void IVP_GridBuilder_Array::insert_opposite_index( const IVP_Compact_Edge *edge, int index){
    ((IVP_Compact_Edge *)edge)->set_opposite_index(index);
    
    IVP_IF(1){
	const IVP_Compact_Edge *oppo;
	oppo = edge->get_opposite();
	if (oppo->get_triangle() - c_ledge->get_first_triangle() >= triangle_count) return;	// not in list yet
//	if((index < 0)&&(index > -1000) && (oppo->get_opposite_index() > -1000)){
	    IVP_ASSERT(edge==oppo->get_opposite());

	    // check for convexity
	    IVP_U_Hesse hesse;
	    IVP_U_Point p0, p1, p2;
	    p0.set(edge->get_start_point(c_ledge));
	    p1.set(edge->get_next()->get_start_point(c_ledge));
	    p2.set(edge->get_prev()->get_start_point(c_ledge));

	    hesse.calc_hesse(&p0, &p1, &p2);
	    IVP_U_Point pp;
	    pp.set(oppo->get_prev()->get_start_point(c_ledge));
	    IVP_ASSERT(hesse.get_dist(&pp) >= -1e-12f);
//	}
    }
}


IVP_Compact_Ledge *IVP_GridBuilder_Array::convert_convex_triangle_to_compact_ledge( int strip_points[] ){

    int num_triangles = 2;
    //const int n_points = 3;
    triangle_count= 0;

    // get memory for Ledge Header, Triangles and Points
    int mem_size = 	sizeof(IVP_Compact_Ledge) +	num_triangles * sizeof(IVP_Compact_Triangle);

    this->c_ledge = (IVP_Compact_Ledge *)mm->get_memc( mem_size);
    IVP_ASSERT( (intptr_t(this->c_ledge) & 15) == 0);	// make sure it is aligned

    { // set point arrays
	// search the minimum point index and install points
	int min = IVP_GRID_MAX_ROWS * IVP_GRID_MAX_ROWS * 3;	// super high number
	int max = 0;
	for (int i=2;i>=0;i--){
	    int index = install_grid_point(strip_points[i]);
	    if (index < min) min = index;
	    if (index > max) max = index;
	    c_point_to_point_index[i] = index;
	}
	// shift start so min gets zero
	c_point_to_point_index[0] -= min;
	c_point_to_point_index[1] -= min;
	c_point_to_point_index[2] -= min;

	IVP_ASSERT(max - min + 1 < n_cols * 4 + 2 * n_cols);
	this->c_points = &compact_poly_point_buffer[min];

	c_ledge->set_offset_ledge_points( (int)((uchar *)c_points - (uchar *)c_ledge) ); // byte offset from 'this' to (ledge) point array
    }

    c_ledge->set_is_compact( IVP_FALSE);
    c_ledge->set_size(mem_size);	// <0 indicates a non compact compact ledge 
    c_ledge->n_triangles = num_triangles;
    c_ledge->has_chilren_flag = IVP_FALSE;

    // triangles
    this->add_triangle(0, 1, 2,   6, 4, 2);
    this->add_triangle(0, 2, 1,   -2, -4, -6);

    // pierce info
    IVP_Compact_Triangle *tri = c_ledge->get_first_triangle();
    tri->set_pierce_index(1);
    tri->get_next_tri()->set_pierce_index(0);

#ifdef DEBUG
	IVP_Compact_Ledge_Solver::check_ledge(c_ledge);
#endif

    return this->c_ledge;
}

IVP_Compact_Ledge *IVP_GridBuilder_Array::convert_convex_square_to_compact_ledge( int strip_points[], IVP_BOOL is_right_starter){

    int num_triangles = 4;
    const int n_points = 4;
    triangle_count= 0;

    // get memory for Ledge Header, Triangles and Points
    int mem_size = 	sizeof(IVP_Compact_Ledge) +	num_triangles * sizeof(IVP_Compact_Triangle);

    this->c_ledge = (IVP_Compact_Ledge *)mm->get_memc( mem_size);
    IVP_ASSERT( (intptr_t(this->c_ledge) & 15) == 0);	// make sure it is aligned

    { // set point arrays
	// search the minimum point index and install points
	int min = IVP_GRID_MAX_ROWS * IVP_GRID_MAX_ROWS * 3;	// super high number
	int max = 0;
	for (int i= n_points-1 ;i>=0;i--){
		int index = install_grid_point(strip_points[i]);
		if (index < min) min = index;
		if (index > max) max = index;
		c_point_to_point_index[i] = index;
	}
	// shift start so min gets zero
	c_point_to_point_index[0] -= min;
	c_point_to_point_index[1] -= min;
	c_point_to_point_index[2] -= min;
	c_point_to_point_index[3] -= min;

	IVP_ASSERT(max - min + 1 < n_cols * 4 + 2 * n_cols);
	this->c_points = &compact_poly_point_buffer[min];

	c_ledge->set_offset_ledge_points( (int)((uchar *)c_points - (uchar *)c_ledge) ); // byte offset from 'this' to (ledge) point array
    }
    c_ledge->set_is_compact( IVP_FALSE);
    c_ledge->set_size(mem_size);	
    c_ledge->n_triangles = num_triangles;
    c_ledge->has_chilren_flag = IVP_FALSE;


    // triangles
    int p1 = 1;
    int p2 = 2;
    if ( is_right_starter ){	// swap points 1 and 2
	p1 = 2;
	p2 = 1;
    }

    this->add_triangle(p2, p1, 0,   4, 8, 12);
    this->add_triangle(p1, p2, 3,   -4, 8, 4);
    this->add_triangle( 3, 0, p1,   4, -8, -4);
    this->add_triangle( 0, 3, p2,   -4, -8, -12);

    // pierce info
    IVP_Compact_Triangle *tri;
    tri = c_ledge->get_first_triangle();		tri->set_pierce_index(2);
    tri = tri->get_next_tri();					tri->set_pierce_index(2);
    tri = tri->get_next_tri();					tri->set_pierce_index(0);
    tri = tri->get_next_tri();					tri->set_pierce_index(0);

#ifdef DEBUG
	IVP_Compact_Ledge_Solver::check_ledge(c_ledge);
#endif

    return this->c_ledge;
}



// -1 , -2 are reference points
IVP_Compact_Poly_Point *IVP_GridBuilder_Array::get_point_at_strip_pos(int pos){
    int point_index = c_point_to_point_index[pos+2];
    return &c_points[point_index];
}



IVP_Compact_Ledge *IVP_GridBuilder_Array::convert_convex_stripe_to_compact_ledge_fast(
						int strip_points[],		int n_points,
						IVP_U_Float_Point ref_point[2],
						IVP_BOOL is_right_starter)
{
    ///// Convert a convex strip into a compact ledge
    IVP_ASSERT(n_points >=4);


    //// INITS
    //// INITS
    int num_triangles = 4 + 2*(n_points-2);
    this->triangle_count = 0;


    // get memory for Ledge Header, Triangles and Points
    int mem_size = 	sizeof(IVP_Compact_Ledge) +	num_triangles * sizeof(IVP_Compact_Triangle);

    this->c_ledge = (IVP_Compact_Ledge *)mm->get_memc( mem_size);
    IVP_ASSERT ( (intptr_t(c_ledge) & 15) == 0);



    { // set point arrays
	// search the minimum point index and install points
	int min = c_point_to_point_index[0] = install_point(&ref_point[0]);
	int max = c_point_to_point_index[1] = install_point(&ref_point[1]);
	if (min > max){ int h = max; max = min; min = h; }; // exchange points
	for (int i=n_points-1;i>=0;i--){
	    int index = install_grid_point(strip_points[i]);
	    if (index < min) min = index;
	    if (index > max) max = index;
	    c_point_to_point_index[i+2] = index;
	}
	// shift start so min gets zero
	for (int j = n_points +1; j>=0; j--){
	    c_point_to_point_index[j] -= min;
	}
	IVP_ASSERT(max - min + 1 < n_cols * 4 + 1);
	this->c_points = &compact_poly_point_buffer[min];
	c_ledge->set_offset_ledge_points( (uchar *)c_points - (uchar *)c_ledge ); // byte offset from 'this' to (ledge) point array
    }
    
    c_ledge->set_is_compact( IVP_FALSE);
    c_ledge->set_size(mem_size);		// if size < 0 than compact_edge is not compact and shares point arrays
    c_ledge->n_triangles = num_triangles;
    c_ledge->has_chilren_flag = IVP_FALSE;


    int cp = 2; // init current point index counter
    int end_point_idx = n_points-1; // used to determine completion
    int last_opp = 0; // used to link last with first triangle
    int left_off, mid_off, right_off; // used to override the upper edge opposite indices

    //// START TRIANGLES
    //// START TRIANGLES
    {
	IVP_U_Hesse hesse;
	hesse.calc_hesse(	get_point_at_strip_pos(-1), get_point_at_strip_pos(-2), get_point_at_strip_pos(0));
	IVP_DOUBLE dist = hesse.get_dist( get_point_at_strip_pos(1) );

	if(is_right_starter){
	    /// right starter

	    // figure out the convex way to connect start edge with ref points
	    if(dist>0.0f){	// sign.
		// make triangles
		this->add_triangle(3, 2, 0,		8, 5, 19);
		this->add_triangle(2, 1, 0,		10, -1000, -5); // -1000 is replaced later
		left_off = -19;
		mid_off  = -8;
		right_off= -10;
	    }else{
		// make triangles
		this->add_triangle(3, 2, 1,		8, 13, 2);
		this->add_triangle(3, 1, 0,		-2, -1000, 15); // -1000 is replaced later
		left_off = -15;
		mid_off  = -8;
		right_off= -13;
	    }

	    // skip to regular lower right proc
	    goto lower_right;
	}else{
	    /// left starter
	    // figure out the convex way to connect start edge with ref points
	    if(dist<0.0f){	// sign.
		// make triangles p1,r0,p0 and p1,r1,r0
		this->add_triangle(3, 0, 2,   6, 12, 6);
		this->add_triangle(3, 1, 0,   18, -1000,-6); // -1000 is replaced later
		left_off = -12;
		mid_off  = -6;
		right_off= -18;
	    }else{
		// make triangles p0,p1,r1 and p0,r1,r0
		this->add_triangle(2, 3, 1,    8, 21, 2);
		this->add_triangle(2, 1, 0,    -2, -1000, 7); // -1000 is replaced later
		left_off = -7;
		mid_off  = -8;
		right_off= -21;
	    }
	    // continue with regular upper left proc
	}
    }
    //// REGULAR LOOP
    //// REGULAR LOOP
    while(1){
	    /// upper left plus left neighbor
	    if(cp<end_point_idx){
		    // regular
		    this->add_triangle(cp+1, cp, cp+2, 	   mid_off, 3, 6);
		    mid_off = -6;
		    this->add_triangle(cp+2, cp, 0, 	   -3, left_off, 15);
		    left_off = -15;
		    cp++;
	    }else{
		    /// last one

		    // figure out the convex way to connect last edge with ref points
		    IVP_U_Hesse hesse;
		    hesse.calc_hesse(get_point_at_strip_pos(-2), get_point_at_strip_pos(-1), get_point_at_strip_pos(cp+1-2));
		    IVP_DOUBLE dist = hesse.get_dist(get_point_at_strip_pos(cp /*+2-2*/) );
		    if(dist<0.0f){ // sign.
			    const IVP_Compact_Triangle *tri = &c_ledge->get_first_triangle()[this->triangle_count-1];
			    const IVP_Compact_Edge *edge = tri->get_edge(1);
			    insert_opposite_index(edge,13);

			    this->add_triangle(cp+1, cp,   cp+2,   mid_off, 3, 6);
			    this->add_triangle(cp+2, cp,   0, 	   -3, left_off, 7);
			    this->add_triangle(cp+1, cp+2, 1, 	   -6, 3, -13);

			    last_opp = -1-(num_triangles-2)*4;
			    this->add_triangle(1,    cp+2, 0,      -3, -7, last_opp);
		    }else{
			    const IVP_Compact_Triangle *tri = &c_ledge->get_first_triangle()[this->triangle_count-1];
			    const IVP_Compact_Edge *edge = tri->get_edge(1);
			    insert_opposite_index(edge,17);

			    this->add_triangle(cp+1, cp, cp+2,   mid_off, 3, 6);
			    this->add_triangle(cp+2, cp,   0, 	   -3, left_off, 3);
			    this->add_triangle(cp+1, cp+2, 0, 	   -6, -3, 2);

			    last_opp = -(num_triangles-2)*4;
			    this->add_triangle(cp+1, 0,	   1, 	   -2, last_opp, -17);
		    }
		    break;
	    }
lower_right:
		/// lower right plus right neighbor
		if(cp<end_point_idx){
			// regular
			this->add_triangle(cp, cp+1, cp+2, 	   mid_off, 7, 2);
			mid_off = -7;
			this->add_triangle(cp, cp+2, 1, 	   -2, 17, right_off);
			right_off = -17;
			cp++;
		}else{
			/// last one
			// figure out the convex way to connect last edge with ref points
			IVP_U_Hesse hesse;
			hesse.calc_hesse(get_point_at_strip_pos(-2), get_point_at_strip_pos(-1), get_point_at_strip_pos(cp+1-2));
			IVP_DOUBLE dist = hesse.get_dist(get_point_at_strip_pos(cp /*+2-2*/ ));
			if(dist>0.0f){ // sign.
				const IVP_Compact_Triangle *tri = &c_ledge->get_first_triangle()[this->triangle_count-1];
				const IVP_Compact_Edge *edge = tri->get_edge(2);
				insert_opposite_index(edge, 15);

				this->add_triangle(cp,   cp+1, cp+2,	mid_off, 7, 2);
				this->add_triangle(cp,   cp+2, 1,		-2, 5, right_off);
				this->add_triangle(cp+2, cp+1, 1,		-7, 3, -5);

				last_opp = -1-(num_triangles-2)*4;
				this->add_triangle(1,    cp+1, 0,		-3, -15, last_opp);
			}else{
				IVP_Compact_Triangle *tri = &c_ledge->get_first_triangle()[this->triangle_count-1];
				const IVP_Compact_Edge *edge = tri->get_edge(2);
				insert_opposite_index(edge, 11);

				this->add_triangle(cp,   cp+1, cp+2,	mid_off, 7, 2);
				this->add_triangle(cp,   cp+2, 1,		-2, 9, right_off);
				this->add_triangle(cp+2, cp+1, 0,		-7, -11, 2);

				last_opp = -(num_triangles-2)*4;
				this->add_triangle(cp+2, 0,    1,		-2, last_opp, -9);
			}
			break;
		}
	} // loop

	// link last with first triangle
	{
		IVP_ASSERT(last_opp);
		const IVP_Compact_Triangle *tri = &this->c_ledge->get_first_triangle()[1];
		IVP_ASSERT(tri->get_tri_index() == 1);
		const IVP_Compact_Edge *link_edge = tri->get_edge(1);
		insert_opposite_index(link_edge, -last_opp);
	}

	//// INSERT PIERCING INFO
	//// INSERT PIERCING INFO
	{
		// simple, square worst case (unlikely)
		int found_cnt = 0;
		IVP_DOUBLE take_it_eps = 0.01f;

		const IVP_Compact_Triangle *tri = this->c_ledge->get_first_triangle();
		for(int t=num_triangles-1; t>=0; t--, tri = tri->get_next_tri()){
			IVP_U_Point norm_vec;
			norm_vec.inline_set_vert_to_area_defined_by_three_points(
				tri->get_edge(0)->get_start_point(c_ledge),
				tri->get_edge(1)->get_start_point(c_ledge),
				tri->get_edge(2)->get_start_point(c_ledge) );

			IVP_DOUBLE max_h = -1.0f;
			int	max_tri_num = -1;
			const IVP_Compact_Triangle *tri2 = this->c_ledge->get_first_triangle();
			for(int u=num_triangles-1; u>=0; u--, tri2 = tri2->get_next_tri()){
				if(t==u) continue;
				IVP_U_Point norm_vec2;
				norm_vec2.inline_set_vert_to_area_defined_by_three_points(
					tri2->get_edge(0)->get_start_point(c_ledge),
					tri2->get_edge(1)->get_start_point(c_ledge),
					tri2->get_edge(2)->get_start_point(c_ledge) );

				// check if the normal are looking in different directions 
				IVP_DOUBLE h = norm_vec.dot_product(&norm_vec2);
				if(h > take_it_eps){
					// found
					((IVP_Compact_Triangle *)tri)->set_pierce_index(u);
					found_cnt++;
					goto found_a_tri; // sorry for gotos, want to avoid extra if's
				}else{
					if(h>max_h){
						max_h = h;
						max_tri_num = u;
					}
				}
			} // for u
			IVP_ASSERT(max_tri_num >= 0);
			((IVP_Compact_Triangle *)tri)->set_pierce_index(max_tri_num);
			found_cnt++;				
found_a_tri:
			;
		} // for t
		IVP_ASSERT(found_cnt == num_triangles);
	}

#ifdef DEBUG
	IVP_Compact_Ledge_Solver::check_ledge(c_ledge);
#endif

	return this->c_ledge;
}

/* scan each strip for convex parts, skip flat areas */
void IVP_GridBuilder_Array::convert_strip_to_compact_ledges(int row, IVP_U_Vector<IVP_Compact_Ledge> *ledges){

    int strip_field_offset[IVP_GRID_MAX_COLUMNS * 2];		// jumps alternately between the two rows of points defining row
    int dest_stripe[IVP_GRID_MAX_COLUMNS * 2];		// the compacted strip to be later translated into compact ledge

    {	// build the stripe
	int low = row * n_cols;
	int high = low + n_cols;
	int d = 0;
	for (int c = 0; c < n_cols; c++){
	    strip_field_offset[d] = low++;
	    strip_field_offset[d+1] = high++;
	    d+=2;
	}
    }	

    int strip_size = n_cols + n_cols;

    int start_pos; // start of stripe
    int end_pos; // end of convex stripe
    int dest_index;
	

    for (start_pos = 0; start_pos< strip_size-2; start_pos = end_pos-1){
	dest_index = 0;
	int first_point = start_pos;
	dest_stripe[dest_index] = strip_field_offset[start_pos];	// insert first line
	dest_stripe[dest_index+1] = strip_field_offset[start_pos+1];
	dest_index+=2;
	start_pos+=2;
	// search end pos

	int ep;		// will be the first point (behind start_pos) which will be needed
	check_flat_triangles:
	for ( ep = start_pos; ep < strip_size-3; ep+=2){
	    int p;
	    // check for at least three consequtive parallel triangles
	    for ( p = ep; p< ep+3 && p < strip_size-1 ; p++){
		IVP_DOUBLE diff = height_field [ strip_field_offset[p+0] ] + height_field [ strip_field_offset[p-1] ] -
				      height_field [ strip_field_offset[p-2] ] - height_field [ strip_field_offset[p+1] ];
		if (diff < 0.0f || diff > 1e-3f){
		    p = -1;
		    break;
		}
	    }
	    if (p == -1)	break; // no three flat triangles found
	}
	// now ep is the first point needed
	dest_stripe[dest_index++] = strip_field_offset[ep];	// finalize triangle

	if ( ep < strip_size-1 ){	// check the next triangle for convex piece
	    IVP_DOUBLE diff = height_field [ strip_field_offset[ep+0] ] + height_field [ strip_field_offset[ep-1] ] -
				      height_field [ strip_field_offset[ep-2] ] - height_field [ strip_field_offset[ep+1] ];
	    if (diff >= 0.0f){		// convex edge
		start_pos = ep+1;
		goto check_flat_triangles;
	    }
	}
	end_pos = ep;	// includes ep
	IVP_Compact_Ledge *cl;

	while(1){
	    if ( dest_index == 3){
		cl=convert_convex_triangle_to_compact_ledge( &dest_stripe[0]);
		break;
	    }
	    IVP_BOOL is_right_starter;
	    if ((strip_field_offset[first_point] < strip_field_offset[first_point+1]) ^ (is_left_handed == IVP_TRUE)){
		is_right_starter = IVP_FALSE;
	    }else{
		is_right_starter = IVP_TRUE;
	    }
	    
	    if (dest_index == 4){
		cl = convert_convex_square_to_compact_ledge( &dest_stripe[0], is_right_starter);
		break;
	    }
	    { //else
		IVP_U_Float_Point ref_points[2];
		{ // search for minimum value
		    int p0a_index = dest_stripe[0];
		    int p1a_index = dest_stripe[1];
		    int p0b_index = dest_stripe[(dest_index-1)& ~1];
		    int p1b_index = dest_stripe[((dest_index-2)& ~1) +1];

		    IVP_U_Float_Point *p0a = &height_points[p0a_index];
		    IVP_U_Float_Point *p0b = &height_points[p0b_index];
		    IVP_U_Float_Point *p1a = &height_points[p1a_index];
		    IVP_U_Float_Point *p1b = &height_points[p1b_index];

		    ref_points[0].set(p0a);
		    ref_points[1].set(p1a);
		    IVP_FLOAT col_pos = 0.5f * (p0a->k[ templ->column_info.maps_to ] + p0b->k[ templ->column_info.maps_to ]);
		    ref_points[0].k[ templ->column_info.maps_to ] = col_pos;
		    ref_points[1].k[ templ->column_info.maps_to ] = col_pos;

		    IVP_FLOAT dz = (templ->height_invert_axis)? -1.0f: 1.0f;
		    // search min value of z axis
		    IVP_FLOAT minz, z;
		    minz = p0a->k[ templ->height_maps_to ] * dz;
		       z = p0b->k[ templ->height_maps_to ] * dz; if (z < minz) minz=z;
		       z = p1a->k[ templ->height_maps_to ] * dz; if (z < minz) minz=z;
		       z = p1b->k[ templ->height_maps_to ] * dz; if (z < minz) minz=z;
		    minz -= templ->grid_field_size;
		    minz *= dz;
		    ref_points[0].k[ templ->height_maps_to ] = minz;
		    ref_points[1].k[ templ->height_maps_to ] = minz;
		}

		if(is_right_starter){
		    IVP_U_Point hp;
		    hp.set(&ref_points[0]);
		    ref_points[0].set(&ref_points[1]);
		    ref_points[1].set(&hp);
		}

		cl=convert_convex_stripe_to_compact_ledge_fast( &dest_stripe[0], dest_index, ref_points, is_right_starter );
#ifdef DEBUG
		IVP_Compact_Ledge_Solver::check_ledge(cl);
#endif
		break;
	    }
	    break;
	}
	int ledge_index = ledges->len();
	ledges->add(cl);
	// fill in the reference indizes
	for ( int p = first_point; p < ep-1 /* == end_pos-1*/; p++){
	    int a_offset = strip_field_offset[p];
	    int b_offset = strip_field_offset[p+1];
	    if (a_offset < b_offset) {	// forward strip
		ledge_reference_field[a_offset].compact_ledge_index[0] = ledge_index;
	    }else{			// backward
		ledge_reference_field[b_offset-1].compact_ledge_index[1] = ledge_index;
	    }
	}
    }
}


/* split the array into strips */
void IVP_GridBuilder_Array::convert_array_to_compact_ledges( const IVP_Template_Compact_Grid *gp, IVP_U_Vector<IVP_Compact_Ledge> *ledges){

    IVP_U_Matrix m_object_f_grid;
    {
	IVP_U_Point col_axle; col_axle.set_to_zero();
	IVP_U_Point row_axle; row_axle.set_to_zero();
	IVP_U_Point h_axle;   h_axle.set_to_zero();

	IVP_DOUBLE dx = (gp->row_info.invert_axis) ? -gp->grid_field_size:gp->grid_field_size;
	IVP_DOUBLE dy = (gp->column_info.invert_axis) ? -gp->grid_field_size:gp->grid_field_size;
	IVP_DOUBLE dz = (gp->height_invert_axis) ? -1.0f:1.0f;

	row_axle.k[ gp->row_info.maps_to]    = dx;
	col_axle.k[ gp->column_info.maps_to] = dy;
	h_axle.k[gp->height_maps_to]         = dz;

	m_object_f_grid.get_position()->set(&gp->position_origin_os);
	m_object_f_grid.init_columns3(&row_axle, &col_axle, &h_axle);

	IVP_DOUBLE det = m_object_f_grid.get_determinante();
	if (det > 0.0f){
	    is_left_handed = IVP_TRUE;
	}else{
	    is_left_handed = IVP_FALSE;
	}
    }

    for (int r=0;r< n_rows-1;r++){
	convert_strip_to_compact_ledges(r, ledges);
    }
}


/* merge everything into the final compact grid thing */
IVP_Compact_Grid *IVP_GridBuilder_Array::compile_ledges_into_compact_grid(const IVP_Template_Compact_Grid *gp,IVP_U_Vector<IVP_Compact_Ledge> *ledges){
	IVP_Compact_Grid *cg = NULL;
	int buffer_size = (char *)&cg->offset_compact_ledge_array[0] - (char *)cg;	// size for base compact ledge

	buffer_size += ledges->len() * sizeof(int); // add buffersize for ledge index array
	buffer_size += (n_rows-1) * (n_cols-1) * sizeof(IVP_Compact_Grid_Element);	 //the grid referencing ledge index array
	buffer_size += n_compact_poly_points_used * sizeof(IVP_Compact_Poly_Point);
	buffer_size = (buffer_size + 15) & ~15;	// align compact ledges

	for ( int i = ledges->len()-1; i>=0;i--){	// add buffersize for ledges
		IVP_Compact_Ledge *cl = ledges->element_at(i);
		int l_size = cl->get_size();
		IVP_ASSERT( ((l_size) & 0xf) == 0);
		buffer_size += l_size;
	}

	cg = (IVP_Compact_Grid *)ivp_malloc_aligned(buffer_size+4,16);
	cg->byte_size = buffer_size;

	cg->n_rows = n_rows-1;
	cg->n_columns = n_cols-1;
	cg->n_compact_ledges= ledges->len();

	IVP_U_Matrix m_object_f_grid;
	{
	    IVP_U_Point col_axle; col_axle.set_to_zero();
	    IVP_U_Point row_axle; row_axle.set_to_zero();
	    IVP_U_Point h_axle;   h_axle.set_to_zero();

	    IVP_DOUBLE dx = (gp->row_info.invert_axis) ? -gp->grid_field_size:gp->grid_field_size;
	    IVP_DOUBLE dy = (gp->column_info.invert_axis) ? -gp->grid_field_size:gp->grid_field_size;
	    IVP_DOUBLE dz = (gp->height_invert_axis) ? -1.0f:1.0f;

	    row_axle.k[ gp->row_info.maps_to]    = dx;
	    col_axle.k[ gp->column_info.maps_to] = dy;
	    h_axle.k[gp->height_maps_to]         = dz;

	    m_object_f_grid.get_position()->set(&gp->position_origin_os);
	    m_object_f_grid.init_columns3(&row_axle, &col_axle, &h_axle);

	    cg->m_grid_f_object.real_invert(&m_object_f_grid);
	}
	{
	    IVP_DOUBLE det = m_object_f_grid.get_determinante();
	    if (det > 0.0f){
		is_left_handed = IVP_TRUE;
	    }else{
		is_left_handed = IVP_FALSE;
	    }
	}
	IVP_DOUBLE max_delta_height_os;
	{ // find center, use bounding box
	    IVP_FLOAT max, min;
	    max = min = height_field[0];
	    for (int x= n_cols * n_rows -1;x>=0; x--){
		if (height_field[x] > max) max = height_field[x];
		if (height_field[x] < min) min = height_field[x];
	    }
	    IVP_U_Float_Point center_gs;
	    center_gs.set( (n_rows-1) * 0.5f, (n_cols-1) * 0.5f, (max + min) * 0.5f );

	    m_object_f_grid.vmult4( &center_gs, &cg->center );
	    max_delta_height_os = (max - min ) * 0.5f;
	}

	IVP_U_Point center; center.set(&cg->center);
	{ // find radius, loop through all used 
	    IVP_DOUBLE qrad;
	    qrad = center.quad_distance_to(&compact_poly_point_buffer[0]);

	    for ( int i = n_compact_poly_points_used-1; i>0; i--){
		IVP_DOUBLE 	qr = center.quad_distance_to( & compact_poly_point_buffer[i]);
		if (qr > qrad) qrad = qr;
	    }
	    cg->radius = (IVP_FLOAT)IVP_Inline_Math::sqrtd(qrad);
	}
	cg->inv_grid_size = 1.0f / gp->grid_field_size;

	// reserve space for the grid elements
	cg->offset_grid_elements = ((char *)(&cg->offset_compact_ledge_array[0])) - ((char *)cg) + sizeof(int) * ledges->len(); // skip the ledge offsets
	
	// TL: align offset to next 16 Byte Alignment (I didn't write the GridBuilder, so I have no idea wether the memory will be enough, lets hope ... - the SUN-Debugger will find it)
	int aligned_offset=cg->offset_grid_elements+15;
	aligned_offset=aligned_offset & 0xfffffff0;
    cg->offset_grid_elements=aligned_offset;

	{ // fill in the compact ledges

		char *dest = ((char *)cg) + cg->offset_grid_elements + (n_rows-1) * (n_cols-1) * sizeof(IVP_Compact_Grid_Element);


		char *compact_points = dest;
		{ // all points
			int size_of_points = sizeof(IVP_Compact_Poly_Point) * n_compact_poly_points_used;
			memcpy(compact_points, (char *)compact_poly_point_buffer, size_of_points);
			dest += size_of_points;
		}

		// and the ledges
		dest = (char *)((long(dest)+0xf) & ~0xf);     	// align destination
		for (int ledge_index = 0; ledge_index < ledges->len(); ledge_index++){
			IVP_Compact_Ledge *cl;
			cl = ledges->element_at(ledge_index);
			int l_size = cl->get_size();
			memcpy(dest, (char *)cl, l_size);

			IVP_Compact_Ledge *cl_dest = (IVP_Compact_Ledge *)dest;
			cl_dest->set_offset_ledge_points( (char *)cl->get_point_array() - (char *)compact_poly_point_buffer + compact_points - (char *)cl_dest );

			IVP_ASSERT( cl_dest->get_point_array()->quad_distance_to(cl->get_point_array()) == 0.0f);

			cg->offset_compact_ledge_array[ ledge_index ] = dest - (char *)cg;
			dest += l_size;
		}
		ledges->remove_all();
		IVP_ASSERT( (char *)dest - (char *)cg == cg->byte_size );
	}


	{ // fill in the grid referencing offset_compact_ledge_array
	    IVP_Compact_Grid_Element *ge = (IVP_Compact_Grid_Element *)cg->get_grid_elements();	// discard const
		IVP_Compact_Grid_Element *source = &ledge_reference_field[0];

		for (int r = 1; r < n_rows; r++){
			for (int c=1; c < n_cols; c++){
				*ge = *source;
				ge++;
				source++;
			}
			source++;	// skip extra unused row
		}
	}



	return cg;
}
 

IVP_Compact_Grid  *IVP_GridBuilder_Array::convert_array_to_compact_grid(IVP_Environment *env, const IVP_Template_Compact_Grid *gp, IVP_FLOAT *height_field){
	
    IVP_U_Memory	*mm = env->get_memory_manager();
	mm->start_memory_transaction();

	IVP_GridBuilder_Array builder(mm, gp, height_field);


	IVP_U_Vector<IVP_Compact_Ledge> ledges(1024);
	
	builder.convert_array_to_compact_ledges(gp, &ledges);

	IVP_Compact_Grid *cg = builder.compile_ledges_into_compact_grid(gp, &ledges);

	mm->end_memory_transaction();
	return cg;

}

