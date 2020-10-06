// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	File:	       	ivp_compact_ledge.cxx	
 *	Description:	compacted ledge representation
 ********************************************************************************/

//#include <string.h>
#include <ivp_physics.hxx>
#include <ivu_hash.hxx>


#ifndef WIN32
#	pragma implementation "ivp_compact_ledge.hxx"
#endif
#include <ivp_compact_ledge.hxx>
#include <ivp_cache_object.hxx>
#include <ivp_cache_ledge_point.hxx>


int IVP_Compact_Edge::next_table[] = { 0, 1*int(sizeof(IVP_Compact_Edge)),  1*int(sizeof(IVP_Compact_Edge)), -2*int(sizeof(IVP_Compact_Edge)) };
int IVP_Compact_Edge::prev_table[] = { 0, 2*int(sizeof(IVP_Compact_Edge)), -1*int(sizeof(IVP_Compact_Edge)), -1*int(sizeof(IVP_Compact_Edge)) };


//////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////

IVP_Compact_Edge::IVP_Compact_Edge()
{
    start_point_index = 0;
    opposite_index = 0;
}

void IVP_Compact_Edge::byte_swap()
{
/* 
	unsigned int start_point_index:16;		// point index
	signed   int opposite_index:15;		// rel to this // maybe extra array, 3 bits more than tri_index/pierce_index
	unsigned int   is_virtual:1;
*/

	// XXX COMPILER SPECIFIC XXX
	uint bitfields = *(uint*)(this); 

#if defined (__POWERPC__)

#elif defined (WIN32)

	uint bitfield0 = (bitfields & 0x0000FFFF) << 16;
	uint bitfield1 = (bitfields & 0x7FFF0000) >> 15;
	uint bitfield2 = (bitfields & 0x80000000) >> 31;

	bitfields = bitfield0 | bitfield1 | bitfield2;

#endif
	
	ivp_byte_swap4( bitfields );

	start_point_index = (bitfields & 0x0FFFF);
	opposite_index = (bitfields & 0x7FFF0000) >> 16;
	is_virtual = (bitfields & 0x80000000) >> 31;

	uint sp = start_point_index;
	int  oi = opposite_index;
	uint isv = is_virtual;
}

///////////////////////////////////////////////////////////////////////////////////////

IVP_Compact_Triangle::IVP_Compact_Triangle()
{
    P_MEM_CLEAR(this);
}


void IVP_Compact_Triangle::byte_swap()
{
 /*
  unsigned int tri_index:12; // used for upward navigation
  unsigned int pierce_index:12;
  unsigned int material_index:7;
  unsigned int is_virtual:1;
  IVP_Compact_Edge c_three_edges[3];
*/

	uint bitfields = *(uint*)(this);

#if defined (__POWERPC__)

#elif defined (WIN32)

	uint bitfield0 = (bitfields & 0x00000FFF) << 20;
	uint bitfield1 = (bitfields & 0x00FFF000) >> 4;
	uint bitfield2 = (bitfields & 0x7F000000) >> 23;
	uint bitfield3 = (bitfields & 0x80000000) >> 31;

	bitfields = bitfield0 | bitfield1 | bitfield2 | bitfield3;

#endif
	
	ivp_byte_swap4( bitfields );

	// XXX VERY POSSIBLY COMPILER SPECIFIC XXX
	// Due to different ordering / packing of bitfields
	tri_index = bitfields & 0x0FFF;
	pierce_index = (bitfields & 0x0FFF000) >> 12 ;
	material_index = (bitfields & 0x7F000000) >> 24 ;
	is_virtual = (bitfields & 0x80000000) >> 31 ;

	uint ti = tri_index;
	uint pi = pierce_index;
	uint mi = material_index;
	uint iv = is_virtual;

	c_three_edges[0].byte_swap();
	c_three_edges[1].byte_swap();
	c_three_edges[2].byte_swap();
}

///////////////////////////////////////////////////////////////////////////////////////

void IVP_Compact_Ledge::c_ledge_init()
{
    P_MEM_CLEAR(this);
}

		
void IVP_Compact_Ledge::byte_swap()
{
/*
	int c_point_offset; // byte offset from 'this' to (ledge) point array
    union {
		int ledgetree_node_offset;
		int client_data;	// if indicates a non terminal ledge
    };
    unsigned int has_chilren_flag:2;
    IVP_BOOL is_compact_flag:2;  // if false than compact ledge uses points outside this piece of memory
    unsigned int dummy:4;
    unsigned int size_div_16:24; 
    short n_triangles;
    short for_future_use;
*/
	
	ivp_byte_swap4( (uint&) c_point_offset );
	ivp_byte_swap4( (uint&) ledgetree_node_offset );
	
	uint bitfields = *(uint*)( &ledgetree_node_offset + 1 ); // as can't get addr of bitfield member..

#if defined (__POWERPC__)

#elif defined (WIN32)

	uint bitfield0 = bitfields << 24;
	uint bitfield00 = bitfield0 & 0x03000000;
	uint bitfield01 = bitfield0 & 0x0c000000;
	uint bitfield02 = bitfield0 & 0xf0000000;
	bitfield0 = (bitfield00 << 6) | (bitfield01 << 2) | (bitfield02 >> 4);

	uint bitfield1 = bitfields >> 8;
	bitfields = bitfield0 | bitfield1;

#endif
	
	ivp_byte_swap4( bitfields );
	
	// XXX VERY POSSIBLY COMPILER SPECIFIC XXX
	// Due to different ordering / packing of bitfields
	has_chilren_flag = (uchar)(bitfields & 0x03);
	is_compact_flag = (IVP_BOOL)( (bitfields & 0x0C) >> 2 );
	dummy = (uchar)(bitfields & 0x0F0) >> 4;
	size_div_16 = (bitfields & 0xFFFFFF00) >> 8;

	uchar hasc = has_chilren_flag;
	uchar ic = is_compact_flag;
	uchar d = dummy;
	uint sd = size_div_16;

	ivp_byte_swap2( (ushort&) n_triangles );
	ivp_byte_swap2( (ushort&) for_future_use );
	
}


static void ProcessPoint( IVP_Compact_Poly_Point& point, IVP_U_BigVector<IVP_Compact_Poly_Point>* pre_swapped_points)
{
	if (pre_swapped_points)
	{
		int j;
		for (j = 0; j < pre_swapped_points->len(); ++j)
		{
			if ( pre_swapped_points->element_at(j) == &point )
				break;
		}
		
		if (j == pre_swapped_points->len())
		{
			pre_swapped_points->add(&point);
			point.byte_swap(); // SWAP
		//	printf("point [ %.4f %.4f %.4f ]\n", point.k[0], point.k[1], point.k[2] );
		}
	}
	else
	{
		point.byte_swap(); // SWAP
	//	printf("point [ %.4f %.4f %.4f ]\n", point.k[0], point.k[1], point.k[2] );
	}			
}
	

void IVP_Compact_Ledge::byte_swap_all(IVP_U_BigVector<IVP_Compact_Poly_Point>* pre_swapped_points)
{
#if defined (__POWERPC__)
	byte_swap();
#endif

	// progress into the data:

	IVP_Compact_Poly_Point* points = get_point_array();
	IVP_Compact_Triangle* triangles = get_first_triangle();

	for ( int j = 0; j < n_triangles; ++j)
	{
#if defined (__POWERPC__)
		triangles[j].byte_swap();
#endif

		const int i0 = triangles[j].get_edge(0)->get_start_point_index();
		const int i1 = triangles[j].get_edge(1)->get_start_point_index();
		const int i2 = triangles[j].get_edge(2)->get_start_point_index();

		ProcessPoint( points[i0], pre_swapped_points);
		ProcessPoint( points[i1], pre_swapped_points);
		ProcessPoint( points[i2], pre_swapped_points);

#if defined (WIN32)
		triangles[j].byte_swap();
#endif
	}

#if defined (WIN32)
	byte_swap();
#endif
}

///////////////////////////////////////////////////////////////////////////////////////

void IVP_Compact_Ledgetree_Node::byte_swap()
{
/*
	int offset_right_node; // (if != 0 than children
    int offset_compact_ledge; //(if != 0, pointer to hull that contains all subelements
    IVP_U_Float_Point3 center;	// in object_coords
    IVP_FLOAT radius; // size of sphere
    uchar box_sizes[IVP_CLT_N_DIRECTIONS];
    uchar free_0;
*/
	ivp_byte_swap4( (uint&) offset_right_node );
	ivp_byte_swap4( (uint&) offset_compact_ledge );
	center.byte_swap();
	ivp_byte_swap4( (uint&) radius );

	// uchars don't need to be byte swapped themselves
}

void IVP_Compact_Ledgetree_Node::byte_swap_all(IVP_U_BigVector<IVP_Compact_Poly_Point>* pre_swapped_points)
{
#if defined (__POWERPC__)
	//
	// swaps offsets and data, so we can recurse and not crash!!
	//
	byte_swap();
#endif

	// recurse
	if (!is_terminal())
	{
		// left:
		const IVP_Compact_Ledgetree_Node* l = left_son(); //returns const only?!
		IVP_ASSERT(l);
		const_cast<IVP_Compact_Ledgetree_Node*>(l)->byte_swap_all(pre_swapped_points);

		// right:
		const IVP_Compact_Ledgetree_Node* r = right_son(); //returns const only?!
		IVP_ASSERT(r);
		const_cast<IVP_Compact_Ledgetree_Node*>(r)->byte_swap_all(pre_swapped_points);
	}

#if defined WIN32
	//
	// just swaps data
	//
	byte_swap();
#endif
}

