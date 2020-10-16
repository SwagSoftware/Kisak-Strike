// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.
// [ CK 2001 ]

#include <ivp_physics.hxx>
#include <ivp_compact_ledge.hxx>
#include <ivp_compact_surface.hxx>

void IVP_Compact_Surface::byte_swap()
{
/*

  IVP_U_Float_Point3	mass_center;
  IVP_U_Float_Point3	rotation_inertia;
  IVP_FLOAT		upper_limit_radius;
  unsigned int	max_factor_surface_deviation:8;
  int			byte_size:24;		// size of whole structure in bytes
  int			offset_ledgetree_root;	// offset to root node of internal ledgetree
  int			dummy[3];		// 16byte memory align
*/
	mass_center.byte_swap(); // 3 floats
	rotation_inertia.byte_swap(); // 3 floats
	ivp_byte_swap4( (uint&) upper_limit_radius ); // 1 float
	ivp_byte_swap4( (uint&) offset_ledgetree_root ); // 1 float
	
	uint bitfields = *(uint*)( &upper_limit_radius + 1 );	

#if defined (__POWERPC__)
	
#elif defined (WIN32)

	uint bitfield0 = bitfields << 24;
	uint bitfield1 = bitfields >> 8;
	bitfields = bitfield0 | bitfield1;

#endif

	ivp_byte_swap4( bitfields );

	//XXX COMPILER SPECIFIC XXX
	max_factor_surface_deviation = (uchar)( bitfields & 0x0FF);
	byte_size = (uint)(( bitfields & 0xFFFFFF00) >> 8);

	uchar sd = max_factor_surface_deviation;
	uint bs = byte_size;
}

void IVP_Compact_Surface::byte_swap_all(IVP_BOOL swap_points, int point_estimate)
{
#if defined (__POWERPC__)
	byte_swap();
#endif

	// Recurse
	const IVP_Compact_Ledgetree_Node* ltn = get_compact_ledge_tree_root();
	if (ltn)
	{
		if (swap_points)
		{
			// IVP_U_BigVector is a pointer array
			IVP_U_BigVector<IVP_Compact_Poly_Point> swapped_points(point_estimate);

			const IVP_Compact_Ledge* ledge = ltn->get_compact_hull(); //returns const only?!

			if(ledge != NULL)
			{
				//
				// just swap the convex hull ledge, no recursion
				//
				const_cast<IVP_Compact_Ledge*>(ledge)->byte_swap_all(&swapped_points);
			}

			//
			// recursively swap the ledgetree
			//
			const_cast<IVP_Compact_Ledgetree_Node*>(ltn)->byte_swap_all(&swapped_points);
			printf("Found %d unique points\n", swapped_points.len());
		}
		else
			const_cast<IVP_Compact_Ledgetree_Node*>(ltn)->byte_swap_all(NULL);
	}

#if defined (WIN32)
	byte_swap();
#endif
}


