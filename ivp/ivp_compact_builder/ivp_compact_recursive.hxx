// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

/********************************************************************************
 *	File:	       	ivp_compact_rekursive.hxx	
 *	Description:	create rekursive compact ledges
 ********************************************************************************/

#ifndef WIN32
#	pragma interface
#endif


class IVP_Compact_Recursive {
  IVP_U_Vector<IVP_Compact_Ledge> ledges;
  IVP_Compact_Ledge *hull;

  void build_convex_hull();
  void set_rekursive_convex_hull();

  void add_compact_ledge_treenode(const IVP_Compact_Ledgetree_Node *node);
public:
  IVP_Compact_Recursive();
  ~IVP_Compact_Recursive();

  void add_compact_ledge(const IVP_Compact_Ledge *ledge);
  void add_compact_surface( const IVP_Compact_Surface *surface);

  IVP_Compact_Ledge *compile();
};
