// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef IVP_I_FPOINT_HASH_INCLUDED
#define IVP_I_FPOINT_HASH_INCLUDED

#ifndef WIN32
#	pragma interface
#endif


#ifndef _IVP_VHASH_INCLUDED
#	include  "ivu_vhash.hxx"
#endif


class IVP_I_FPoint_VHash: protected IVP_VHash
{
protected:
    IVP_BOOL compare(void *elem0, void *elem1) const;
    int point_to_index(IVP_U_Float_Point *point);
    
public:
    void add_point(IVP_U_Float_Point *point){
	add_elem( point, point_to_index(point));
    };

    IVP_U_Float_Point *remove_point(IVP_U_Float_Point *point){
	return (IVP_U_Float_Point *)remove_elem( point, point_to_index(point));
    };

    IVP_U_Float_Point *find_point(IVP_U_Float_Point *point){
	return (IVP_U_Float_Point *)find_elem( point, point_to_index(point));
    };

    int len() const { return IVP_VHash::len();};
    IVP_U_Float_Point *element_at(int i) const { return (IVP_U_Float_Point *)elems[i].elem;};


    ~IVP_I_FPoint_VHash();
    IVP_I_FPoint_VHash(int init_size):IVP_VHash(init_size){;};
    void print() const { IVP_VHash::print();};
};


#endif
