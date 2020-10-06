// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef IVP_OBJECT_HASH_INCLUDED
#define IVP_OBJECT_HASH_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#ifndef _IVP_VHASH_INCLUDED
#	include  "ivu_vhash.hxx"
#endif

class IVP_Object_Callback_Table {
public:
    IVP_Real_Object                   *real_object;
    IVP_U_Vector<IVP_Listener_Object>  listeners;
    ~IVP_Object_Callback_Table();
};


class IVP_Object_Callback_Table_Hash : protected IVP_VHash
{
protected:
    IVP_BOOL compare(void *elem0, void *elem1) const;
    int      object_to_index(IVP_Real_Object *real_object);

public:
    void add_table(IVP_Object_Callback_Table *table)
    {
	add_elem(table, object_to_index(table->real_object));
    };

    IVP_Object_Callback_Table *remove_table(IVP_Real_Object *real_object)
    {
	IVP_Object_Callback_Table table;
	table.real_object = real_object;
	return (IVP_Object_Callback_Table *)remove_elem(&table, object_to_index(real_object));
    };

    IVP_Object_Callback_Table *find_table(IVP_Real_Object *real_object)
    {
	IVP_Object_Callback_Table table;
	table.real_object = real_object;
	return (IVP_Object_Callback_Table *)find_elem(&table, object_to_index(real_object));
    };
  

    ~IVP_Object_Callback_Table_Hash();
    IVP_Object_Callback_Table_Hash(int init_size) : IVP_VHash(init_size) {;};
};


#endif
