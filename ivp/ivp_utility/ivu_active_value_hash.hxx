#ifndef _IVP_ACTIVE_VALUE_HASH_INCLUDED
#define _IVP_ACTIVE_VALUE_HASH_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#ifndef _IVP_VHASH_INCLUDED
#	include  "ivu_vhash.hxx"
#endif

class IVP_U_Active_Value;


class IVP_Active_Value_Hash : protected IVP_VHash
{
protected:
    IVP_BOOL compare(void *elem0, void *elem1) const;
    int      object_to_index(IVP_U_Active_Value *av);

public:
    void add_active_value(IVP_U_Active_Value *av)
    {
	add_elem(av, object_to_index(av));
	av->add_reference();
    };

    IVP_U_Active_Value *remove_active_value(IVP_U_Active_Value *av)
    {
	IVP_U_Active_Value *av_out =  (IVP_U_Active_Value *)remove_elem(av, object_to_index(av));
	av_out->remove_reference();
	return av_out;
    };

    IVP_U_Active_Value *find_active_value(IVP_U_Active_Value *av)
    {
	return (IVP_U_Active_Value *)find_elem(av, object_to_index(av));
    };
  

    ~IVP_Active_Value_Hash();
    IVP_Active_Value_Hash(int init_size) : IVP_VHash(init_size) {;};
};


#endif
