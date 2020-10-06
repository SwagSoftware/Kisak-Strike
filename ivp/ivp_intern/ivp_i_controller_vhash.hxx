// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#ifndef _IVP_CONTROLLER_VHASH_INCLUDED
#define _IVP_CONTROLLER_VHASH_INCLUDED


#ifndef _IVP_VHASH_INCLUDED
#	include  "ivu_vhash.hxx"
#endif

class IVP_Real_Object;

class IVP_Controller_VHash : protected IVP_VHash
{
protected:
    IVP_Real_Object *real_object;
    IVP_BOOL compare(void *elem0, void *elem1) const;
    int      controller_to_index(IVP_Controller *cntrl);

public:
    void add_controller(IVP_Controller *controller)
    {
	add_elem(controller, controller_to_index(controller));
    };

    IVP_Controller *remove_controller(IVP_Controller *controller)
    {
	return (IVP_Controller *)remove_elem(controller, controller_to_index(controller));
    };

    IVP_Controller *find_controller(IVP_Controller *controller)
    {
	return (IVP_Controller *)find_elem(controller, controller_to_index(controller));
    };
  

    ~IVP_Controller_VHash();
    IVP_Controller_VHash(IVP_Real_Object *object, int init_size) : IVP_VHash(init_size) { real_object = object; };
};


#endif
