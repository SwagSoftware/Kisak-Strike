#ifndef HK_PHYSICS_MANAGER_H
#define HK_PHYSICS_MANAGER_H

class hk_Manager : public hk_Object
{
protected:
	inline hk_Manager(hk_Environment *env): hk_Object(env){
		;
	}

	inline virtual ~hk_Manager()	{	}

public:
	HK_NEW_DELETE_FUNCTION_VIRTUAL_CLASS( hk_Manager, HK_MEMORY_CLASS_MANAGER );		// e.g. defines new as   new (hk_Memory *)hk_Class_Name
};


#endif /*HK_PHYSICS_MANAGER_H*/

