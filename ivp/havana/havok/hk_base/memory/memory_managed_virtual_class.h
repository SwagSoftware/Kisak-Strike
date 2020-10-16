#ifndef HK_BASE_MEMORY_MANAGED_VIRTUAL_CLASS
#define HK_BASE_MEMORY_MANAGED_VIRTUAL_CLASS




// object is something which will be managed by the scene management
class hk_Memory_Managed_Virtual_Class
{
private:
	protected:
		inline virtual ~hk_Memory_Managed_Virtual_Class(){
			;
		}
	public:
		int m_memsize;
	protected:
	inline hk_Memory_Managed_Virtual_Class()	{	}


	public:
	inline int get_size(){
		return m_memsize;
	}
};


#endif /* HK_BASE_MEMORY_MANAGED_VIRTUAL_CLASS */

