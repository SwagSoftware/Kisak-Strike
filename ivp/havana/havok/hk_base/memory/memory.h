#ifndef HK_BASE_MEMORY_H
#define HK_BASE_MEMORY_H

#define HK_MEMORY_MAX_ROW				12
#define HK_MEMORY_MAX_SIZE_SMALL_BLOCK	512
#define HK_MEMORY_EXTRA_BLOCK_SIZE		8192

#define HK_MEMORY_MAGIC_NUMBER			0x3425234
#define HK_MEMORY_CACHE_ALIGNMENT 64

#define HK_MEMORY_ENABLE_STATISTICS 

//#define HK_MEMORY_ENABLE_DEBUG_CHECK		/* enable if extra asserts and checks should be performed */

// If you add a memory class, remember to change
// the table in print_statistics
enum hk_MEMORY_CLASS
{
	HK_MEMORY_CLASS_UNKNOWN = 0,
	HK_MEMORY_CLASS_DUMMY,
	HK_MEMORY_CLASS_ARRAY,
	HK_MEMORY_CLASS_HASH,
	HK_MEMORY_CLASS_SORTED_SET,
	HK_MEMORY_CLASS_DENSE_VECTOR,
	HK_MEMORY_CLASS_ENTITY,
	HK_MEMORY_CLASS_RIGID_BODY,
	HK_MEMORY_CLASS_EFFECTOR,
	HK_MEMORY_CLASS_CORE,
	HK_MEMORY_CLASS_CONSTRAINT,
	HK_MEMORY_CLASS_MANAGER,

	HK_MEMORY_CLASS_SIMUNINT,
	HK_MEMORY_CLASS_SIM_SLOT,

	HK_MEMORY_CLASS_PSI_TIME,

	HK_MEMORY_CLASS_BROAD_PHASE,
	HK_MEMORY_CLASS_CA_DISPATCHER,

	//havok compat
	HK_MEMORY_CLASS_GENERIC,
	HK_MEMORY_CLASS_STL,
	HK_MEMORY_CLASS_MATH,
	HK_MEMORY_CLASS_EVENT,
	HK_MEMORY_CLASS_ACTION,
	HK_MEMORY_CLASS_GEOMETRY,
	HK_MEMORY_CLASS_CDINFO,
	HK_MEMORY_CLASS_DISPLAY,
	HK_MEMORY_CLASS_EXPORT,

	HK_MEMORY_CLASS_USR1,
	HK_MEMORY_CLASS_USR2,
	HK_MEMORY_CLASS_USR3,
	HK_MEMORY_CLASS_USR4,

	HK_MEMORY_CLASS_MAX = 256
};


#define HK_NEW_DELETE_FUNCTION_CLASS( THIS_CLASS, MEMORY_CLASS )							\
	inline void *operator new(hk_size_t size){										\
		HK_ASSERT ( sizeof( THIS_CLASS ) == size );											\
		void *object = hk_Memory::get_instance()->allocate( size, MEMORY_CLASS );			\
		return object;																		\
	}																						\
																							\
	inline void operator delete(void *o){											\
		hk_Memory::get_instance()->deallocate( o, sizeof( THIS_CLASS ), MEMORY_CLASS );				\
	}																						\
																							\

#define HK_NEW_DELETE_FUNCTION_VIRTUAL_CLASS( THIS_CLASS, MEMORY_CLASS )					\
	inline void *operator new(hk_size_t size ){										\
		THIS_CLASS *object = (THIS_CLASS *)hk_Memory::get_instance()->allocate( size, MEMORY_CLASS );	\
		object->m_memsize = size;															\
		return object;																		\
	}																						\
	inline void operator delete(void *o){											\
		THIS_CLASS *object = (THIS_CLASS *)o;												\
		hk_Memory::get_instance()->deallocate( o, object->m_memsize, MEMORY_CLASS );			\
	}																						\


class hk_Memory
{
	public:

		hk_Memory();
		//: a really empty constructor

		hk_Memory(char *buffer, int buffer_size);

		void init_memory( char *buffer, int buffer_size );
		//: initialized the memory pool


		~hk_Memory();

		void* allocate(int byte_size, hk_MEMORY_CLASS );
		//: allocate a piece of memory
		//: Note: the size of that piece is not stored, so
		//: the user has to do remember the size !!!
		//: the memory class is currently used only for statistics

		void  deallocate(void*, int byte_size, hk_MEMORY_CLASS );
		//: deallocate a piece of memory 

		void* allocate_and_store_size(int byte_size, hk_MEMORY_CLASS );
		//: allocate a piece of memory
		//: Note: the size of that piece is stored, so
		//: 16 bytes of memory are wasted
		void  deallocate_stored_size(void*, hk_MEMORY_CLASS );
		//: deallocate a piece of memory which has been allocated of allocate_and_store_size

		void* allocate_debug(int n, const char* file, int line);
		void  deallocate_debug(void*, int n,const char* file,int line);

		static hk_Memory *get_instance();

		//void print_statistics(class hk_Console *); // see hk_Memory_Util

	public: // THE interfaces to the system allocate, change this if you want to add in your own big block memory allocation
		static void *aligned_malloc( hk_size_t size, hk_size_t alignment);
		static void aligned_free(	 void *data );
		static inline void* memcpy(void* dest,const void* src,int size);
		static inline void* memset(void* dest, hk_uchar val, hk_int32 size);

	private:
		void* allocate_real(int size);
		inline int size_to_row(int size);

	protected:
		friend class hk_Memory_Util;
		class hk_Memory_Elem {
		public:
			hk_Memory_Elem *m_next;
			int m_magic;
		};

		class hk_Memory_Statistics {
		public:
			int m_size_in_use;
			int m_n_allocates;
			int m_blocks_in_use;
			int m_max_size_in_use;
		};

		class hk_Memory_Block {
		public:
			hk_Memory_Block *m_next;
			hk_int32		m_pad[(64 - sizeof(hk_Memory_Block *))/ sizeof(hk_int32)];
		};

	protected:
		hk_Memory_Elem *m_free_list[HK_MEMORY_MAX_ROW];
		hk_Memory_Block *m_allocated_memory_blocks;

		int  m_blocks_in_use[HK_MEMORY_MAX_ROW];
		char *m_memory_start;
		char *m_memory_end;
		char *m_used_end;
		int m_row_to_size[HK_MEMORY_MAX_ROW];
		hk_Memory_Statistics m_statistics[HK_MEMORY_CLASS_MAX];
		char m_size_to_row[ HK_MEMORY_MAX_SIZE_SMALL_BLOCK+1 ];

};

#include <hk_base/memory/memory.inl>


#ifdef HK_DEBUG
#	define hk_allocate(T,N,CLASS)	\
		(T*)hk_Memory::get_instance()->allocate(sizeof(T)*(N), CLASS)
		//(T*)hk_Memory::get_instance()->allocate_debug(sizeof(T)*(N),__FILE__,__LINE__)
#	define hk_deallocate(T,P,N, CLASS)	\
		hk_Memory::get_instance()->deallocate(P,sizeof(T)*(N), CLASS)
		//hk_Memory::get_instance()->deallocate_debug(P,sizeof(T)*(N),__FILE__,__LINE__)
#else


#	define hk_allocate(T,N, CLASS)	\
		(T*)hk_Memory::get_instance()->allocate(sizeof(T)*(N), CLASS)
#	define hk_deallocate(T,P,N, CLASS)	\
		hk_Memory::get_instance()->deallocate(P,sizeof(T)*(N), CLASS)
#endif



#endif // HK_BASE_MEMORY_H
