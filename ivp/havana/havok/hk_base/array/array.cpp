#include <hk_base/base.h>
#include <hk_base/memory/memory.h>
#include <string.h>

void hk_Array_Base::alloc_mem( int size, int num)
{
	m_elems = hk_allocate( char, size * num, HK_MEMORY_CLASS_ARRAY );
	m_memsize = num;
}

void hk_Array_Base::grow_mem( int size )
{
	int new_memsize = m_memsize + m_memsize;
	if (!new_memsize) {
		new_memsize = 2;
	}

	char *new_array = hk_allocate( char, new_memsize * size, HK_MEMORY_CLASS_ARRAY );

	memcpy( new_array, m_elems, m_memsize * size );
	if ( m_elems && ((char *)m_elems != (char *)(this + 1)))
	{
		hk_deallocate( char, m_elems, m_memsize * size, HK_MEMORY_CLASS_ARRAY );
	}
	m_memsize = new_memsize;
	m_elems = new_array;
}

void hk_Array_Base::grow_mem( int size, int n_elems )
{
	int new_memsize = m_memsize + n_elems;
	char *new_array = hk_allocate( char, new_memsize * size, HK_MEMORY_CLASS_ARRAY );

	memcpy( new_array, m_elems, m_memsize * size );
	if ( m_elems && ((char *)m_elems != (char *)(this + 1)))
	{
		hk_deallocate( char, m_elems, m_memsize * size, HK_MEMORY_CLASS_ARRAY );
	}
	m_memsize = new_memsize;
	m_elems = new_array;
}

