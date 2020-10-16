hk_Array_Base::hk_Array_Base()
{
}


template <class T>
hk_Array<T>::hk_Array( int initial_size )
{
	m_n_elems = 0;
	if ( initial_size) {
		m_memsize = initial_size;
		m_elems = (char *)hk_allocate(T, initial_size, HK_MEMORY_CLASS_ARRAY );
	}else{
		m_memsize = 0;
		m_elems = HK_NULL;
	}
}


template <class T>
hk_Array<T>::hk_Array(T* elems, int initial_size)
{
	HK_ASSERT( (char *)elems == (char *)(this +1 ));
	m_n_elems = 0;
	m_elems = (char *)elems;
	m_memsize = initial_size;
}


template <class T>
hk_Array<T>::~hk_Array()
{
	if (m_elems)
	{
		HK_ASSERT( get_elems() != (T *)(this+1));
		hk_deallocate( T, get_elems(), m_memsize, HK_MEMORY_CLASS_ARRAY );
	}
}


template <class T>
hk_array_index hk_Array<T>::add_element_unchecked( T element )
{
	HK_ASSERT( m_n_elems < m_memsize );
	get_elems()[ m_n_elems ] = element;
	return m_n_elems++;
}

template <class T>
hk_array_index hk_Array<T>::add_element( T element )
{
	if ( m_n_elems >= m_memsize)
	{
		grow_mem( sizeof(T) );
	}
	get_elems()[ m_n_elems ] = element;
	return m_n_elems++;
}

#if !defined(_XBOX)
template <class T>
void hk_Array<T>::remove_element( hk_array_index idx,
		void (*static_member_func)( T&, hk_Array<T>* , hk_array_store_index ) )
{
	m_n_elems --;
	if (idx < m_n_elems)
	{
		(*static_member_func)( get_elems()[ m_n_elems ], this, idx );
		get_elems()[idx] = get_elems()[ m_n_elems ];
	}
}
#endif

template <class T>
void hk_Array<T>::search_and_remove_element( T& t)
{
	int index = index_of(t);
	HK_ASSERT(index>=0);

	m_n_elems --;
	if (index < m_n_elems)
	{
		get_elems()[index ] = get_elems()[ m_n_elems ];
	}
}

template <class T>
void hk_Array<T>::search_and_remove_element_sorted( T& t)
{
	int index = index_of(t);
	HK_ASSERT(index>=0);

	m_n_elems --;
	while ( index < m_n_elems ){
		get_elems()[index ] = get_elems()[ index+1];
		index++;
	}
}



template <class T>
T& hk_Array<T>::operator() (int i)
{
	return get_elems()[i];
}

template <class T>
const T& hk_Array<T>::operator() (int i) const
{
	return get_elems()[i];
}

template <class T>
hk_array_index hk_Array<T>::index_of( T& t)
{
	for (int i = m_n_elems-1; i>=0 ;i--)
	{
		if(get_elems()[i]==t)
		{
			return i;
		}
	}
	return -1;
}

template <class T>
T& hk_Array<T>::element_at( int i )
{
	return get_elems()[i];
}

template <class T>
const T& hk_Array<T>::element_at( int i ) const 
{
	return get_elems()[i];
}

template <class T>
void hk_Array<T>::remove_all()
{
	m_n_elems = 0;
}

template <class T>
void hk_Array<T>::free_elem_array()
{
	if ( m_elems && ((char *)m_elems != (char *)(this + 1)))
	{
		hk_deallocate( char, m_elems, m_memsize * sizeof(T), HK_MEMORY_CLASS_ARRAY );
	}
	m_n_elems = 0;
	m_memsize = 0;
	m_elems = HK_NULL;
}



template <class T>
int hk_Array<T>::length()
{
	return m_n_elems;
}

template <class T>
int hk_Array<T>::get_capacity()
{
	return m_memsize;
}

template <class T>
void hk_Array<T>::reserve(int n)
{
	if ( m_memsize < n + m_n_elems)
	{
		int new_size = m_memsize + m_memsize;
		if ( new_size == 0) new_size = 2;
		while ( new_size < n + m_n_elems ){
			new_size += new_size;
		}
		grow_mem( sizeof(T), new_size - m_memsize );
	}
}

template <class T>
T& hk_Array<T>::get_element( iterator i )
{
	return get_elems()[i];
}

template <class T>
hk_bool hk_Array<T>::is_valid( iterator i)
{
	return hk_bool(i>=0);
}

template <class T>
typename hk_Array<T>::iterator hk_Array<T>::next(iterator i)
{
	return i-1;
}

template <class T>
typename hk_Array<T>::iterator hk_Array<T>::start()
{
	return m_n_elems-1;
}

template <class T>
void hk_Array<T>::set( hk_Array<T> &t ) // OS: name to be discussed
{
	*this = t;
	t.m_n_elems = 0;
	t.m_memsize = 0;
	t.m_elems = HK_NULL;
}



