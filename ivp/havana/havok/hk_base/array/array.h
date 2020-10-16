#ifndef HK_BASE_ARRAY_H
#define HK_BASE_ARRAY_H

#ifndef HK_BASE_BASE_H
#	error Include <hk_base/base.h> instead.
#endif

class hk_Array_Base
{
	public:
			inline hk_Array_Base();
	protected:
			inline hk_Array_Base(char *elems, int n_elems);

			void alloc_mem(int size, int num);
			void grow_mem(int size, int num);
			void grow_mem(int size);

			hk_array_store_index m_memsize;
			hk_array_store_index m_n_elems;
			char* m_elems;
};


template<class T>
class hk_Array :  protected hk_Array_Base
{
	public:
		inline hk_Array( int initial_size = 0 );
		inline ~hk_Array();

		typedef hk_array_index iterator;

		inline hk_array_index add_element_unchecked( T element );

		inline hk_array_index add_element( T element );

		inline void remove_element(	hk_array_index idx, void (*static_reindex_element)( T&, hk_Array<T>*, hk_array_store_index ) );
		inline void search_and_remove_element( T& );
		inline void search_and_remove_element_sorted( T& );

		inline T& operator() (int i);
		inline const T& operator() (int i) const;

		inline T& element_at(int i);
		inline const T& element_at (int i) const;

		inline hk_array_index index_of( T& );
		inline void remove_all();

		inline void free_elem_array();		// OS: name to be discussed
		inline void set( hk_Array<T> & ); // OS: name to be discussed


		inline int length();

		inline int get_capacity();
		inline void reserve(int n);

		inline T& get_element( iterator );
		inline hk_bool is_valid( iterator );
		inline iterator next( iterator );
		inline iterator start();

	HK_PUBLIC:

		T *get_elems(){ return (T*)m_elems; };
	protected:
		inline hk_Array(T *elems, int initial_size);
		// for preallocated array
};

#include <hk_base/array/array.inl>

#endif /* HK_BASE_ARRAY_H */


