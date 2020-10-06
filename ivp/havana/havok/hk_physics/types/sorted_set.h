#ifndef HK_PHYSICS_SORTED_SET_H
#define HK_PHYSICS_SORTED_SET_H

// HK_EXPORT_PUBLIC


#ifdef HK_HAVE_PRAGMA_INTERFACE
//#	pragma interface
#endif

#define HK_SORTED_SET_MAXVALUE 1e10f

#define HK_SORTED_SET_USELONG
//: enable an extra sparse linkage amoung the objects for faster insert 

#define HK_SORTED_SET_UNUSED ( (1<<16) -1 )
#define HK_SORTED_SET_LONG_UNUSED ( (1<<16) -2 )


//: keeps a sorted list of elements
// memory usage:
// base 20 bytes
// per element storage 16 bytes:
class hk_Sorted_Set
{
	public:
		typedef hk_real hk_ss_real;
		typedef hk_sorted_set_index hk_ss_index;

		hk_Sorted_Set(int size=4, hk_Memory *mem =hk_Memory::get_instance());

		~hk_Sorted_Set();
		
		hk_ss_index add_ss_element(void *elem, hk_ss_real value, hk_Memory *mem = hk_Memory::get_instance()); 
			// insert the element and returns and access index 

		void remove_ss_element (hk_ss_index index);
			// removes the element using the index returned at the add()

		inline void* find_min_element();
			//: find the first element in the queue

		inline hk_bool has_elements();
			// check for an empty set

		inline hk_ss_real find_min_value();
			// find the minimum value 


#ifdef HK_DEBUG
		void check();
		// check the consistency of this datastructure
#endif

	protected:
		struct hk_SS_Element {
			typedef hk_real hk_ss_real;

			public:
#ifdef HK_SORTED_SET_USELONG
				hk_ss_index  m_long_next;
				hk_ss_index  m_long_prev;
#endif
				hk_ss_index m_next;
				hk_ss_index m_prev;
				hk_ss_real m_value;
				void *m_element;
		} *m_elems;

		hk_sorted_set_store_index	m_malloced_size;
		hk_sorted_set_store_index	m_free_list;
	public:
		hk_SS_Element::hk_ss_real  m_min_value;
#ifdef HK_SORTED_SET_USELONG
		hk_sorted_set_store_index	m_first_long;
#endif
		hk_sorted_set_store_index	m_first_element;
		hk_sorted_set_store_index	m_n_added_elements;
};


void* hk_Sorted_Set::find_min_element()
{
	HK_ASSERT( m_first_element != HK_SORTED_SET_UNUSED);
	return m_elems[m_first_element].m_element;
};


hk_bool hk_Sorted_Set::has_elements()
{
	return (hk_bool) (m_n_added_elements!=0);
}

hk_Sorted_Set::hk_ss_real hk_Sorted_Set::find_min_value()
{
	return m_min_value;
}

#endif /* HK_PHYSICS_SORTED_SET_H */
