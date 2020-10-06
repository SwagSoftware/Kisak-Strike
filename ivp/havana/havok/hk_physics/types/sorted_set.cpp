#include <hk_physics/physics.h>

#ifdef HK_HAVE_PRAGMA_INTERFACE
#	pragma implementation
#endif

#include <hk_physics/types/sorted_set.h>

hk_Sorted_Set::hk_Sorted_Set(int start_size, hk_Memory *mem)
{
    m_malloced_size = start_size;
    m_n_added_elements = 0;
    m_elems = hk_allocate(hk_SS_Element, m_malloced_size, HK_MEMORY_CLASS_SORTED_SET);

    m_free_list = 0;
    for( unsigned int i = 0; i < m_malloced_size; i++){
		m_elems[i].m_next = i+1;
    }

    m_elems[m_malloced_size-1].m_next = HK_SORTED_SET_UNUSED;
    m_first_element = HK_SORTED_SET_UNUSED;
#ifdef HK_SORTED_SET_USELONG
    m_first_long = HK_SORTED_SET_UNUSED;
#endif    
    m_min_value = HK_SORTED_SET_MAXVALUE;
}

hk_Sorted_Set::~hk_Sorted_Set()
{
    hk_deallocate( hk_SS_Element, m_elems, m_malloced_size, HK_MEMORY_CLASS_SORTED_SET);
}


hk_Sorted_Set::hk_ss_index hk_Sorted_Set::add_ss_element(
		void *elem,
		hk_ss_real value, hk_Memory *mem)
{
    // search free element first
    HK_ASSERT(value < HK_SORTED_SET_MAXVALUE);

    hk_SS_Element *e;
    hk_ss_index return_index;

    m_n_added_elements += 1;

	// check for space left in array
    if ( m_free_list != HK_SORTED_SET_UNUSED )
	{
		// use old place
		return_index = m_free_list;
		e = &m_elems[m_free_list];
		m_free_list = e->m_next;
    }
	else
	{
		// increase array
		unsigned int i;
		hk_SS_Element *new_elems = hk_allocate(	hk_SS_Element,	2 * (m_malloced_size+1), HK_MEMORY_CLASS_SORTED_SET); 
		for(i=0; i < m_malloced_size; i++){
			new_elems[i] = m_elems[i];
		}

		hk_deallocate(hk_SS_Element, m_elems, m_malloced_size, HK_MEMORY_CLASS_SORTED_SET);

    	m_malloced_size = m_malloced_size * 2 + 1;

		m_elems = new_elems;
		return_index = i;
		e = &m_elems[i++];
		m_free_list = i;
		
		for(; i < m_malloced_size; i++){
			m_elems[i].m_next = i+1;
		}
		m_elems[m_malloced_size-1].m_next = HK_SORTED_SET_UNUSED;
    }

    e->m_element = elem;
    e->m_value = value;

#ifdef HK_SORTED_SET_USELONG
    e->m_long_next = HK_SORTED_SET_LONG_UNUSED;
#endif

    if ( value <= m_min_value ){  // quick insert first element
		m_min_value = value;
		e->m_next = m_first_element;
		if (m_first_element != HK_SORTED_SET_UNUSED) {
			m_elems[m_first_element].m_prev = return_index;
		}
		m_first_element = return_index;
		e->m_prev = HK_SORTED_SET_UNUSED;
			//check();
		return return_index;
    }

    HK_ASSERT( m_first_element != HK_SORTED_SET_UNUSED);

    // find place to insert element, we know at least one exists
    int lastj = m_first_element;

#ifdef HK_SORTED_SET_USELONG
    // lastj is the last element after we want to insert
    int lo = m_first_long;
    int max_cmp_len = 3;
    while (lo != HK_SORTED_SET_UNUSED){
		// do _long jumps if possible
		hk_SS_Element *flong = & m_elems[lo];
		if (flong->m_value >= value){
			break;
	}
	max_cmp_len++;
	lastj = lo;
	lo = flong->m_long_next;
    }
    int firstj_after = lastj;
    int count_cmp = 0;
#endif

    hk_SS_Element *f = & m_elems[lastj];
	int j = f->m_next;
    for (; j != HK_SORTED_SET_UNUSED; j = f->m_next){
		f = &m_elems[j];
	#ifdef HK_SORTED_SET_USELONG
		count_cmp ++;
	#endif
		if ( value > f->m_value ){
			lastj = j;
			continue;
		}
		hk_SS_Element *l = &m_elems[lastj];
		e->m_next = l->m_next;
		m_elems[j].m_prev = return_index;
		e->m_prev = lastj;
		l->m_next = return_index;
		goto end;
    }
    // insert after the last element f
    e->m_next = j;
    e->m_prev = lastj;
    f->m_next = return_index;
end:
#ifdef HK_SORTED_SET_USELONG
    // insert long jmp
    if ( count_cmp > max_cmp_len ) {
	int new_long_pos = firstj_after; // search new position for longjump
	for (int k = 2; k < max_cmp_len; k ++){
	    new_long_pos = m_elems[new_long_pos].m_next;
	}
	int next_of_first = m_elems[firstj_after].m_long_next;
    hk_SS_Element *nl = &m_elems[new_long_pos];

	HK_ASSERT(nl->m_long_next == HK_SORTED_SET_LONG_UNUSED);

	if ( next_of_first != HK_SORTED_SET_LONG_UNUSED ){ // real long as first element
	    nl->m_long_next = next_of_first;
	    nl->m_long_prev = firstj_after;
	    if ( next_of_first != HK_SORTED_SET_UNUSED ) {
		    m_elems[next_of_first].m_long_prev = new_long_pos;
	    }
	    m_elems[firstj_after].m_long_next = new_long_pos;
	}else{  // insert long as first long
	    nl->m_long_next = m_first_long;
	    nl->m_long_prev = HK_SORTED_SET_UNUSED;
	    if (m_first_long != HK_SORTED_SET_UNUSED){
		    m_elems[m_first_long].m_long_prev = new_long_pos;
	    }	
	    m_first_long = new_long_pos;
	}
    }
#endif    
    //check();
    return return_index;
}

void hk_Sorted_Set::remove_ss_element(hk_ss_index index){
    hk_SS_Element *e = & m_elems[index];
    unsigned int prev = e->m_prev;
    unsigned int next = e->m_next;
    if (prev != HK_SORTED_SET_UNUSED) { // not the first element
	    m_elems[prev].m_next = next;
	    if (next != HK_SORTED_SET_UNUSED){
		    m_elems[next].m_prev = prev;
	    }
    }else{
	    m_first_element = next;
	    if (next != HK_SORTED_SET_UNUSED){
		    m_elems[next].m_prev = prev;
		    m_min_value = m_elems[next].m_value;
	    }else{
		    m_min_value = HK_SORTED_SET_MAXVALUE;
	    }
    }
#ifdef HK_SORTED_SET_USELONG
    unsigned int long_next = e->m_long_next;
    if ( long_next != HK_SORTED_SET_LONG_UNUSED ){
	    // element is used as a long insert
	    unsigned int long_prev = e ->m_long_prev;
	    if ( long_next != HK_SORTED_SET_UNUSED ) {
		    m_elems[long_next].m_long_prev = long_prev;
	    }
	    if (long_prev == HK_SORTED_SET_UNUSED ){
		    m_first_long = long_next;
	    }else{
		    m_elems[long_prev].m_long_next = long_next;
	    }
    }
#endif
    m_n_added_elements -= 1;
    e->m_next = m_free_list;
    m_free_list = index;
    //check();
}

#if 0 && defined(HK_DEBUG)
void hk_Sorted_Set::check(){
    {
		HK_SORTED_SET_FIXED_POINT last_val = 0;
		for (unsigned int i = m_first_element; i != HK_SORTED_SET_UNUSED; i = m_elems[i].m_next){
			hk_Sorted_Set_Element *e = &m_elems[i];
			HK_ASSERT(last_val <= e->m_value);
			last_val = e->m_value;

			if ( i == first_element ) {
				HK_ASSERT( e->m_prev == HK_SORTED_SET_UNUSED);
			}else{
				HK_ASSERT( m_elems[e->m_prev].m_next == i );
			}
			if ( e->m_next != HK_SORTED_SET_UNUSED ) {
				HK_ASSERT( elems[e->m_next].m_prev == i);
			}
		}
    }
#ifdef HK_SORTED_SET_USELONG
    {
		HK_SORTED_SET_FIXED_POINT last_val = 0;
		for (unsigned int i = m_first_long; i != HK_SORTED_SET_UNUSED; i = elems[i].long_next){
			hk_Sorted_Set_Element *e = &m_elems[i];
			HK_ASSERT(last_val <= e->m_value);
			last_val = e->m_value;

			if ( i == m_first_long ) {
				HK_ASSERT( e->m_long_prev == HK_SORTED_SET_UNUSED);
			}else{
				HK_ASSERT( m_elems[e->m_long_prev].m_long_next == i );
			}
			if ( e->m_long_next != HK_SORTED_SET_UNUSED ) {
				HK_ASSERT( m_elems[e->m_long_next].m_long_prev == i);
			}
		}
    }
#endif
}

#endif
