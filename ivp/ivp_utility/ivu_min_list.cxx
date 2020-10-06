// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.


#ifndef WIN32
#	pragma implementation "ivu_min_list.hxx"
#endif
#include <ivp_physics.hxx>
#include "ivu_min_list.hxx"
						 
IVP_U_Min_List::IVP_U_Min_List(int start_size)
{
	// If this assertion triggers, it's going to overflow...
	if (start_size > IVP_U_MINLIST_MAX_ALLOCATION)
		start_size = IVP_U_MINLIST_MAX_ALLOCATION;
		
	malloced_size = start_size;
	counter = 0;
	elems = (IVP_U_Min_List_Element *) p_malloc(sizeof(IVP_U_Min_List_Element) * malloced_size); 
	unsigned int i = 0;
	free_list = i;
	for( ; i < malloced_size; i++ )
	{
		elems[i].next = i+1;
	}
	
	elems[malloced_size-1].next = IVP_U_MINLIST_UNUSED;
	first_element = IVP_U_MINLIST_UNUSED;
	
#ifdef IVP_U_MINLIST_USELONG
	first_long = IVP_U_MINLIST_UNUSED;
#endif    
	min_value = IVP_U_MINLIST_MAXVALUE;
}


IVP_U_Min_List::~IVP_U_Min_List()
{
	P_FREE(elems);
}

IVP_U_MINLIST_INDEX IVP_U_Min_List::add(void *elem, IVP_U_MINLIST_FIXED_POINT value)
{
	// search free element first
	IVP_ASSERT(value <= P_FLOAT_MAX);
	IVP_U_Min_List_Element *e;
	IVP_U_MINLIST_INDEX return_index;
	counter += 1;
	
	if ( free_list != IVP_U_MINLIST_UNUSED )
	{
		IVP_ASSERT( free_list < malloced_size );

		return_index = free_list;
		e = &elems[free_list];
		free_list = e->next;
	}
	else
	{
		// If this assertion triggers, it's going to overflow...
		IVP_ASSERT( malloced_size != IVP_U_MINLIST_MAX_ALLOCATION );

		// Clamp allocation to 65535
		int nNewMallocSize = malloced_size * 2 + 1;
		if (nNewMallocSize > IVP_U_MINLIST_MAX_ALLOCATION)
			nNewMallocSize = IVP_U_MINLIST_MAX_ALLOCATION;

#ifdef _DEBUG
		static int g_ErrorCount = 0;
		if (g_ErrorCount < 3)
		{
			if (nNewMallocSize > 2000)
			{
				ivp_message("Warning! Large min_list array size! Could indicate long sweeps\n");
				++g_ErrorCount;
			}
		}
#endif

		unsigned int i;
		IVP_U_Min_List_Element *new_elems = (IVP_U_Min_List_Element *)
			p_malloc(sizeof(IVP_U_Min_List_Element) * (nNewMallocSize + 1));
		
		for(i=0; i < malloced_size; i++)
		{
			new_elems[i] = elems[i];
		}

		malloced_size = nNewMallocSize;

		P_FREE(elems);
		elems = new_elems;
		return_index = i;
		e = &elems[i++];
		free_list = i;

		for(; i < malloced_size; i++)
		{
			elems[i].next = i+1;
		}

		elems[malloced_size-1].next = IVP_U_MINLIST_UNUSED;
    }
	
	e->element = elem;
	e->value = value;
	
#ifdef IVP_U_MINLIST_USELONG
	e->long_next = IVP_U_MINLIST_LONG_UNUSED;
#endif
	
	if ( value <= min_value )
	{  
		// quick insert first element
		min_value = value;
		e->next = first_element;
		if (first_element != IVP_U_MINLIST_UNUSED) 
		{
			elems[first_element].prev = return_index;
		}
		first_element = return_index;
		IVP_ASSERT( (first_element == IVP_U_MINLIST_UNUSED) || (first_element < malloced_size) );

		e->prev = IVP_U_MINLIST_UNUSED;

#ifdef _DEBUG
//		check();
#endif 
		
		return return_index;
	}
	
	IVP_ASSERT( first_element != IVP_U_MINLIST_UNUSED);
	// find place to insert element, we know at least one exists

	int lastj = first_element;
	
#ifdef IVP_U_MINLIST_USELONG
	// lastj is the last element after we want to insert
	int lo = first_long;
	int max_cmp_len = 3;
	while (lo != IVP_U_MINLIST_UNUSED)
	{
		// do _long jumps if possible
		IVP_U_Min_List_Element *flong = & elems[lo];
		if (flong->value >= value)
		{
			break;
		}
		max_cmp_len++;
		lastj = lo;
		lo = flong->long_next;
	}

	int firstj_after = lastj;
	int count_cmp = 0;
#endif

	IVP_U_Min_List_Element *f = &elems[lastj];
	int j;
	for (j = f->next; j != IVP_U_MINLIST_UNUSED; j = f->next)
	{
		f = &elems[j];
		
#ifdef IVP_U_MINLIST_USELONG
		count_cmp ++;
#endif
		if ( value > f->value )
		{
			lastj = j;
			continue;
		}
		IVP_U_Min_List_Element *l = &elems[lastj];
		e->next = l->next;
		elems[j].prev = return_index;
		e->prev = lastj;
		l->next = return_index;
		goto end;
	}
	// insert after the last element f
	e->next = j;
	e->prev = lastj;
	f->next = return_index;
	
end:
#ifdef IVP_U_MINLIST_USELONG
	// insert long jmp
	if ( count_cmp > max_cmp_len ) 
	{
		int new_long_pos = firstj_after; 
		
		// search new position for longjump
		for (int k = 2; k < max_cmp_len; k ++)
		{
			new_long_pos = elems[new_long_pos].next;
		}
		
		int next_of_first = elems[firstj_after].long_next;
		IVP_U_Min_List_Element *nl = &elems[new_long_pos];
		IVP_ASSERT(nl->long_next == IVP_U_MINLIST_LONG_UNUSED);
		
		if ( next_of_first != IVP_U_MINLIST_LONG_UNUSED )
		{ 
			// real long as first element
			nl->long_next = next_of_first;
			nl->long_prev = firstj_after;
			if ( next_of_first != IVP_U_MINLIST_UNUSED ) 
			{
				elems[next_of_first].long_prev = new_long_pos;
			}
			elems[firstj_after].long_next = new_long_pos;
		}
		else
		{  // insert long as first long
			nl->long_next = first_long;
			nl->long_prev = IVP_U_MINLIST_UNUSED;
			if (first_long != IVP_U_MINLIST_UNUSED)
			{
				elems[first_long].long_prev = new_long_pos;
			}	
			first_long = new_long_pos;
		}
    }
#endif
    
#ifdef _DEBUG
//	check();
#endif

    return return_index;
}


void IVP_U_Min_List::remove_minlist_elem(IVP_U_MINLIST_INDEX index)
{
	IVP_ASSERT( index < malloced_size );

	IVP_U_Min_List_Element *e = & elems[index];
	unsigned int prev = e->prev;
	unsigned int next = e->next;
	if (prev != IVP_U_MINLIST_UNUSED) 
	{ 
		// not the first element
		elems[prev].next = next;
		if (next != IVP_U_MINLIST_UNUSED)
		{
			elems[next].prev = prev;
		}
	}
	else
	{
		first_element = next;
		IVP_ASSERT( (first_element == IVP_U_MINLIST_UNUSED) || (first_element < malloced_size) );
		if (next != IVP_U_MINLIST_UNUSED)
		{
			elems[next].prev = prev;
			min_value = elems[next].value;
		}
		else
		{
			min_value = IVP_U_MINLIST_MAXVALUE;
		}
	}
	
#ifdef IVP_U_MINLIST_USELONG
	unsigned int long_next = e->long_next;
	if ( long_next != IVP_U_MINLIST_LONG_UNUSED )
	{
		// element is used as a long insert
		unsigned int long_prev = e ->long_prev;
		if ( long_next != IVP_U_MINLIST_UNUSED ) 
		{
			elems[long_next].long_prev = long_prev;
		}
		
		if (long_prev == IVP_U_MINLIST_UNUSED )
		{
			first_long = long_next;
		}
		else
		{
			elems[long_prev].long_next = long_next;
		}
	}
#endif

    counter-=1;
    e->next = free_list;
    free_list = index;

#ifdef _DEBUG
//	check();
#endif
}


void IVP_U_Min_List::check()
{
	{
		IVP_U_MINLIST_FIXED_POINT last_val = 0;
		for (unsigned int i = first_element; i != IVP_U_MINLIST_UNUSED; i = elems[i].next)
		{
			IVP_ASSERT( i < malloced_size );

			IVP_U_Min_List_Element *e = &elems[i];
			IVP_ASSERT(last_val <= e->value);
			last_val = e->value;
			
			if ( i == first_element ) 
			{
				IVP_ASSERT( e->prev == IVP_U_MINLIST_UNUSED);
			}
			else
			{
				IVP_ASSERT( elems[e->prev].next == i );
			}
			
			if ( e->next != IVP_U_MINLIST_UNUSED ) 
			{
				IVP_ASSERT( elems[e->next].prev == i);
			}
		}
	}
	
#ifdef IVP_U_MINLIST_USELONG
	{
		IVP_U_MINLIST_FIXED_POINT last_val = 0;
		for (unsigned int i = first_long; i != IVP_U_MINLIST_UNUSED; i = elems[i].long_next)
		{
			IVP_ASSERT( i < malloced_size );

			IVP_U_Min_List_Element *e = &elems[i];
			IVP_ASSERT(last_val <= e->value);
			last_val = e->value;
			
			if ( i == first_long ) 
			{
				IVP_ASSERT( e->long_prev == IVP_U_MINLIST_UNUSED);
			}
			else
			{
				IVP_ASSERT( elems[e->long_prev].long_next == i );
			}
			
			if ( e->long_next != IVP_U_MINLIST_UNUSED ) 
			{
				IVP_ASSERT( elems[e->long_next].long_prev == i);
			}
		}
	}
#endif
}

