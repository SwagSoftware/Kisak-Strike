// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

//IVP_EXPORT_PUBLIC

#ifndef _P_LIST_INCLUDED
#define _P_LIST_INCLUDED

/* list header, list elements should have:
   next, prev
   */

template<class T>
class P_List {
public:
    T *first;
    int len;
    
    void insert(T *elem){
	IVP_ASSERT(!elem->next);
	elem->next = this->first;
	if(first){
	    first->prev = elem;
	}
	elem->prev = 0;
	first = elem;
	len++;
    };
    
    void remove(T *elem){
	register T *e = elem;
	register T *h;
	IVP_ASSERT(e->next != (T*)-1);
	if ((h = e->prev)){
	    h->next = e->next;
	}else{
	    first = e->next;
	}
	if ((h = e->next)){
	    h->prev = e->prev;
	}
	e->next = (T *)-1;	// invalid flag !!
	len--;
	//elem->next = 0;
    };
    
    P_List(){
	first=0;
	len=0;
    }
};


/* list header, list elements should have:
   next
   */

template<class T> class P_Simple_List {
public:
    T *first;
    
    void insert(T *elem){
	elem->next = this->first;
	first = elem;
    };
    
    void remove_first(T *elem){
	register T *e = elem;
	register T *h;
	first = e->next;
	if (h = e->next){
	    h->prev = e->prev;
	}
    }
};

#endif

