// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

#include <ivp_physics.hxx>
#include <stdlib.h>

#if !defined(__MWERKS__) || !defined(__POWERPC__)
#include <malloc.h>
#endif

#ifndef WIN32
#	pragma implementation "ivu_memory.hxx"
#endif

#include "ivu_memory.hxx"

//IVP_Environment *ivp_global_env=NULL;

void ivp_memory_check(void *a) {
  if(a) return;
#if 0
  //if( !ivp_global_env ) {
  //        return;
  //    }
	    //int fp;
	    //fp=sceOpen("host0:/ipion_out/ipion.txt",SCE_CREAT);
		unsigned int adress=(unsigned int)a;
		IVP_Time now_time=ivp_global_env->get_current_time();
		IVP_DOUBLE tt=now_time.get_time();
		if(tt>11.98) {
		    printf("trying to free %lx time %f\n",a,tt);
		    if ( a == (void *)0x30ca50){
		    	printf("Crashing soon\n");
		    }

#endif
}

void ivp_byte_swap4(uint& fourbytes)
{
	struct FOURBYTES {
		union { 
			unsigned char b[4]; 
			uint v;
		};
	} in, out;
		  
	in.v = fourbytes;

	out.b[0] = in.b[3];
	out.b[1] = in.b[2];
	out.b[2] = in.b[1];
	out.b[3] = in.b[0];

	fourbytes = out.v;
}

void ivp_byte_swap2(ushort& twobytes)
{
	struct TWOBYTES {
		union { 
			unsigned char b[2]; 
			ushort v;
		};
	} in, out;
		  
	in.v = twobytes;

	out.b[0] = in.b[1];
	out.b[1] = in.b[0];
	
	twobytes = out.v;
}

void *p_malloc(unsigned int size)
{
#ifndef GEKKO
    return malloc(size);
#else
    return (void*)( new char[size] );
 #endif
}

char *p_calloc(int nelem,int size)
{
	int s = nelem * size;

#ifndef GEKKO
	char *sp = (char *)malloc(s);
#else
	char *sp = new char[s];
#endif

	memset(sp,0,s);
	return sp;
}

void* p_realloc(void* memblock, int size)
{
#ifndef GEKKO
	return realloc(memblock, size);
#else	
	IVP_ASSERT(0);
	return 0;
#endif
}

void p_free(void* data)
{
#ifndef GEKKO
	free(data);
#else
	delete [] (char*)data;
#endif
}

#define IVP_MEMORY_MAGIC 0x65981234
struct IVP_Aligned_Memory {
    int magic_number;
    void *back_link;
};

void *ivp_malloc_aligned(int size, int alignment){
#if defined(SUN__)
    return memalign( alignment, size);
#else    
    size += alignment + sizeof (IVP_Aligned_Memory);
    
    IVP_Aligned_Memory *data = (IVP_Aligned_Memory*)p_malloc( (unsigned int) size);
    data->magic_number = IVP_MEMORY_MAGIC;
    
    void *ret = (void *)((((long)data) + alignment + sizeof(IVP_Aligned_Memory) - 1) & (-alignment));
    ((void **)ret)[-1] = (void *)data;
    return ret;
#endif    
}

void ivp_free_aligned(void *data)
{
#if defined(SUN__)
    p_free(data);
#else    
    IVP_Aligned_Memory *am = (IVP_Aligned_Memory *)((void **)data)[-1];
    IVP_ASSERT ( am->magic_number == IVP_MEMORY_MAGIC);
    IVP_IF(1){
	am->magic_number = 0;
    }
    p_free( (char *)am );
#endif    
}

IVP_U_Memory::~IVP_U_Memory()
{
    free_mem();
}

void IVP_U_Memory::init_mem_transaction_usage(char *external_mem, int size){
    //IVP_IF(1) {
	transaction_in_use=0;
    //}
#if defined(MEMTEST)
#else    
    if (external_mem){
	size_of_external_mem = size - IVU_MEM_ALIGN;  // for header
	struct p_Memory_Elem *memelem = (struct p_Memory_Elem *)external_mem;
	memelem->next = last_elem;
	last_elem = memelem;
	char *tmp = (char*)align_to_next_adress(&memelem->data[0]);
	speicherbeginn = tmp;
   	speicherende = tmp + size_of_external_mem;
    }else{
	size_of_external_mem = 0;
        neuer_sp_block(0);
    }
    first_elem=last_elem;
#endif    
}


void IVP_U_Memory::free_mem_transaction()
{
#if defined(MEMTEST)
    for (int i = mem_vector.len()-1; i>=0; i--){
	ivp_free_aligned(mem_vector.element_at(i));
    }
    mem_vector.clear();
#else    
        //IVP_ASSERT(first_elem!=NULL); playstation doesn't like this ...
	struct p_Memory_Elem	*f,*n;
	for (f = last_elem; f; f = n){
		n = f->next;
		if(f==first_elem) {
		    break;
		}
		p_free( f);
	}
	speicherbeginn = (char*)align_to_next_adress(&first_elem->data[0]);

	size_t ng;
	if (size_of_external_mem){
	    ng = size_of_external_mem;
	}else{
	    ng = IVU_MEMORY_BLOCK_SIZE;
	}
	speicherende = speicherbeginn + ng;
	last_elem = first_elem;
#endif	
}

#if !defined(MEMTEST)
char *IVP_U_Memory::neuer_sp_block(unsigned int groesse)
{
	size_t ng = IVU_MEMORY_BLOCK_SIZE - sizeof(p_Memory_Elem);
	groesse += IVU_MEM_ALIGN-1;
	groesse &= IVU_MEM_MASK;
	if (groesse > ng) ng = groesse;
	struct p_Memory_Elem *memelem = (struct p_Memory_Elem *)p_malloc(sizeof(p_Memory_Elem)+ng+IVU_MEM_ALIGN);
	memelem->next = last_elem;
	last_elem = memelem;
	char *tmp = (char*)align_to_next_adress(&memelem->data[0]);
	speicherbeginn =  tmp + groesse;
   	speicherende   =  tmp + ng;
	return tmp;
}
#endif

void *IVP_U_Memory::get_memc(unsigned int groesse)
{
//	if (groesse & 0x7) *(int *)0 = 0;
    void *neubeginn=get_mem(groesse);
    register long *z=(long*)neubeginn;
    memset((char *)z,0,groesse);
    return(neubeginn);
}

IVP_U_Memory::IVP_U_Memory(){
    init_mem();
}

void IVP_U_Memory::init_mem()
{
        transaction_in_use=3; //for transactions: first start init_mem_transaction_usage
	size_of_external_mem = 0;
#if defined(MEMTEST)
#else	
	speicherbeginn = 0;
	speicherende = 0;
	last_elem = 0;
	first_elem=NULL;
#endif
}

void IVP_U_Memory::free_mem()
{
#if defined(MEMTEST)
    for (int i = mem_vector.len()-1; i>=0; i--){
	ivp_free_aligned(mem_vector.element_at(i));
    }
    mem_vector.clear();
#else    
	struct p_Memory_Elem	*f,*n;
	for (f = last_elem; f; f = n){
		n = f->next;
		if (this->size_of_external_mem &&  f==first_elem) {
		    break;
		}
		P_FREE( f);
	}
	speicherbeginn = 0;
	speicherende = 0;
	last_elem = 0;
	first_elem=0;
#endif
}







