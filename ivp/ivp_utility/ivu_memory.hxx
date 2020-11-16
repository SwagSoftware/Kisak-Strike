

#ifndef IVP_MEM_INCLUDED
#define IVP_MEM_INCLUDED

#ifndef WIN32
#	pragma interface
#endif

#include <cstdint> //lwss -x64 fixes

//lwss - x64 changes
#ifdef __x86_64__
#define IVU_MEM_ALIGN 0x50 // grabbed from vphysics retail
#define IVU_MEM_MASK 0xffffffffffffffe0; //^^
#define IVU_MEMORY_BLOCK_SIZE 0x7fe0	// size of block loaded by
#else
#define IVU_MEM_ALIGN 0x20 //align to chach line data 32Byte
#define IVU_MEM_MASK 0xffffffe0; 
#define IVU_MEMORY_BLOCK_SIZE (0x8000-IVU_MEM_ALIGN)	// size of block loaded by
#endif
//lwss end

struct p_Memory_Elem {
    struct p_Memory_Elem *next;
    char	data[4];
};

class IVP_U_Memory {
#ifdef MEMTEST
    IVP_U_Vector<char> mem_vector;
#else    
        struct p_Memory_Elem *first_elem;
	struct p_Memory_Elem *last_elem;
	char *speicherbeginn;
	char *speicherende;
#endif
    
        short transaction_in_use:16; //for asserts
	unsigned short size_of_external_mem:16;  // or 0 if not external mem; used for e.g. PS2 scratchpad
    
	void *speicher_callok(unsigned int groesse);
        void free_mem_transaction();
        inline void *align_to_next_adress(void *p);
    
	public:
        IVP_U_Memory();
        ~IVP_U_Memory();
	void init_mem();
	void *get_mem(unsigned int size);	
	void *get_memc(unsigned int size);
	void free_mem();
#if !defined(MEMTEST)
	char *neuer_sp_block(unsigned int groesse);
#endif    

        //for usage with transactions
        void init_mem_transaction_usage(char *external_mem = 0, int size = 0); //alloc first block, that block is never freed (except whole memory is destroyed)
        inline void end_memory_transaction(); //free all memory blocks except first one, reset memory pointer
        inline void start_memory_transaction(); //only one transaction at a time
        inline void *get_mem_transaction(unsigned int size);
};

void IVP_U_Memory::start_memory_transaction()
{
    //IVP_ASSERT(transaction_in_use==0);
    IVP_IF(1) {
#ifdef SUN
		if(size_of_external_mem==0) {
	        init_mem_transaction_usage(); //get first block
		}
#endif
    }
	transaction_in_use++;
}

void IVP_U_Memory::end_memory_transaction()
{
    //IVP_IF(1) {
	    transaction_in_use--;
    //}
    //IVP_ASSERT(transaction_in_use==0);
    free_mem_transaction();
    IVP_IF(1) {
#ifdef SUN
		free_mem(); //clear last block to be able to detect unitialized memory access
#endif
    }
}


//warning: dependency with function neuer_sp_block
inline void *IVP_U_Memory::align_to_next_adress(void *p) {
    long adress=(long)p;
    adress += IVU_MEM_ALIGN-1;
    adress  =adress & IVU_MEM_MASK;
    return (void*)adress;
}

    
inline void    *IVP_U_Memory::get_mem(unsigned int groesse)
{
#ifdef MEMTEST
    if (groesse){
	char *data = (char *)ivp_malloc_aligned(groesse,IVU_MEM_ALIGN);
	mem_vector.add(data);
	return (void *)data;
    }else{
	return NULL;
    }
#else
	char *p = speicherbeginn;
	char *op = p;
	p += groesse;
        p = (char*)align_to_next_adress(p);
	
	if (p >= speicherende) {
	    return ((void *) this->neuer_sp_block(groesse));
	} else {
	    speicherbeginn = p;
		IVP_IF( ((intptr_t)op > 0x780000 ) && ((intptr_t)op < 0x792f48)) {
			op++;
			op--;
		}
	    return ((void *)op);
	}
#endif
}

inline void    *IVP_U_Memory::get_mem_transaction(unsigned int groesse){
    IVP_ASSERT(transaction_in_use==1);
    return get_mem(groesse);
}



#endif












