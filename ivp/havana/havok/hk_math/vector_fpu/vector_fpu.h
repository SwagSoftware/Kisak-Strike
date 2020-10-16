#ifndef HK_VECTOR_FPU_INCLUDED
#define HK_VECTOR_FPU_INCLUDED

#define hk_VecFPU_SIZE_DOUBLE 2
#define hk_VecFPU_MEM_MASK_DOUBLE 0xfffffff0 //16Byte per Block
#define hk_VecFPU_MASK_DOUBLE 0xfffffffe
#define hk_VecFPU_MEMSHIFT_DOUBLE 3 //8 Bytes per Floating Point Number

#define hk_VecFPU_SIZE_FLOAT 4
#define hk_VecFPU_MEM_MASK_FLOAT 0xfffffff0 //16Byte per Block
#define hk_VecFPU_MASK_FLOAT 0xfffffffc
#define hk_VecFPU_MEMSHIFT_FLOAT 2 //4 Bytes per Floating Point Number


#if 0
#if defined( IVP_WILLAMETTE ) || defined( IVP_WMT_ALIGN )
    #define hk_VecFPU_SIZE 2  
    #define IVP_VECFPU_LD 1
    #define IVP_VECFPU_MASK 0xfffffffe 

    #define IVP_VECFPU_MEM_MASK 0xfffffff0 //16Byte per Block
    #define IVP_VECFPU_MEMSHIFT 3 //8 Bytes per Floating Point Number
#else
    #define IVP_VECFPU_SIZE 4  
    #define IVP_VECFPU_LD 2
    #define IVP_VECFPU_MASK 0xfffffffc 

    #ifdef IVP_NO_DOUBLE
    #	define IVP_VECFPU_MEM_MASK 0xfffffff0 //16Byte
    #	define IVP_VECFPU_MEMSHIFT 2 //4 Bytes per Floating Point Number
    #else
    #	define IVP_VECFPU_MEM_MASK 0xffffffe0 //32Byte
    #	define IVP_VECFPU_MEMSHIFT 3 //8 Bytes per Floating Point Number
    #endif
#endif
#endif



// assumption: the memory is always allocated aligned to the block size
//             otherwise -> failure
// hk_BOOL adress_aligned is meant to indicate wether array is starting at aligned adress. If not,
// extra work has to be done
class hk_VecFPU {
public:
    static inline void fpu_add_multiple_row(hk_real *target_adress,hk_real *source_adress,hk_real factor,int size,hk_bool adress_aligned);
    static inline void fpu_multiply_row(hk_real *target_adress,hk_real factor,int size,hk_bool adress_aligned);
    static inline void fpu_exchange_rows(hk_real *target_adress1,hk_real *target_adress2,int size,hk_bool adress_aligned);
    static inline void fpu_copy_rows(hk_real *target_adress,hk_real *source_adress,int size,hk_bool adress_aligned);
    static inline void fpu_set_row_to_zero(hk_real *target_adress,int size,hk_bool adress_aligned);
    static inline hk_real fpu_large_dot_product(hk_real *base_a,hk_real *x,int size,hk_bool adress_aligned);
    static inline int calc_aligned_row_len(int unaligned_len,hk_real *dummy_type);


    static inline void fpu_add_multiple_row(hk_double *target_adress,hk_double *source_adress,hk_double factor,int size,hk_bool adress_aligned);
    static inline void fpu_multiply_row(hk_double *target_adress,hk_double factor,int size,hk_bool adress_aligned);
    static inline void fpu_exchange_rows(hk_double *target_adress1,hk_double *target_adress2,int size,hk_bool adress_aligned);
    static inline void fpu_copy_rows(hk_double *target_adress,hk_double *source_adress,int size,hk_bool adress_aligned);
    static inline void fpu_set_row_to_zero(hk_double *target_adress,int size,hk_bool adress_aligned);
    static inline hk_double fpu_large_dot_product(hk_double *base_a,hk_double *x,int size,hk_bool adress_aligned);
    static inline int calc_aligned_row_len(int unaligned_len,hk_double *dummy_type);
};

#include <hk_math/vector_fpu/vector_fpu.inl>

#endif //HK_VECTOR_FPU_INCLUDED
