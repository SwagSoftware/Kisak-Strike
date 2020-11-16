// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.
#ifdef IVP_WILLAMETTE
#include <emmintrin.h>
#endif

// IVP_EXPORT_PRIVATE

#include <ivp_physics.hxx>

#if defined(LINUX) || defined(SUN) || (__MWERKS__ && __POWERPC__)
#	include <alloca.h>
#endif
#include <ivp_great_matrix.hxx>
#include <ivp_debug_manager.hxx>
#include <ivu_memory.hxx>

#define IVP_UNILATERAL_SOLVER_MAX_STEP 250
#define IVP_STATIC_FRICTION_TEST_EPS 0.00001f //is a percentage value
#define COMPLEX_EPS 10E-6f
#define IVP_NUMERIC_TEST_EVERY_N 7


inline void IVP_VecFPU::fpu_add_multiple_row(IVP_DOUBLE *target_adress,IVP_DOUBLE *source_adress,IVP_DOUBLE factor,int size,IVP_BOOL adress_aligned) {
    if(adress_aligned==IVP_FALSE) {
	//we have to calculate the block size and shift adresses to lower aligned adresses
	//lwss- x64 fixes
	//long result_adress = long(source_adress) & IVP_VECFPU_MEM_MASK;
	//target_adress = (IVP_DOUBLE *)( long (target_adress) & IVP_VECFPU_MEM_MASK);
	//size += (long(source_adress)-result_adress)>>IVP_VECFPU_MEMSHIFT;
    intptr_t result_adress = intptr_t(source_adress) & IVP_VECFPU_MEM_MASK;
    target_adress = (IVP_DOUBLE *)( intptr_t (target_adress) & IVP_VECFPU_MEM_MASK);
    size += (intptr_t(source_adress)-result_adress)>>IVP_VECFPU_MEMSHIFT;
	//lwss end
	source_adress=(IVP_DOUBLE *)result_adress;
    }

#if defined(IVP_USE_PS2_VU0)

    asm __volatile__("
        mfc1    $8,%3
        qmtc2    $8,vf5		# vf5.x   factor
    1:
	lqc2    vf4,0x0(%1)     # vf4 *source
	vmulx.xyzw vf6,vf4,vf5  # vf6 *source * factor
	lqc2    vf4,0x0(%0)
	addi    %0, 0x10
	vadd.xyzw vf6,vf4,vf6   # vf6 = *dest + factor * *source
	addi    %2,-4
	addi    %1, 0x10
	sqc2    vf6,-0x10(%0)
	bgtz    %2,1b
	nop
	"
	: /* no output */
	: "r" (target_adress), "r" (source_adress) , "r" (size), "f" (factor)
	: "$8" , "memory");
#else
	IVP_IF_WILLAMETTE_OPT(IVP_Environment_Manager::get_environment_manager()->ivp_willamette_optimization) {
#       if defined(IVP_WILLAMETTE)
	    ;
        __m128d factor128=_mm_set1_pd(factor);

        for(int i=size;i>0;i-=IVP_VECFPU_SIZE) {
	        __m128d source_d=_mm_load_pd(source_adress);
	        __m128d prod_d=_mm_mul_pd(factor128,source_d);
	        __m128d target_d=_mm_load_pd(target_adress);
	        target_d=_mm_add_pd(prod_d,target_d);
	        _mm_store_pd(target_adress,target_d);

	        target_adress+=IVP_VECFPU_SIZE;
	        source_adress+=IVP_VECFPU_SIZE;
		}
#endif
	} else {

        for(int i=size;i>0;i-=IVP_VECFPU_SIZE) {
#       if IVP_VECFPU_SIZE == 2
	    IVP_DOUBLE a = source_adress[0] * factor;
	    IVP_DOUBLE b = source_adress[1] * factor;
	    a += target_adress[0];
	    b += target_adress[1];
	    target_adress[0] = a;
	    target_adress[1] = b;
#   elif IVP_VECFPU_SIZE == 4
	    IVP_DOUBLE a = source_adress[0] * factor;
	    IVP_DOUBLE b = source_adress[1] * factor;
	    IVP_DOUBLE c = source_adress[2] * factor;
	    IVP_DOUBLE d = source_adress[3] * factor;
	    a += target_adress[0];
	    b += target_adress[1];
	    c += target_adress[2];
	    d += target_adress[3];
	    target_adress[0] = a;
	    target_adress[1] = b;
	    target_adress[2] = c;
	    target_adress[3] = d;
#   else 
	    IVP_DOUBLE a = source_adress[0] * factor;
	    a += target_adress[0];
	    target_adress[0] = a;
#   endif
	    target_adress+=IVP_VECFPU_SIZE;
	    source_adress+=IVP_VECFPU_SIZE;
		}
	}
#endif
}

inline IVP_DOUBLE IVP_VecFPU::fpu_large_dot_product(IVP_DOUBLE *base_a, IVP_DOUBLE *base_b, int size, IVP_BOOL adress_aligned) {
    if(adress_aligned==IVP_FALSE) {
	    //we have to calculate the block size and shift adresses to lower aligned adresses
	    long result_adress = long(base_a) & IVP_VECFPU_MEM_MASK;
	    base_b = (IVP_DOUBLE *)( long (base_b) & IVP_VECFPU_MEM_MASK);
	    size += (long(base_a)-result_adress)>>IVP_VECFPU_MEMSHIFT;  // because start changed
	    base_a=(IVP_DOUBLE *)result_adress;
    }
#   if defined(IVP_WILLAMETTE)
    IVP_IF_WILLAMETTE_OPT(IVP_Environment_Manager::get_environment_manager()->ivp_willamette_optimization) {

	__m128d sum128 =_mm_set1_pd(0.0f);

	int i;
	for(i=size;i>=IVP_VECFPU_SIZE; i-= IVP_VECFPU_SIZE) {
	    __m128d mult1=_mm_load_pd(base_a);
	    __m128d mult2=_mm_load_pd(base_b);
	    __m128d prod =_mm_mul_pd(mult1,mult2);
	    sum128 =_mm_add_pd(prod,sum128);
	    base_a += IVP_VECFPU_SIZE;
	    base_b += IVP_VECFPU_SIZE;
	}

	__m128d dummy,low,high;
	low=sum128;
	dummy=sum128;
	//_mm_shuffle_pd(sum128,sum128,1); //swap high and low
	sum128=_mm_unpackhi_pd(sum128,dummy);
	high=sum128;
	__m128d sum64=_mm_add_sd(low,high);
	__m128d res=sum64;
	//_mm_shuffle_pd(sum64,sum64,1); 
	IVP_DOUBLE result_sum; 
	_mm_store_sd(&result_sum,res);
	
	for(;i>=0;i--) {
	    result_sum += base_a[i] * base_b[i];
	}

	return result_sum;
    } else {
	IVP_DOUBLE sum=0.0f;
	for(int i=size-1;i>=0;i--) {
	    sum += base_a[i] * base_b[i];
	}
	return sum;
    } 
#else
    IVP_DOUBLE sum=0.0f;
    for(int i=size-1;i>=0;i--) {
	sum += base_a[i] * base_b[i];
    }
    return sum;
#endif
}

inline void IVP_VecFPU::fpu_multiply_row(IVP_DOUBLE *target_adress,IVP_DOUBLE factor,int size,IVP_BOOL adress_aligned) {
    if(adress_aligned==IVP_FALSE) {
	//we have to calculate the block size and shift adresses to lower aligned adresses
	long adress,result_adress;
	adress=(long)target_adress;
	result_adress=adress & IVP_VECFPU_MEM_MASK;
	size+=(adress-result_adress)>>IVP_VECFPU_MEMSHIFT;
	target_adress=(IVP_DOUBLE *)result_adress;
    }
	IVP_IF_WILLAMETTE_OPT(IVP_Environment_Manager::get_environment_manager()->ivp_willamette_optimization) {
#ifdef IVP_WILLAMETTE
        __m128d factor128=_mm_set1_pd(factor);
        int i;
        for(i=size;i>0;i-=IVP_VECFPU_SIZE) {
	    __m128d target_d=_mm_load_pd(target_adress);
	    target_d=_mm_mul_pd(factor128,target_d);
	    _mm_store_pd(target_adress,target_d);
	
	    target_adress+=IVP_VECFPU_SIZE;
		}    
#endif
	} else {
        int i;
        for(i=size;i>0;i-=IVP_VECFPU_SIZE) {
#       if IVP_VECFPU_SIZE == 2
	    IVP_DOUBLE a = target_adress[0] * factor;
	    IVP_DOUBLE b = target_adress[1] * factor;
	    target_adress[0] = a;
	    target_adress[1] = b;
#       elif IVP_VECFPU_SIZE == 4
	    IVP_DOUBLE a = target_adress[0] * factor;
	    IVP_DOUBLE b = target_adress[1] * factor;
	    IVP_DOUBLE c = target_adress[2] * factor;
	    IVP_DOUBLE d = target_adress[3] * factor;
	    target_adress[0] = a;
	    target_adress[1] = b;
	    target_adress[2] = c;
	    target_adress[3] = d;
#       else
	    IVP_DOUBLE a = target_adress[0] * factor;
	    target_adress[0] = a;
#       endif
		target_adress+=IVP_VECFPU_SIZE;
	}
	}
}
// #+# sparc says rui, optimize for non vector units ( IVP_VECFPU_SIZE = 4 )
inline void IVP_VecFPU::fpu_exchange_rows(IVP_DOUBLE *target_adress1,IVP_DOUBLE *target_adress2,int size,IVP_BOOL adress_aligned) {
    if(adress_aligned==IVP_FALSE) {
	//we have to calculate the block size and shift adresses to lower aligned adresses
	long adress,result_adress;
	adress=(long)target_adress1;
	result_adress=adress & IVP_VECFPU_MEM_MASK;
	size+=(adress-result_adress)>>IVP_VECFPU_MEMSHIFT;
	target_adress1=(IVP_DOUBLE *)result_adress;
	adress=(long)target_adress2;
	adress=adress & IVP_VECFPU_MEM_MASK;
	target_adress2=(IVP_DOUBLE *)adress;
    }
    IVP_IF_WILLAMETTE_OPT(IVP_Environment_Manager::get_environment_manager()->ivp_willamette_optimization) {
#ifdef IVP_WILLAMETTE
        int i;
        for(i=size;i>0;i-=IVP_VECFPU_SIZE) {
	    __m128d a_d=_mm_load_pd(target_adress1);
	    __m128d b_d=_mm_load_pd(target_adress2);
	    _mm_store_pd(target_adress1,b_d);    
	    _mm_store_pd(target_adress2,a_d);    
	    
	    target_adress1+=IVP_VECFPU_SIZE;
	    target_adress2+=IVP_VECFPU_SIZE;
	}
#endif
	} else {
        int i;
        for(i=size;i>0;i-=IVP_VECFPU_SIZE) {
	        IVP_DOUBLE h;
#       if IVP_VECFPU_SIZE == 2
	        h=target_adress1[0];  target_adress1[0]=target_adress2[0];  target_adress2[0]=h;
	        h=target_adress1[1];  target_adress1[1]=target_adress2[1];  target_adress2[1]=h;
#       elif IVP_VECFPU_SIZE == 4
	        h=target_adress1[0];  target_adress1[0]=target_adress2[0];  target_adress2[0]=h;
	        h=target_adress1[1];  target_adress1[1]=target_adress2[1];  target_adress2[1]=h;
	        h=target_adress1[2];  target_adress1[2]=target_adress2[2];  target_adress2[2]=h;
	        h=target_adress1[3];  target_adress1[3]=target_adress2[3];  target_adress2[3]=h;
#       else
		h=target_adress1[0];  target_adress1[0]=target_adress2[0];  target_adress2[0]=h;
#       endif
	    target_adress1+=IVP_VECFPU_SIZE;
	    target_adress2+=IVP_VECFPU_SIZE;
		}
	}
}

inline void IVP_VecFPU::fpu_copy_rows(IVP_DOUBLE *target_adress,IVP_DOUBLE *source_adress,int size,IVP_BOOL adress_aligned) {
    if(adress_aligned==IVP_FALSE) {
	//we have to calculate the block size and shift adresses to lower aligned adresses
	long adress,result_adress;
	adress=(long)source_adress;
	result_adress=adress & IVP_VECFPU_MEM_MASK;
	size+=(adress-result_adress)>>IVP_VECFPU_MEMSHIFT;
	source_adress=(IVP_DOUBLE *)result_adress;
	adress=(long)target_adress;
	adress=adress & IVP_VECFPU_MEM_MASK;
	target_adress=(IVP_DOUBLE *)adress;
    }
	IVP_IF_WILLAMETTE_OPT(IVP_Environment_Manager::get_environment_manager()->ivp_willamette_optimization) {
#ifdef IVP_WILLAMETTE
        int i;
        for(i=size-1;i>=0;i-=IVP_VECFPU_SIZE) {
	        __m128d target_d=_mm_load_pd(source_adress);
	        _mm_store_pd(target_adress,target_d);

	        target_adress+=IVP_VECFPU_SIZE;
	        source_adress+=IVP_VECFPU_SIZE;
		}
#endif
	} else {
        int i;
        for(i=size-1;i>=0;i-=IVP_VECFPU_SIZE) {
	    int j;
	    for(j=0;j<IVP_VECFPU_SIZE;j++) {
	        target_adress[j] = source_adress[j];
		}
	    target_adress+=IVP_VECFPU_SIZE;
	    source_adress+=IVP_VECFPU_SIZE;
		}
	}
}

inline void IVP_VecFPU::fpu_set_row_to_zero(IVP_DOUBLE *target_adress,int size,IVP_BOOL adress_aligned) {
    if(adress_aligned==IVP_FALSE) {
        long adress,result_adress;
	adress=(long)target_adress;
	result_adress=adress & IVP_VECFPU_MEM_MASK;
	size+=(adress-result_adress)>>IVP_VECFPU_MEMSHIFT;
	target_adress=(IVP_DOUBLE *)result_adress;
    }
	IVP_IF_WILLAMETTE_OPT(IVP_Environment_Manager::get_environment_manager()->ivp_willamette_optimization) {
#ifdef IVP_WILLAMETTE
        __m128d zero128=_mm_set1_pd(0.0f);
        int i;
        for(i=size-1;i>=0;i-=IVP_VECFPU_SIZE) {	
	        _mm_store_pd(target_adress,zero128);

	        target_adress+=IVP_VECFPU_SIZE;
		} 
#endif
	} else {
        int i;
        for(i=size-1;i>=0;i-=IVP_VECFPU_SIZE) {
	        int j;
	        for(j=0;j<IVP_VECFPU_SIZE;j++) {
	            target_adress[j] = 0.0f;
			}
	        target_adress+=IVP_VECFPU_SIZE;
		}     
	}
}


// tests a line to be greater equal result value
IVP_RETURN_TYPE IVP_Great_Matrix_Many_Zero::matrix_check_unequation_line(int linenum)
{
    IVP_DOUBLE *dline = &matrix_values[aligned_row_len*linenum];
    IVP_DOUBLE left_side=0.0f;
    for(int i=0;i<columns;i++)
    {
	left_side+=result_vector[i]* *dline++;
    }
    IVP_DOUBLE eps = IVP_Inline_Math::fabsd(desired_vector[linenum]*IVP_STATIC_FRICTION_TEST_EPS);
    if(left_side+eps >= desired_vector[linenum])
    {
	return IVP_OK; 
    } else {
	//printf("gauss_unequ %f %f\n",left_side,desired_vector[linenum]);
	return IVP_FAULT;
    }
}

void IVP_Great_Matrix_Many_Zero::matrix_test_unequation()
{
    for(int i=0;i<columns;i++)
    {
	IVP_DOUBLE left=0.0f;
	for(int j=0;j<columns;j++){
	    left+=result_vector[j]*matrix_values[i*aligned_row_len+j];
	}
	IVP_IF(1) { printf("unequation_test left %f right %f\n",left,desired_vector[i]); }
    }
}

void IVP_Great_Matrix_Many_Zero::matrix_out_before_gauss()
{
    IVP_IF(1) {
    for(int i=0;i<columns;i++)
    {
	for(int j=0;j<columns;j++)
	{
	    printf("%.5f  ",matrix_values[i*aligned_row_len+j]);
	}
	printf("=  %.5f\n",desired_vector[i]);
    }
    }
}

// add factor * line number j to line number i (j=first i=second)
// the matrix-entries left of j in both lines are zero
void IVP_Great_Matrix_Many_Zero::add_multiple_line(int first,int second,IVP_DOUBLE factor)
{    
    IVP_DOUBLE *first_line=&matrix_values[first * this->aligned_row_len + first];
    IVP_DOUBLE *second_line=&matrix_values[second * this->aligned_row_len + first];

    IVP_VecFPU::fpu_add_multiple_row(second_line,first_line,factor,columns-first,IVP_FALSE);
    desired_vector[second]+=desired_vector[first] * factor;
}

void IVP_Great_Matrix_Many_Zero::debug_fill_zero() {
    //prevent SUN from getting read from uninitialized errors
    for(int i=0;i<columns;i++) {
	for(int j=0;j<aligned_row_len;j++) {
	    matrix_values[i*aligned_row_len+j]=0.0f;
	}
    }
}

IVP_RETURN_TYPE IVP_Great_Matrix_Many_Zero::solve_great_matrix_many_zero()
{
#ifdef DEBUG
    IVP_IF(0) {
	this->plausible_input_check();
    }
#endif
    transform_to_lower_null_triangle();
    IVP_RETURN_TYPE rt=solve_lower_null_matrix();
    return rt;
    
}

void IVP_Great_Matrix_Many_Zero::copy_matrix(IVP_Great_Matrix_Many_Zero *orig_mat) {
    for(int i=0;i<columns;i++) {
	for(int j=0;j<columns;j++) {
	    this->matrix_values[i*columns+j] = orig_mat->matrix_values[i*columns+j];
	}
	this->desired_vector[i] = orig_mat->desired_vector[i];
    }
}


void IVP_Great_Matrix_Many_Zero::align_matrix_values() {
    long adress,result_adress;
    adress=(long)matrix_values;
    int maximal_shift=(IVP_VECFPU_SIZE-1)<<IVP_VECFPU_MEMSHIFT;
    result_adress=(adress+maximal_shift) & IVP_VECFPU_MEM_MASK;
    matrix_values=(IVP_DOUBLE*)result_adress;

    IVP_IF(1){
	for(int i=0;i<columns;i++) {
	    IVP_DOUBLE *x = & matrix_values[ i * aligned_row_len ];
	    for(int j=columns; j < aligned_row_len; j++) {
		x[j] = 0.0f;
	    }
	}
	for(int j=columns; j < aligned_row_len; j++) {
	    desired_vector[j] = 0.0f;
	}
    }
}

void IVP_Great_Matrix_Many_Zero::copy_matrix(IVP_DOUBLE *values,IVP_DOUBLE *desired)
{
    for (int d = columns * columns - 1; d>=0; d--){
	values[d] = matrix_values[d];
    }
    for(int j=0;j<columns;j++){
	desired[j]=desired_vector[j];
    }
}

void IVP_Great_Matrix_Many_Zero::find_pivot_in_column(int col)
{
    IVP_DOUBLE best_val = IVP_Inline_Math::fabsd(matrix_values[col*aligned_row_len+col]);
    int pos=-1;
    for(int i=columns-1;i > col;i--)    {
	IVP_DOUBLE h=matrix_values[i*aligned_row_len+col];
	if( IVP_Inline_Math::fabsd(h) > best_val )	{
	    best_val = IVP_Inline_Math::fabsd(h);
	    pos=i;
	}
    }
    if(pos>=0) {
        exchange_rows(col,pos);
    }
}

//optimization: when called from transform_to_lower_null_triangle, begin by 'a' (first values are 0.0f)
void IVP_Great_Matrix_Many_Zero::exchange_rows(int a,int b)
{
    IVP_DOUBLE h;
#if 0 /* without Vector FPU */   
    for(int i=columns-1;i>=0;i--) {
	h=matrix_values[a*aligned_row_len+i];
	matrix_values[a*aligned_row_len+i]=matrix_values[b*aligned_row_len+i];
	matrix_values[b*aligned_row_len+i]=h;
    }
#endif
    IVP_VecFPU::fpu_exchange_rows(&matrix_values[b*aligned_row_len],&matrix_values[a*aligned_row_len],columns,IVP_TRUE);
    
    h=desired_vector[a];
    desired_vector[a]=desired_vector[b];
    desired_vector[b]=h;
}

// solve with Gauss-Algo. : upper triangle matrix with nulls in lower part
void IVP_Great_Matrix_Many_Zero::transform_to_lower_null_triangle()
{
    //matrix_out_before_gauss();
    int i,j;
    
    //j is counting columns, i is counting lines

    //walk through columns
    for(j=0;j<columns;j++)
    {
	IVP_DOUBLE diagonal_elem,negativ_inv_diagonal;
#if 0
	//not always pivot search
	diagonal_elem=matrix_values[j*aligned_row_len+j];
	if(IVP_Inline_Math::fabsd(diagonal_elem)<MATRIX_EPS) {
	    find_pivot_in_column(j);
	    diagonal_elem=matrix_values[j*aligned_row_len+j];
	    if(IVP_Inline_Math::fabsd(diagonal_elem)<MATRIX_EPS) {
		goto column_done;
	    }
	}
#endif
	//always pivot search
	find_pivot_in_column(j);
	diagonal_elem=matrix_values[j*aligned_row_len+j];
	if(IVP_Inline_Math::fabsd(diagonal_elem)<MATRIX_EPS) {
	    goto column_done;
	}	
	
	negativ_inv_diagonal=-1.0f/diagonal_elem;
	//walk through lines
	for(i=j+1;i<columns;i++)
	{
	    IVP_DOUBLE col_down_elem=matrix_values[i*aligned_row_len+j];
	    if(IVP_Inline_Math::fabsd(col_down_elem)>MATRIX_EPS)
	    {
		add_multiple_line(j,i,col_down_elem*negativ_inv_diagonal);
	    }
	}
column_done: ;	
    }

    //now matrix has zeros in lower-left part
    //solve columns from bottom to top
#if 0
    printf("matrix_after_gauss:\n");
    for(j=0;j<columns;j++)
    {
	//walk through lines
	for(i=0;i<columns;i++)
	{
	    printf("%.5f  ",matrix_values[j*columns+i]);
	    old_matrix[j*columns+i]=matrix_values[j*columns+i];
	}
	printf(" x -> %.5f\n",desired_vector[j]);
	res_p[j]=desired_vector[j];
    }
#endif

    //this->test_result(old_matrix,res_p);
    //P_DELETE(old_matrix);
    //return 0;
}


#ifdef DEBUG
void IVP_Great_Matrix_Many_Zero::plausible_input_check() {
    int i,j;
    for(i=0;i<columns;i++) {
	for(j=0;j<columns;j++) {
	    IVP_DOUBLE a=matrix_values[i*aligned_row_len+j];
	    IVP_ASSERT(a>-P_DOUBLE_MAX);
	    IVP_ASSERT(a<P_DOUBLE_MAX);
	}
	IVP_DOUBLE b=desired_vector[i];
	IVP_ASSERT(b>-P_DOUBLE_MAX);
	IVP_ASSERT(b<P_DOUBLE_MAX);
    }
}
#endif

// matrix values are in upper triangle form, solve from bottom to top
IVP_RETURN_TYPE IVP_Great_Matrix_Many_Zero::solve_lower_null_matrix()
{
    int i;
    for(i=columns-1;i>=0;i--)
    {
	IVP_DOUBLE *pointer_in_line=&matrix_values[i*aligned_row_len+columns-1];
     
	IVP_DOUBLE right_side=desired_vector[i];
	for(int j=columns-1;j>i;j--)
	{
	    right_side -= desired_vector[j] * *pointer_in_line;
	    pointer_in_line--;
	}
	IVP_DOUBLE left_side=*pointer_in_line;
	if(IVP_Inline_Math::fabsd(left_side) < MATRIX_EPS){
	    if(IVP_Inline_Math::fabsd(right_side) < MATRIX_EPS*1000.0f)	    {
		// rang(matrix) is not maximal and result space is greater 1
		// choose an arbitrary result element (0.0f)
		//printf("got_gauss_null %f %f\n",left_side,right_side);
		desired_vector[i]=0.0f;
		IVP_IF(0) {
		    printf("gauss_singular_matrix\n");
		}
	    } else {
		// no result possible
		if(1) {
		    i=columns-1;
		    while(i>=0) {
		        result_vector[i]=0.0f; //prevent SUN from detecting rui error
		        i--;
		    }
		}
		return IVP_FAULT;
	    }
	} else {
	    desired_vector[i]=right_side/left_side; // division left_side is not needed, it was done already above (make a vector of inverses)
	    if(desired_vector[i]>10E11f) {
		IVP_IF(1) { printf("\ngauss_failure\n\n"); }
	    }
	}
    }

    for(i=0;i<columns;i++) {
	result_vector[i]=desired_vector[i];
    }
    
    return IVP_OK;
}

void IVP_Great_Matrix_Many_Zero::matrix_multiplication(IVP_DOUBLE *first,IVP_DOUBLE *second) {
    int i,j,k;
    for(i=0;i<columns;i++) {
        for(j=0;j<columns;j++) {
	    IVP_DOUBLE sum=0.0f;
	    for(k=0;k<columns;k++) {
	        IVP_DOUBLE val=first[i*columns+k] * second[k*columns+j];
		sum += val;
	    }
	    matrix_values[i*columns+j] = sum;
        }
    }
}

//for debug only
int IVP_Great_Matrix_Many_Zero::test_result(IVP_DOUBLE *old_matrix,IVP_DOUBLE *old_desired)
{
    //desired_vector and matrix is changed during calculation, old_matrix and old_desired has values needed for test

    IVP_IF(1) {
    int i,j;
    for(j=0;j<columns;j++)
    {
	//walk through lines
	IVP_DOUBLE left_side=0.0f;
	for(i=0;i<columns;i++)
	{
	    left_side+=old_matrix[j*columns+i]*result_vector[i];
	    printf("multip %.5f %.5f  ",old_matrix[j*columns+i],result_vector[i]);
	}
	printf("\nwanted_res was %.5f now %.5f\n",old_desired[j],left_side);
    }
    }
    return 0;
}

IVP_Great_Matrix_Many_Zero::IVP_Great_Matrix_Many_Zero(int n) {
    IVP_ASSERT(0); //does not work any longer: because of Vector-FPU adress has to be aligned
                   //normal malloc + free is no good
    
    MATRIX_EPS=1.0f/10E8f;
    columns=n;
    calc_aligned_row_len();
    matrix_values=(IVP_DOUBLE*)p_malloc(n*aligned_row_len*sizeof(IVP_DOUBLE));
    desired_vector=(IVP_DOUBLE*)p_malloc(aligned_row_len*sizeof(IVP_DOUBLE));
    result_vector=(IVP_DOUBLE*)p_malloc(aligned_row_len*sizeof(IVP_DOUBLE));
}

IVP_Great_Matrix_Many_Zero::IVP_Great_Matrix_Many_Zero() {
    MATRIX_EPS=1.0f/10E8f; 
    matrix_values=NULL;
    desired_vector=NULL;
    result_vector=NULL;
    columns=0;
}

// both matrices are aligned
// my matrix is smaller than big_matrix, copy only those lines/columns that are wanted
// which are wanted is written in array original_pos
void IVP_Great_Matrix_Many_Zero::fill_from_bigger_matrix(IVP_Great_Matrix_Many_Zero *big_matrix,int *original_pos, int n_column)
{
    if (n_column == 0) return;
    if ( original_pos[ n_column-1 ] == n_column-1 ){ // no reorder
	IVP_IF(1){
	    for (int i = 0; i < n_column; i++){
		IVP_ASSERT( original_pos[i] == i);
	    }
	}
	for(int i=0;i<n_column;i++) {
	    IVP_DOUBLE *source = & big_matrix->matrix_values[  original_pos[i]*big_matrix->aligned_row_len ];
	    IVP_DOUBLE *dest = &matrix_values[i * aligned_row_len];
	    IVP_VecFPU::fpu_copy_rows(dest,source, aligned_row_len, IVP_TRUE);
	    this->desired_vector[i] = big_matrix->desired_vector[ original_pos[i] ];	    
	}
    }else{
	for(int i=0;i<n_column;i++) {
	    IVP_DOUBLE *dest = &matrix_values[i * aligned_row_len];
	    IVP_IF(( original_pos[i] != i )) {
		printf("original_pos_diff %d %d\n",i,original_pos[i]);
	    }
	    
	    IVP_DOUBLE *source = & big_matrix->matrix_values[  original_pos[i]*big_matrix->aligned_row_len ];
	    for(int j=0;j<n_column;j++){
		dest[j] = source[ original_pos[j] ];
	    }
	    this->desired_vector[i] = big_matrix->desired_vector[ original_pos[i] ];
	}
    }
}
    
// copies a column from original matrix to sub_matrix
void IVP_Great_Matrix_Many_Zero::copy_to_sub_matrix(IVP_DOUBLE *values_big_matrix,IVP_Great_Matrix_Many_Zero *sub_matrix,int *original_pos)
{
    for(int i=0;i<sub_matrix->columns;i++)
    {
	for(int j=0;j<sub_matrix->columns;j++)
	{
	    sub_matrix->matrix_values[i*sub_matrix->columns+j] = values_big_matrix[ original_pos[i]*columns + original_pos[j] ];
	}
    }
}

void IVP_Great_Matrix_Many_Zero::set_value(IVP_DOUBLE val, int col, int row)
{
    // ATT: slow through mults
    IVP_ASSERT(col>=0 && col<columns);
    IVP_ASSERT(row>=0 && row<columns);

    *(this->matrix_values+col+columns*row) = val;
}


IVP_DOUBLE IVP_Great_Matrix_Many_Zero::get_value(int col, int row) const
{
    // ATT: slow through mults
    IVP_ASSERT(col>=0 && col<columns);
    IVP_ASSERT(row>=0 && row<columns);

    return *(this->matrix_values+col+aligned_row_len*row);
}

int IVP_Great_Matrix_Many_Zero::print_great_matrix(const char *comment) const
{
    ivp_message("Matrix: %s\n", comment);
    int i;
    for(i=0; i<columns; i++){ // rows
	int j;
	for(j=0; j<columns; j++){ // columns
	    IVP_DOUBLE val = this->get_value(j, i);
	    if(IVP_Inline_Math::fabsd(val)<1e-20f){
		ivp_message("    0  ");
	    }else{
		ivp_message("%2.6g  ", val);
	    }
	}
	ivp_message("\n");
    }
    ivp_message("desired ");
    for(i=0;i<columns;i++) {
	ivp_message("%.6f ",desired_vector[i]);
    }
    ivp_message("\n");
    return 0; // for dbx
}

// #+# use vector FPU
void IVP_Great_Matrix_Many_Zero::mult_aligned() {
// can be optimized -> usage of vector FPU
    
	int j; // row
    // for each row
    for(j=0;j<columns;j++)    {
	int i; // col
	//walk through row
	IVP_DOUBLE left_side=0.0f;
	IVP_DOUBLE *source = &matrix_values[ j * aligned_row_len ];
	for(i=0;i<columns;i++)	{
	    left_side += source[i] * desired_vector[i];
	}
	result_vector[j] = left_side;
    }
}

void IVP_Great_Matrix_Many_Zero::mult()
{
    // from desired_vector to result_vector

    // @@OG superslow copypaste, can be optimized!! (rowoffset, revert loop, etc)
    
    int j; // row
    // for each row
    for(j=0;j<columns;j++)
    {
	int i; // col
	//walk through row
	IVP_DOUBLE left_side=0.0f;
	for(i=0;i<columns;i++)
	{
	    left_side+=matrix_values[j*columns+i]*desired_vector[i];
	}
	result_vector[j] = left_side;
    }
}

//can not be vector optimized because of indexed access
void IVP_Linear_Constraint_Solver::mult_active_x_for_accel() {
    int i,j;
    for(i=r_actives;i<n_variables;i++) {
	IVP_DOUBLE sum=0.0f;
	int row_of_A=actives_inactives_ignored[i];
	IVP_DOUBLE *row_vals=&full_solver_mat.matrix_values[ row_of_A * aligned_size ];
	int real_pos;
	for(j=0;j<r_actives;j++) {
	    real_pos=actives_inactives_ignored[j];
	    sum+= delta_f[real_pos]*row_vals[real_pos];
	}
	real_pos=actives_inactives_ignored[ignored_pos];
	sum+=row_vals[real_pos]*1.0f;
	delta_accel[row_of_A]=sum;
    }
}

void IVP_Linear_Constraint_Solver::mult_x_with_full_A_minus_b() {
    for(int i=0;i<n_variables;i++) {
	    IVP_DOUBLE *base_a=&full_A[i*aligned_size];
	    IVP_DOUBLE sum=IVP_VecFPU::fpu_large_dot_product(base_a,full_x,n_variables,IVP_TRUE);
	    temp[i] = sum-full_b[i];
    }
}

IVP_BOOL IVP_Linear_Constraint_Solver::numerical_stability_ok() {
    mult_x_with_full_A_minus_b();
    int k;
    IVP_DOUBLE val;
    IVP_DOUBLE worst_error=0.0f;
    for(k=0;k<r_actives;k++) {
	val=IVP_Inline_Math::fabsd(temp[actives_inactives_ignored[k]]);
	IVP_IF(1) {
	    if(worst_error<val) {
		worst_error=val;
	    }
	}
	if( val > TEST_EPS ) {
	    IVP_IF(debug_lcs) {
		printf("numerical_unstable %d %.10f should be zero\n",ignored_pos,val);
	    }
	    return IVP_FALSE;
	}
    }
    for(k=r_actives; k<n_variables; k++) {
	val=IVP_Inline_Math::fabsd( temp[actives_inactives_ignored[k]] - accel[actives_inactives_ignored[k]] );
	IVP_IF(1) {
	    if(worst_error<val) {
		worst_error=val;
	    }
	}
	if( val > TEST_EPS ) {
	    IVP_IF(debug_lcs) {
		printf("numerical_unstable %d %.10f should be %.10f\n",ignored_pos,temp[actives_inactives_ignored[k]],accel[actives_inactives_ignored[k]]);
	    }
	    return IVP_FALSE;
	}
    }
    IVP_IF(0) {
	printf("numerical_worst %.5e\n",worst_error);
    }
    return IVP_TRUE;
}

void IVP_Linear_Constraint_Solver::alloc_memory(IVP_U_Memory *my_mem) {
    actives_inactives_ignored=(int*)my_mem->get_mem(sizeof(int)*aligned_size);
    variable_is_found_at=(int*)my_mem->get_mem(sizeof(int)*aligned_size);

    accel =        (IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    delta_accel =  (IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    delta_f     =  (IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    reset_x     =  (IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    reset_accel =  (IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);

    lu_sub_solver.L_matrix  =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size*n_variables);
    lu_sub_solver.U_matrix  =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size*n_variables);
    lu_sub_solver.input_vec =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    lu_sub_solver.out_vec   =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    lu_sub_solver.temp_vec  =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    lu_sub_solver.mult_vec  =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);

    sub_solver_mat.matrix_values  =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size*n_variables);
    sub_solver_mat.desired_vector =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    sub_solver_mat.result_vector  =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    
    inv_mat.matrix_values  =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size*n_variables);
    inv_mat.desired_vector =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
    inv_mat.result_vector  =(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);

    IVP_IF(1){ // fill LU for fast align access
	for (int j = 0; j < n_variables; j++){
	    IVP_DOUBLE *L = &lu_sub_solver.L_matrix[ j * aligned_size ];
	    IVP_DOUBLE *U = &lu_sub_solver.U_matrix[ j * aligned_size ];
	    IVP_DOUBLE *S = &sub_solver_mat.matrix_values[ j * aligned_size ];
	    IVP_DOUBLE *M = &inv_mat.matrix_values[ j * aligned_size ];
	    for (int i = 0 /*n_variables*/; i< aligned_size; i++){
		L[i] = 0.0f;
		U[i] = 0.0f;
		S[i] = 0.0f;
		M[i] = 0.0f;
	    }
	}
	for (int k = n_variables; k< aligned_size; k++){
	    reset_accel[k] = 0.0f;
	}
    }
}

IVP_RETURN_TYPE IVP_Linear_Constraint_Solver::init_and_solve_lc(IVP_DOUBLE *A_in,IVP_DOUBLE *b_in,IVP_DOUBLE *result_vec_out,int var_num,int actives_at_begin,IVP_U_Memory *my_mem) {
    n_variables=var_num;
    aligned_size=(n_variables+IVP_VECFPU_SIZE-1) & IVP_VECFPU_MASK;

    alloc_memory(my_mem);
    
    r_actives=0;
    ignored_pos=0;
    SOLVER_EPS=1E-7f; //do not make this one bigger
    GAUSS_EPS=SOLVER_EPS;
    TEST_EPS=SOLVER_EPS*1000.0f;
    sub_solver_status=0;
    MAX_STEP_LEN=10000;
    
    lu_sub_solver.aligned_row_len=aligned_size;
    lu_sub_solver.n_sub=0;
    lu_sub_solver.MATRIX_EPS=SOLVER_EPS*10; // makes incrementel lu more often to fail but prevents from getting numerical inaccurate

    temp=lu_sub_solver.temp_vec;
    
    sub_solver_mat.MATRIX_EPS = GAUSS_EPS; // for gauss a more liberal EPS is enough (otherwise it may happen the unilateral solver to fail completely)
                                            // sorry, but for fdirection its not acceptable to have very large values
    
    inv_mat.MATRIX_EPS = SOLVER_EPS;
    
    IVP_IF(1) {
	debug_mat.matrix_values=(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size*var_num);
	debug_mat.desired_vector=(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
	debug_mat.result_vector=(IVP_DOUBLE*)my_mem->get_mem(sizeof(IVP_DOUBLE)*aligned_size);
	debug_mat.MATRIX_EPS = SOLVER_EPS;
    }
    
    full_solver_mat.columns = var_num;
    full_solver_mat.aligned_row_len = aligned_size;
    full_solver_mat.matrix_values = A_in;
    full_solver_mat.desired_vector = delta_f;
    full_solver_mat.result_vector = delta_accel;
    full_solver_mat.MATRIX_EPS = SOLVER_EPS;
    
    int i;
    for(i=0;i<aligned_size;i++) {
	actives_inactives_ignored[i]=i;
	variable_is_found_at[i] = i;
	result_vec_out[i] = 0.0f;
	accel[i] = -b_in[i];
    }

    full_A = A_in;
    full_b = b_in;
    full_x = result_vec_out;

    first_permute_index=0;
    second_permute_index=0;
    first_permute_ignored=0;
    second_permute_ignored=0;
    
    debug_no_lu_count=0;
    debug_lcs=0;
    
    IVP_IF(debug_lcs) {
	printf("\n***********************************************************\n");
	start_debug_lcs();
    }

    startup_setup( actives_at_begin );
    IVP_RETURN_TYPE ret_val = solve_lc();

    IVP_IF((debug_no_lu_count>0) && (debug_lcs)) {
	printf("sum_lu_failed %d\n",debug_no_lu_count);
    }
    
    IVP_IF(ret_val==IVP_OK) {
	int j;
	full_solver_mat.desired_vector=full_x;
	full_solver_mat.mult_aligned();
	for(j=0;j<n_variables;j++) {
	    if(full_solver_mat.result_vector[j] + 0.0001f < full_b[j]) {
		printf("linear_constraint_failed %d %f should be greater %f\n",j,full_solver_mat.result_vector[j],full_b[j]);
	    }
	}
	for(j=0;j<r_actives;j++) {
	    int index_full=actives_inactives_ignored[j];
	    IVP_DOUBLE diff=IVP_Inline_Math::fabsd(full_solver_mat.result_vector[index_full] - full_b[index_full]);
	    if(diff>0.0001f) {
		printf("linear_constraint_failed %d %f should be equal %f\n",j,full_solver_mat.result_vector[index_full],full_b[index_full]);
	    }
	}
    }
    
    full_solver_mat.result_vector = NULL;
    full_solver_mat.desired_vector = NULL;
    full_solver_mat.matrix_values = NULL;
    sub_solver_mat.result_vector = NULL;
    sub_solver_mat.desired_vector = NULL;
    sub_solver_mat.matrix_values = NULL;
    inv_mat.result_vector = NULL;
    inv_mat.desired_vector = NULL;
    inv_mat.matrix_values = NULL;
    IVP_IF(1) {
	debug_mat.result_vector = NULL;
	debug_mat.desired_vector = NULL;
	debug_mat.matrix_values = NULL;
    }
    
    return ret_val;
}


void IVP_Linear_Constraint_Solver::decrement_sub_solver( int sub_pos ) {
    if(sub_solver_status==0) {
	IVP_ASSERT( lu_sub_solver.n_sub == r_actives + 1 );
	if( lu_sub_solver.decrement_l_u( sub_pos ) != IVP_OK ) {
	    IVP_IF(1) {
		printf("\n\ndecrement_lu_failed_need_setup\n\n");
	    }
	    sub_solver_status=2;
	}
    } else {
	lu_sub_solver.n_sub--;
    }
}

void IVP_Linear_Constraint_Solver::increment_sub_solver() {
    int k;
    int new_var_orig = actives_inactives_ignored[ r_actives-1 ];
    if(sub_solver_status==0) {
	IVP_ASSERT( lu_sub_solver.n_sub == r_actives - 1);
	for(k=0;k<r_actives;k++) {
	    lu_sub_solver.U_matrix[ lu_sub_solver.n_sub * lu_sub_solver.aligned_row_len + k ] = full_A[ new_var_orig*aligned_size + actives_inactives_ignored[ k ] ]; 
	}
	for(k=0;k<r_actives-1;k++) {
	    lu_sub_solver.input_vec[k] = full_A[ aligned_size * actives_inactives_ignored[ k ] + new_var_orig ];
	}
	if( lu_sub_solver.increment_l_u() != IVP_OK ) {
	    IVP_IF(debug_lcs) {
		printf("\n\nincrement_lu_failed_need_setup\n\n");
	    }
	    sub_solver_status=2;
	}
    } else {
	lu_sub_solver.n_sub++;
    }
}

void IVP_Linear_Constraint_Solver::do_a_little_random_permutation() {
    // in case numeric gets unstable, neverending loops can occur
    // do some permutions to get fresh values ( in gauss the last variable gets zero, this "wrong" values is distributed alternatively )

    if( r_actives < 2) {
	return;
    }

    first_permute_index +=1;
    second_permute_index +=2;

    while(first_permute_index >= r_actives) {
	first_permute_index-=r_actives;
    }
    while(second_permute_index >= r_actives) {
	second_permute_index-=r_actives;
    }
    exchange_lcs_variables( first_permute_index, second_permute_index );

    first_permute_ignored +=1;
    second_permute_ignored +=2;

    int free_space=n_variables-ignored_pos-1;
    if(free_space<2) {
	return;
    }
    while(first_permute_ignored >= free_space) {
	first_permute_ignored-=free_space;
    }
    while(second_permute_ignored >= free_space) {
	second_permute_ignored-=free_space;
    }
    exchange_lcs_variables( first_permute_ignored+ignored_pos+1, second_permute_ignored+ignored_pos+1 );

}

void IVP_Linear_Constraint_Solver::move_not_necessary_actives_to_inactives() {
    int i;
    for( i=0;i<r_actives;i++ ) {
	if( full_x[ actives_inactives_ignored[i] ] < SOLVER_EPS ) {
	    //printf("juhu_removed_not_necessary_active %d full %d\n",i,actives_inactives_ignored[i]);
	    full_x[ actives_inactives_ignored[i] ]=0.0f;
	    exchange_lcs_variables( i, r_actives-1 );
	    r_actives--;
	    decrement_sub_solver( i );
	    i--;
	}
    }
}

void IVP_Linear_Constraint_Solver::increase_step_count(int *step_count) {
    (*step_count)++;
}

// actives_inactives_ignored : array to transform index from small variable space (that is growing) to full variable space
// first are active variables: their accel - val is 0.0f and x val is > 0.0f
// then inactives: their x val is 0.0f and accel val is > 0.0f
// last ignored: x and accel unknown, first of ignored is processed

// doctrin:
// when step size and variables are near to 0.0f than try to move corresponding variable into inactive variables (makes algorithm faster (sub matrix small) and robust (sub matrix not singular)) 
// but moving variables with x=0.0f into active cannot be avoided: sometimes it is necessary when moving first ignored into actives (as second step)
IVP_RETURN_TYPE IVP_Linear_Constraint_Solver::solve_lc() {
    int index; //var to go from sub space to full space
    int step_count=0;
    int nr_zero_steps=0; //to avoid endless loops
    int did_real_step=0; //real step means that active variables have changed
    int next_numeric_stability_test = IVP_NUMERIC_TEST_EVERY_N;

    //printf("\n\n\n\n\ndoing_lcs_ %d\n",n_variables);
    
    IVP_DOUBLE solver_eps = SOLVER_EPS;
    
    while(1) {
	if(did_real_step) {
	    increase_step_count(&step_count);
	}
	if(step_count > IVP_UNILATERAL_SOLVER_MAX_STEP) {
	    IVP_IF(1) {
	        printf("\n\ngiveup_stepcount_linear_constraint_solver\n\n\n");
	    }
	    return IVP_FAULT;
	}
        int limiting_var;
	
	if( next_numeric_stability_test == 0 ) {
	    if( (numerical_stability_ok() == IVP_FALSE) ) {
perform_full_setup:
		increase_step_count(&step_count);
    		if(step_count > IVP_UNILATERAL_SOLVER_MAX_STEP) {
		    IVP_IF(1) {
			printf("\n\ngiveup_stepcount_linear_constraint_solver\n\n\n");
		    }
		    return IVP_FAULT;
		}
		nr_zero_steps=0;
		if( full_setup() == IVP_FAULT ) {
		    return IVP_FAULT;
		}
	    }
	    next_numeric_stability_test = IVP_NUMERIC_TEST_EVERY_N;
	} else {
	    next_numeric_stability_test--;
	}
	
	if( ignored_pos >= n_variables ) {
	    return IVP_OK;
	}
	
	int ignored_full_index = actives_inactives_ignored[ ignored_pos ];

	IVP_IF(debug_lcs) {
	    int k;
	    for(k=0;k<r_actives;k++) {
		int idx=actives_inactives_ignored[k];
		IVP_IF( IVP_Inline_Math::fabsd( accel[idx] ) > 0.01f ) {
		    printf("incorrect_accel %f at %d  should be zero\n",accel[idx],idx);
		}
		IVP_IF( IVP_Inline_Math::fabsd( full_x[idx] < solver_eps )) {
		    if ( IVP_Inline_Math::fabsd( full_x[ignored_full_index] ) < solver_eps) {
			printf("active_var %d shouldnt be active but inactive\n",idx);
		    }
		}
		if( full_x[idx]<0.0f ) {
		    IVP_IF(1) {
			printf("active_var %d should be positive %f\n",idx,full_x[idx]);
		    }
		}
	    }
	}
	IVP_IF(1) {
	    int k;
	    for(k=r_actives;k<ignored_pos;k++) {
		int idx=actives_inactives_ignored[k];
		IVP_IF( IVP_Inline_Math::fabsd( full_x[idx] ) > 0.1f ) {
		    printf("incorrect_x %f at %d  should be zero\n",accel[idx],idx);
		}
		if(full_x[idx]<-0.001f) {
		    printf("incorrect_negative_x %f at %d  should be zero\n",accel[idx],idx);
		}
		if( accel[idx] < 0.0f ) {
		    printf("warning_inactive_neg accel %d %f\n",idx,accel[idx]);
		}	
	    }
	}
	
	IVP_IF(debug_lcs) {
	    debug_out_lcs();
	}

	IVP_IF( IVP_Inline_Math::fabsd( full_x[ ignored_full_index ] ) < solver_eps ) {
	    debug_test_all_values();
	}
	
	if(( sub_solver_status > 0)) {
	    if( (sub_solver_status == 1) && (did_real_step) ) {
		do_a_little_random_permutation();
		if(setup_l_u_solver()==IVP_OK) {
		    sub_solver_status=0; 
		} else {
		    IVP_IF(debug_lcs) {
		    printf("setup_lu_notok\n");
		    }
		}
	    } else {
		sub_solver_status = 1; //ok then next time
	    }
	}
	
	if( IVP_Inline_Math::fabsd( accel[ignored_full_index] ) < solver_eps ) {
	    // my processed variable has no acceleration and so could be an inactive var
	    // except the force x is greater 0.0f than we must move it to actives
	    limiting_var=ignored_pos;
	    if( IVP_Inline_Math::fabsd( full_x[ignored_full_index] ) < solver_eps ) {
		goto move_ignored_to_inactive;
	    } else {
		goto move_ignored_to_active;
	    }
	}
	
	if(accel[ignored_full_index] < 0.0f) {
	    
	    IVP_DOUBLE smallest_step;
	    
	    //first r vars are clamped
	    //compute x with Acc x = b
	    if(get_fdirection() == IVP_FAULT) {
		IVP_IF(debug_lcs) {
		    printf("lcs_fdirection_total_fail\n");
		}
		goto perform_full_setup;
	    }
	    delta_f[ ignored_full_index ] = 1.0f;
	    
	    //delta_f is now filled in original var space

	    IVP_IF(0) {
		printf("fullmat:\n");
		full_solver_mat.print_great_matrix("");
		printf("\n");
	    }
	    	    
	    //full_solver_mat.mult();  //delta_accel = A * delta_f
	    //delta_accel is now filled in original var space
	    mult_active_x_for_accel();

	    int i;
	    for(i=0;i<r_actives;i++) {
		int index_full = actives_inactives_ignored[i];
		delta_accel[index_full] = 0.0f; //just to overwrite rounding-errors
	    }
	    
	    IVP_IF(0) {
		for(int j=0;j<r_actives;j++) {
		    int full_index = actives_inactives_ignored[ j ];
		    printf("should be zero %f\n",delta_accel[full_index]);
		}
	    }
	    
	    // now find smallest steps
	    
	    // step to make clamped active out of ignored_pos
	    IVP_IF( accel[ ignored_full_index ] > 0.0f ) {
		printf("errror_accel_ignored_negativ\n");
	    }
	    if( -accel[ ignored_full_index ] < delta_accel[ ignored_full_index ] * MAX_STEP_LEN ) {
		smallest_step = - accel[ignored_full_index] / delta_accel[ ignored_full_index ];
		IVP_ASSERT( smallest_step > 0.0f );
		limiting_var = ignored_pos;
	    } else {
		limiting_var = -1;
		smallest_step = P_DOUBLE_MAX;
	    }

#if 0
	    if( delta_accel[ ignored_full_index ] > solver_eps ) {
		smallest_step = - accel[ignored_full_index] / delta_accel[ ignored_full_index ];
		IVP_ASSERT( smallest_step > 0.0f );
		limiting_var = ignored_pos;
	    } else {
		limiting_var = -1;
		smallest_step = P_DOUBLE_MAX;
	    }
#endif
	    // smallest step of clamped
	    for(i=0;i<r_actives;i++) {
		index = actives_inactives_ignored[i];
		IVP_DOUBLE del_f = delta_f[ index ];
		if( del_f < - solver_eps ) {
		    IVP_DOUBLE local_step = - full_x[ index ] / del_f;
		    if( (IVP_Inline_Math::fabsd( local_step ) < solver_eps) && ( full_x[ index ] < solver_eps ) ) {
			limiting_var = i;
			smallest_step = local_step;
			goto limiting_var_found;
		    }
		    if( local_step < smallest_step + solver_eps) { //make it easier to move from active to inactive
			smallest_step = local_step;
			limiting_var = i;
		    }
		}
	    }

	    // smallest step of unclamped
	    for(i=r_actives;i<ignored_pos;i++) {
		index = actives_inactives_ignored[i];
		IVP_DOUBLE del_a = delta_accel[ index ];
		if( del_a < -SOLVER_EPS ) {
		    IVP_DOUBLE local_step = - accel[ index ] / del_a;
		    if( local_step < smallest_step - solver_eps ) { // make it harder to move from inactive to active
			IVP_IF(debug_lcs) {
			    printf("violate_inactive: accel %.4f  delta %.4f\n",accel[index],del_a);
			}			
			// we prefer making inactives out of actives instead other way round if both is possible ( both steps are nearly zero )
			// uuups this is all wrong
			if( (IVP_Inline_Math::fabsd( local_step ) > solver_eps ) || (limiting_var==ignored_pos) || 1) {
			    smallest_step = local_step;
			    limiting_var = i;
			}
		    }
		}
	    }

	    if(smallest_step < 0.0f) {
		IVP_IF(1) {
		    printf("error_stepsize_negative %f\n",smallest_step);
		}
		smallest_step=0.0f;
	    }	
	    
	    if( limiting_var < 0 ) {
		return IVP_FAULT;
	    }

	    IVP_IF(debug_lcs) {
		printf("smallest_step_is %f\n",smallest_step);
	    }

limiting_var_found:
	    if(smallest_step > MAX_STEP_LEN) {
		goto perform_full_setup;
	    }
	    if(smallest_step < SOLVER_EPS) {
		nr_zero_steps++;
		if(nr_zero_steps > (n_variables>>1) +2 ) { //heuristic
		    goto perform_full_setup; //does a little random permutation, we hope next time no endless loop
		}
	    } else {
		nr_zero_steps=0;
	    }
	    did_real_step=1;
	    update_step_vars( smallest_step );
	    
	    if(limiting_var < r_actives) {
		// one active var becomes inactive
		
		IVP_IF(debug_lcs) {
		    printf("\nactive %d becomes inactive (relp %d)\n\n",actives_inactives_ignored[limiting_var],limiting_var);
		}
		
		int indx=actives_inactives_ignored[limiting_var];
		IVP_ASSERT( (IVP_Inline_Math::fabsd(full_x[indx]) < 0.001f) || (IVP_Inline_Math::fabsd( smallest_step ) < solver_eps) );
		full_x[indx]=0.0f;
		r_actives--;
		//exchange_activ_inactiv(limiting_var);
		exchange_lcs_variables(r_actives,limiting_var);
		decrement_sub_solver( limiting_var );
	    } else {
		if(limiting_var < ignored_pos) {
		    // one inactive var becomes active
		    if(smallest_step > SOLVER_EPS) {
			move_not_necessary_actives_to_inactives();
		    }

		    IVP_IF(debug_lcs) {
			printf("\ninactive %d becomes active (relp %d)\n\n",actives_inactives_ignored[limiting_var],limiting_var);
		    }
		    int indx=actives_inactives_ignored[limiting_var];
		    IVP_ASSERT( IVP_Inline_Math::fabsd(accel[indx]) < 0.001f );
		    accel[indx]=0.0f;
		    //exchange_activ_inactiv(limiting_var);
		    exchange_lcs_variables(r_actives,limiting_var);
		    r_actives++;
		    increment_sub_solver();
		} else {
move_ignored_to_active:
		    // ignored_pos comes to the active variables		    
		    move_not_necessary_actives_to_inactives();		    
		    IVP_IF(debug_lcs) {
			printf("\nignored %d becomes active (relp %d)\n\n",actives_inactives_ignored[ignored_pos],ignored_pos);
		    }
		    int indx=actives_inactives_ignored[limiting_var];
		    IVP_ASSERT( IVP_Inline_Math::fabsd(accel[indx]) < 0.001f );
		    accel[indx]=0.0f;
		    if(full_x[indx]<0.0f) { //really necessary ?
			full_x[indx]=0.0f;
		    }
		    //exchange_activ_inactiv(limiting_var);
		    exchange_lcs_variables(r_actives,limiting_var);
		    ignored_pos++;
		    r_actives++;
		    if(ignored_pos<n_variables) {
			increment_sub_solver();
		    } else {
			next_numeric_stability_test=0;
		    }
		    IVP_IF(0) {
			debug_test_all_values();
		    }
		}
	    }
	} else {
move_ignored_to_inactive:	    
	    // ignored_pos comes to the inactives variables
	    did_real_step=0; //active variables are unchanged
	    IVP_IF(debug_lcs) {
		printf("ignored %d becomes inactive (relp %d)\n\n",actives_inactives_ignored[ignored_pos],ignored_pos);
	    }
	    int indx=actives_inactives_ignored[ignored_pos];
	    IVP_IF( !(IVP_Inline_Math::fabsd(full_x[indx] ) < 0.001f) ) {
		printf("error_move_ignored_to_inactive_x_not_zero\n");
	    }
	    
	    full_x[indx]=0.0f;
	    if( accel[indx] < 0.0f) { //really necessary ?
		accel[indx]=0.0f;
	    }
	    ignored_pos++;
	    if(ignored_pos >= n_variables) {
		next_numeric_stability_test=0;
	    }	    
	    //reset_values();
	}
    }

    IVP_IF(debug_lcs) {
	debug_out_lcs();
    }
    
    return IVP_OK;
}

//both numbers are in sub-space
void IVP_Linear_Constraint_Solver::exchange_lcs_variables( int first_nr, int second_nr ) {
    IVP_IF(1) {
	IVP_ASSERT(variable_is_found_at[ actives_inactives_ignored[ second_nr ] ] == second_nr );
	IVP_ASSERT(variable_is_found_at[ actives_inactives_ignored[ first_nr ] ] == first_nr );
    }
    IVP_ASSERT( first_nr >= 0 );
    IVP_ASSERT( second_nr >= 0 );
    IVP_ASSERT( first_nr < n_variables );
    IVP_ASSERT( second_nr < n_variables );
    int full_space_0,full_space_1;
    full_space_0=actives_inactives_ignored[second_nr];
    full_space_1=actives_inactives_ignored[first_nr];
    actives_inactives_ignored[first_nr]=full_space_0;
    actives_inactives_ignored[second_nr]=full_space_1;
    variable_is_found_at[full_space_0]=first_nr;
    variable_is_found_at[full_space_1]=second_nr;
}
//please merge these two functions !!!!
void IVP_Linear_Constraint_Solver::exchange_activ_inactiv( int change_var ) {
    IVP_IF(1) {
	IVP_ASSERT(variable_is_found_at[ actives_inactives_ignored[ change_var ] ] == change_var );
	IVP_ASSERT(variable_is_found_at[ actives_inactives_ignored[ r_actives ] ] == r_actives );
    }
    int first_inactiv=r_actives;
    int full_space_0,full_space_1;
    full_space_0=actives_inactives_ignored[change_var];
    full_space_1=actives_inactives_ignored[first_inactiv];
    actives_inactives_ignored[first_inactiv]=full_space_0;
    actives_inactives_ignored[change_var]=full_space_1;
    variable_is_found_at[full_space_0]=first_inactiv;
    variable_is_found_at[full_space_1]=change_var;
}

//optimization: do not go from 0 to n_variables
//              make two loops: one for actives, one for others
void IVP_Linear_Constraint_Solver::update_step_vars( IVP_DOUBLE step_size ) {
    int i;
    IVP_IF(debug_lcs) {
	printf("deltaa  ");
    }
    for( i = 0; i < n_variables; i++) {
	IVP_IF(debug_lcs) {
	    printf("%f ",delta_accel[i]);
	}
	accel[ i ] = accel[i] + step_size * delta_accel[ i ]; //Vector Optimization possible
	full_x[ i ] = full_x[ i ] + step_size * delta_f[ i ];           //dito
    }
    IVP_IF(debug_lcs) {
	printf("\n");
    }

    //in some cases (e.g. overriding an inactive) acceleration of inactives gets negative
    for( i=r_actives; i<ignored_pos; i++ ) {
	if(accel[actives_inactives_ignored[i]] < 0.0f) {
	    accel[actives_inactives_ignored[i]]=0.0f;  
	}
    }
    //same with actives
    for(i=0;i<r_actives;i++) {
	if(full_x[actives_inactives_ignored[i]] < 0.0f) {
	    full_x[actives_inactives_ignored[i]]=0.0f;
	}
    }
}
    
IVP_RETURN_TYPE IVP_Linear_Constraint_Solver::get_fdirection() {
    // clear all
    int i,k;
    for(i=0;i<aligned_size;i++) {
        this->delta_f[ i ] = 0.0f;
    }

    if(r_actives==0) {
	return IVP_OK;
    }

    IVP_RETURN_TYPE ret_val = IVP_FAULT;
    

    if(sub_solver_status == 0) {
	int ignored_index = actives_inactives_ignored[ ignored_pos ];

	for(k=0;k<r_actives;k++) {
	    int index=actives_inactives_ignored[k];
	    lu_sub_solver.input_vec[k]= - this->full_A[index * aligned_size + ignored_index];
	}
	lu_sub_solver.solve_lin_equ();
	ret_val = IVP_OK;
	
	IVP_IF(debug_lcs) {
	    sub_solver_mat.columns = r_actives;
	    sub_solver_mat.calc_aligned_row_len();
	    sub_solver_mat.MATRIX_EPS = GAUSS_EPS * 0.001f; //we know our matrix is stable, so it is save to raise EPS
	    inv_mat.columns = r_actives;
	    debug_mat.columns = r_actives;
	    
	    for(i=0;i<r_actives;i++) {
		int index=actives_inactives_ignored[i];
		sub_solver_mat.desired_vector[i]= - this->full_A[index * aligned_size + ignored_index];
		debug_mat.desired_vector[i]= - this->full_A[index * aligned_size + ignored_index];
		int j;
		for(j=0;j<r_actives;j++) {
		    int index2=actives_inactives_ignored[j];
		    sub_solver_mat.matrix_values[i*sub_solver_mat.aligned_row_len + j] = this->full_A[ index * aligned_size + index2 ];
		    inv_mat.matrix_values[i*r_actives + j] = this->full_A[ index * aligned_size + index2 ];
		}
	    }
	    IVP_RETURN_TYPE ret_val2 = sub_solver_mat.solve_great_matrix_many_zero();
	    IVP_RETURN_TYPE ret_val3 = IVP_FAULT; //inv_mat.invert(&debug_mat);
	    if( ret_val2 == IVP_OK ) {
		for( i=0;i<r_actives;i++ ) {
		    IVP_DOUBLE diff=lu_sub_solver.out_vec[i]-sub_solver_mat.result_vector[i];
		    if(IVP_Inline_Math::fabsd(diff) > 0.001f) {
		      IVP_IF(debug_lcs) {
			printf("gauss_lu_test_fail %f %f should be equal\n",lu_sub_solver.out_vec[i],sub_solver_mat.result_vector[i]);
		      }
		    }
		}
	    } else {
	      IVP_IF(debug_lcs) {
		printf("gauss_lu_test_gauss_invalid\n");
	      }
	    }
	    if( ret_val3 == IVP_OK ) {
		debug_mat.mult();
		for( i=0;i<r_actives;i++ ) {
		    IVP_DOUBLE diff=lu_sub_solver.out_vec[i]-debug_mat.result_vector[i];
		    if(IVP_Inline_Math::fabsd(diff) > 0.001f) {
		      IVP_IF(debug_lcs) {
			printf("inv_lu_test_fail %f %f should be equal\n",lu_sub_solver.out_vec[i],debug_mat.result_vector[i]);
		      }
		    }
		}
	    }
	    sub_solver_mat.MATRIX_EPS = GAUSS_EPS;
	}
    } else {
	IVP_IF((debug_lcs)) {
	    printf("sub_lu_is_off %d\n",r_actives);
	}
	debug_no_lu_count++;
	sub_solver_mat.columns = r_actives;
	sub_solver_mat.calc_aligned_row_len();
	int ignored_index = actives_inactives_ignored[ ignored_pos ];
    
	for(i=0;i<r_actives;i++) {
	    int index=actives_inactives_ignored[i];
	    sub_solver_mat.desired_vector[i]= - this->full_A[index * aligned_size + ignored_index];

	    int j;
	    for(j=0;j<r_actives;j++) {
		int index2=actives_inactives_ignored[j];
		sub_solver_mat.matrix_values[i*sub_solver_mat.aligned_row_len + j] = this->full_A[ index * aligned_size + index2 ];

	    }
	}
	ret_val = sub_solver_mat.solve_great_matrix_many_zero();
	for( i=0;i<r_actives;i++ ) {
	    lu_sub_solver.out_vec[i]=sub_solver_mat.result_vector[i]; //Vector Optimization possible
	}
    }
    
    //copy clamped
    for(i=0;i<r_actives;i++) {
	this->delta_f[ actives_inactives_ignored[i] ] = lu_sub_solver.out_vec[i];
    }

    this->delta_f[ actives_inactives_ignored[ignored_pos] ] = 1.0f;
    
    return ret_val;
}

void IVP_Linear_Constraint_Solver::start_debug_lcs() {
    printf("b_vals ");
    int i;
    int j;
    for(i=0;i<n_variables;i++) {
	printf("%.4f ",full_b[i]);
    }
    printf("\n");
    
    printf("matrix:\n");
    for(i=0;i<n_variables;i++) {
	for(j=0;j<n_variables;j++) {
	    printf("%.4f ",full_A[i*aligned_size + j]);
	}
	printf("\n");
    }
    printf("\n");
}

void IVP_Linear_Constraint_Solver::debug_out_lcs() {
    printf("r_actives %d  ignored_nr %d\n",r_actives,ignored_pos);
    int i;

    if(ignored_pos>n_variables) {
	ignored_pos=n_variables;
    }
    
    printf("actives_inactives  ");
    for(i=0;i<r_actives;i++) {
	printf("%d ",actives_inactives_ignored[i]);
    }
    printf("  ");
    for(i=r_actives;i<ignored_pos;i++) {
	printf("%d ",actives_inactives_ignored[i]);
    }
    printf("  ");
    for(i=ignored_pos;i<n_variables;i++) {
	printf("%d ",actives_inactives_ignored[i]);
    }
    printf("\n");
    
    printf("variable_is_found  ");
    for(i=0;i<r_actives;i++) {
	printf("%d ",variable_is_found_at[i]);
    }
    printf("  ");
    for(i=r_actives;i<ignored_pos;i++) {
	printf("%d ",variable_is_found_at[i]);
    }
    printf("  ");
    for(i=ignored_pos;i<n_variables;i++) {
	printf("%d ",variable_is_found_at[i]);
    }
    printf("\n");

    printf("x-vals ");
    for(i=0;i<n_variables;i++) {
	printf("%.4f ",full_x[i]);
    }
    printf("\n");

    full_solver_mat.desired_vector = full_x;
    full_solver_mat.mult_aligned(); // temporary missuse
    full_solver_mat.desired_vector = delta_f;
    
    printf("accel1 ");
    for(i=0;i<n_variables;i++) {
	printf("%.4f ",full_solver_mat.result_vector[i] - full_b[i]);
    }
    printf("\n");

    printf("incr_accel ");
    for(i=0;i<n_variables;i++) {
	printf("%.4f ",accel[i]);
    }
    printf("\n\n");
  
}

//cannot be vector optimized because of indexed access
IVP_RETURN_TYPE IVP_Linear_Constraint_Solver::setup_l_u_solver() {
    int i;
    IVP_IF(0) {
	printf("setting_up_lu\n");
    }
    
    for(i=0; i <r_actives;i++) {
	    int index = actives_inactives_ignored[i];
	    //sub_solver_mat.desired_vector[i]= b[index];
	    lu_sub_solver.input_vec[i]=full_b[index];
	    IVP_DOUBLE *source = & this->full_A[ index * aligned_size ];
	    IVP_DOUBLE *dest = & lu_sub_solver.U_matrix[ i * lu_sub_solver.aligned_row_len ];
	    for(int j=0;j<r_actives;j++) {
	        int index2=actives_inactives_ignored[j];
		dest[j] = source[index2];
	    }
    }
    
    return lu_sub_solver.l_u_decomposition_with_pivoting();
}

void IVP_Linear_Constraint_Solver::lcs_bubble_sort_x_vals() {
    int r1=0;
    int r2;
    while(1) {
	r2=r1+1;
	if(r2>=r_actives) {
	    break;
	}
	if( full_x[ actives_inactives_ignored[ r1 ] ] < full_x[ actives_inactives_ignored[ r2 ] ] ) {
	    exchange_lcs_variables( r1, r2 );
	    int r_t = r1;
	    while( (r_t > 0) && ( full_x[ actives_inactives_ignored [r_t-1] ] < full_x[ actives_inactives_ignored[ r_t ] ] ) ) {
		exchange_lcs_variables( r_t, r_t-1 );
		r_t--;
	    }
	}
	r1=r2;
    }

    IVP_IF(1) {
	int i;
	for(i=1;i<r_actives;i++) {
	    if( full_x[ actives_inactives_ignored[ i-1 ] ] < full_x[ actives_inactives_ignored[ i ] ] ) {
		printf("lcs_bubble_failed\n");
	    }
	}
    }
}

void IVP_Linear_Constraint_Solver::get_values_when_setup() {
    full_solver_mat.result_vector=reset_accel;
    full_solver_mat.desired_vector=reset_x; //temporary misuse
    full_solver_mat.mult_aligned();
    full_solver_mat.result_vector=delta_accel;
    full_solver_mat.desired_vector=delta_f;

    int i;
    {
	for(i=0;i<n_variables;i++) {
	    reset_accel[i]-=full_b[i];
	}
    }

    for(i=0;i<r_actives;i++) {
	reset_accel[actives_inactives_ignored[i]]=0.0f; //just to remove rounding errors
    }
    
    for(i=n_variables-1;i>=0;i--) {
	accel[i]=reset_accel[i];
	full_x[i]=reset_x[i];
    }

}

void IVP_Linear_Constraint_Solver::startup_setup(int try_actives) {
    r_actives=try_actives;
    aligned_sub_size=(r_actives + IVP_VECFPU_SIZE-1) & IVP_VECFPU_MASK;
    ignored_pos=try_actives;
    lu_sub_solver.n_sub = r_actives;

    int i;
    for(i=0;i<aligned_size;i++) {
	reset_x[i]=0.0f;
    }
    if(setup_l_u_solver() == IVP_OK) {
	sub_solver_status = 0;
        //input vec for lu_sub_solver is filled within setup_l_u_solver

	while(1) {
begin_of_while:	    
	    //solution_not_found=0;
	    lu_sub_solver.solve_lin_equ();
	    for(i=0;i<r_actives;i++) {
		int index=actives_inactives_ignored[i];
		reset_x[index]=lu_sub_solver.out_vec[i];
	    }
	    get_values_when_setup(); //fills x[i] and accel[i]

	    for(i=r_actives-1;i>=0;i--) {
		int index=actives_inactives_ignored[i];
		if(full_x[ index ] < 0.0f){
		    full_x[ index ] = 0.0f;
		    reset_x[ index ] = 0.0f;
		    r_actives--;
		    ignored_pos--;
		    exchange_lcs_variables(r_actives,i);
		    decrement_sub_solver(i);
		    if(sub_solver_status > 0) {
			for(i=0;i<n_variables;i++) {
			   accel[i]=0.0f;
			   full_x[i]=0.0f;
			}
			goto fast_setup_failed;
		    } else {
			for(i=0;i<r_actives;i++) {
			    index=actives_inactives_ignored[i];
			    lu_sub_solver.input_vec[i]=full_b[index];
			}
			goto begin_of_while;
		    }
		}
	    }
	    break;
	}
    } else {
fast_setup_failed:	
	// sorry fast setup failed
	r_actives=0;
	ignored_pos=0;
	lu_sub_solver.n_sub=0;
    }
}

IVP_RETURN_TYPE IVP_Linear_Constraint_Solver::full_setup() {
    IVP_IF(0) {
	printf("doing_fulll_setup\n");
    }
    
    int i;
    lu_sub_solver.n_sub = r_actives;
    
    for(i=0;i<aligned_size;i++) {
	reset_x[i]=0.0f;
    }

    do_a_little_random_permutation();
    if( setup_l_u_solver() == IVP_OK ) {
	sub_solver_status = 0;
        //input vec for lu_sub_solver is filled within setup_l_u_solver
	lu_sub_solver.solve_lin_equ();
	for(i=0;i<r_actives;i++) {
	    int index=actives_inactives_ignored[i];
	    reset_x[index]=lu_sub_solver.out_vec[i];
	}
    } else {
	sub_solver_status = 2;
	IVP_IF(debug_lcs) {
	    printf("setup_lu_failed\n");
	}

	lcs_bubble_sort_x_vals(); //move actives with lowest x to the end (matrix is singular and last vars get a zero)
	                          //but giving the lowest x's the zeros is just a heuristic
	do_a_little_random_permutation();
	
	sub_solver_mat.columns = r_actives;
	sub_solver_mat.calc_aligned_row_len();
	
	for(i=0;i<r_actives;i++) {
	    int index=actives_inactives_ignored[i];
	    sub_solver_mat.desired_vector[i]= full_b[index];

	    int j;
	    for(j=0;j<r_actives;j++) {
		int index2=actives_inactives_ignored[j];
		sub_solver_mat.matrix_values[i*sub_solver_mat.aligned_row_len + j] = this->full_A[ index * aligned_size + index2 ];

	    }
	}
	IVP_RETURN_TYPE ret_val = sub_solver_mat.solve_great_matrix_many_zero();

	if(ret_val!=IVP_OK) {
	    IVP_IF(1) {
		printf("error_setup_gauss_failed_too\n");
	    }
	    return ret_val;
	}
    
	for(i=0;i<r_actives;i++) {
	    int index=actives_inactives_ignored[i];
	    reset_x[index]=sub_solver_mat.result_vector[i];
	}
    }

    get_values_when_setup();
    
    if( full_setup_test_ranges() > 0 ) {
	IVP_IF(debug_lcs) {
	    printf("\ncleared_some_illegal_vars\n\n");
	}
	return full_setup();
    } else {
	
    }
    
    return IVP_OK;
    //optimization:
    //fill directly in data, not versus reset_*
}

void IVP_Linear_Constraint_Solver::move_variable_to_end( int var_nr ) {
    int i;
    for(i=var_nr+1;i<n_variables;i++) {
	exchange_lcs_variables( i-1, i );
    }
}	

// returns number of removed actives
int IVP_Linear_Constraint_Solver::full_setup_test_ranges() {
    int illegal_vars=0;
    int i;
    for(i=0;i<r_actives;i++) {
	if( full_x[actives_inactives_ignored[i]] < SOLVER_EPS ) {
	    if( full_x[actives_inactives_ignored[i]] > -SOLVER_EPS ) {
		full_x[actives_inactives_ignored[i]] = 0.0f;
		exchange_lcs_variables( i, r_actives-1 );
	    } else {
		move_variable_to_end( i );
		illegal_vars++;
		ignored_pos--;
	    }
	    i--;
	    r_actives--;
	    lu_sub_solver.n_sub--;
	    sub_solver_status=1;
	}
    }
    for(i=r_actives;i<ignored_pos;i++) {
	if( accel[actives_inactives_ignored[i]] < 0.0f ) {
	    move_variable_to_end( i );
	    i--;
	    ignored_pos--;
	}
    }
    return illegal_vars;
}

void IVP_Linear_Constraint_Solver::debug_test_all_values() {
    sub_solver_mat.columns = r_actives;
    int i;

    for(i=0;i<aligned_size;i++) {
	reset_x[i] = full_x[i];
    }


    full_solver_mat.result_vector = reset_accel;
    full_solver_mat.desired_vector = reset_x; //temporary misuse
    full_solver_mat.mult_aligned();
    full_solver_mat.result_vector = delta_accel;
    full_solver_mat.desired_vector = delta_f;

    for(i=0;i<aligned_size;i++) {
	reset_accel[i]-=full_b[i];
    }

    IVP_IF(debug_lcs) {
	for(i=0;i<n_variables;i++) {
	    IVP_DOUBLE diff=IVP_Inline_Math::fabsd( reset_accel[i] - accel[i] );
	    if( diff > 0.001f ) {
		printf("accel_violated at %d %f should be %f\n",i,reset_accel[i],accel[i]);
	    }
	    diff=IVP_Inline_Math::fabsd( full_x[i] - reset_x[i] );
	    if( diff > 0.001f ) {
		printf("x_violated at %d\n",i);
	    }
	}
    }
}

#if 1
IVP_RETURN_TYPE IVP_Great_Matrix_Many_Zero::lu_crout(int *index_vec,IVP_DOUBLE *sign)
{
    /* use desired_vector to store scalings */
    
   int    i, imax = 0, j, k, zeros = 0 ;
   IVP_DOUBLE   sum ;
   IVP_DOUBLE     big, temp ;

   int n_columns = this->columns;
   
   *sign = 1.0f ;

   //Find implicit scaling factors

   for ( i = 0; i < n_columns ; i++ ) {
      big = 0.0f;
      for ( j = 0; j < n_columns; j++ ) {
         temp = IVP_Inline_Math::fabsd( (IVP_DOUBLE) matrix_values[i*n_columns+j] ) ;
         if ( temp > big )
            big = temp ;
      } 
      desired_vector[i] = ( big == 0.0f ? 0.0f : 1.0f / big ) ;
   }

   for ( j = 0; j < n_columns; j++ ) {
      /*************************************
       Run down jth column from top to diag,
       to form the elements of U.
      **************************************/
      for ( i = 0; i < j; i++ ) {
         sum = matrix_values[i*n_columns+j] ;
         for ( k = 0; k < i; k++ ) {
	     sum -= matrix_values[i*n_columns+k] * matrix_values[k*n_columns+j] ; }
         matrix_values[i*n_columns+j] = sum ;
      }
      
      /******************************************
       Run down jth subdiag to form the residuals
       after the elimination of the first j-1
       subdiags.  These residuals divided by the
       appropriate diagonal term will become the
       multipliers in the elimination of the jth.
       subdiag. Find index of largest scaled term
       in imax.
      *******************************************/
      big = 0.0f ;
      for ( i = j; i < n_columns; i++ ) {
         sum = matrix_values[i*n_columns+j] ;
         for ( k = 0; k < j; k++ )
            sum -= matrix_values[i*n_columns+k] * matrix_values[k*n_columns+j] ;
         matrix_values[i*n_columns+j] = sum ;
         temp = desired_vector[i] * ( IVP_Inline_Math::fabsd( sum ) ) ;
         if ( temp >= big ) {
            big = temp ;
            imax = i ;
         } 
      }  
      
      //Permute current row with imax
    
      if ( j != imax ) {
         for ( k = 0; k < n_columns; k++ ) {
            temp = matrix_values[ imax*n_columns+ k ] ;
            matrix_values[ imax*n_columns + k ] = matrix_values[j*n_columns+k] ;
            matrix_values[j*n_columns+k] = temp ;
         } 
         *sign = - *sign ;
         desired_vector[ imax ] = desired_vector[j] ;
      }
      index_vec[j] = imax;
      
      //If diag term is not zero divide subdiag to form multipliers.
      
      if ( IVP_Inline_Math::fabsd( matrix_values[j*n_columns+j] ) < MATRIX_EPS ) 
         zeros++;
      else if ( j != n_columns-1 ) {
         temp = 1.0f / matrix_values[j*n_columns+j] ;
         for ( i = j+1; i < n_columns; i++ )
            matrix_values[i*n_columns+j] *= temp;
      } 
   } 
   if ( zeros ) {
      return IVP_FAULT;
   }
   return IVP_OK;
}

IVP_RETURN_TYPE IVP_Great_Matrix_Many_Zero::lu_inverse(IVP_Great_Matrix_Many_Zero *matrix_out,int *index_vec) {
// solve the Identity matrix column for column to get inverse
    int i,j;
    for(i=0;i<columns;i++) {
	for(j=0;j<columns;j++) {
	    desired_vector[j]=0.0f;
	}
	desired_vector[i]=1.0f;

	if( this->lu_solve(index_vec) == IVP_OK ) {
	    for(j=0;j<columns;j++) {
		matrix_out->matrix_values[j*columns+i]=this->result_vector[j];
	    }
	} else {
	    return IVP_FAULT;
	}
    }
    return IVP_OK;
}

IVP_RETURN_TYPE IVP_Great_Matrix_Many_Zero::lu_solve(int *index_vec) {
   int  i, nonzero=-1, iperm, j ;
   IVP_DOUBLE sum, zero = 0.0f ;

   int n_columns = this->columns;
   
   //check for zero diagonals
   
   for ( i = 0; i < n_columns ; i++ ) {
       if ( IVP_Inline_Math::fabsd( matrix_values[i*n_columns+i] ) < MATRIX_EPS ) {
         return IVP_FAULT ;
       }
   }
   
   //transform b allowing for leading zeros

   for ( i = 0; i < n_columns; i++ ) {
      iperm = index_vec[i] ;
      sum =  desired_vector[ iperm ] ;
      desired_vector[ iperm ] = desired_vector[i] ;
      if ( nonzero >= 0 )
	  for ( j = nonzero-1; j <= i-1; j++ ) {
            sum -= matrix_values[i*n_columns+j] * desired_vector[j] ;
	  }
      else if ( sum != zero )
         nonzero = i+1 ;
      desired_vector[i] = sum ;
   }

  
   //backsubstitution

   for ( i = n_columns-1; i >= 0; i-- ) {
      sum = desired_vector[i] ;
      for ( j = i+1; j < n_columns; j++ ) {
         sum -= matrix_values[i*n_columns+j] * desired_vector[j] ;
      }
      desired_vector[i] = ( sum / matrix_values[i*n_columns+i] ) ;
      result_vector[i]=desired_vector[i]; // in place of desired_vector is possible!
   }

   return IVP_OK;
}

// destroys matrix this
// also uses and destroys this->desired_vector and this->result_vector
IVP_RETURN_TYPE IVP_Great_Matrix_Many_Zero::invert(IVP_Great_Matrix_Many_Zero *dest)
{
    // NOTE: dest memory already has to be allocated
    
    IVP_ASSERT(dest->columns == this->columns);

#if defined(IVP_NO_ALLOCA)
    int index_vec[IVP_MAX_GREAT_MATRIX_SIZE];
    IVP_ASSERT( columns < IVP_MAX_GREAT_MATRIX_SIZE);
#else    
    int *index_vec=(int*)alloca( columns*sizeof(IVP_DOUBLE) );
#endif

    IVP_DOUBLE sign;

    if( lu_crout(index_vec,&sign) == IVP_FAULT ) {
	return IVP_FAULT;
    }
    
    return lu_inverse(dest,index_vec);
}

#endif


IVP_RETURN_TYPE IVP_Incr_L_U_Matrix::normize_row_L(int row_nr) {
    IVP_DOUBLE *base=&L_matrix[row_nr*aligned_row_len];
    IVP_DOUBLE val=base[row_nr];
    if(IVP_Inline_Math::fabsd(val)<MATRIX_EPS) {
	return IVP_FAULT;
    }
    IVP_DOUBLE inv_val=1.0f/val;
    
    IVP_VecFPU::fpu_multiply_row(&base[0],inv_val,n_sub,IVP_FALSE); //should be TRUE
    
    base[row_nr]=1.0f;
    return IVP_OK;
}

IVP_RETURN_TYPE IVP_Incr_L_U_Matrix::normize_row(int row_nr) {
    int row_base = row_nr * aligned_row_len;
    IVP_DOUBLE val=U_matrix[row_base + row_nr];
    if(IVP_Inline_Math::fabsd(val)<MATRIX_EPS) {
	return IVP_FAULT;
    }
    IVP_DOUBLE inv_val = 1.0f / val;
    IVP_VecFPU::fpu_multiply_row(&L_matrix[row_base],inv_val,n_sub,IVP_FALSE);
    IVP_VecFPU::fpu_multiply_row(&U_matrix[row_base+row_nr+1],inv_val,n_sub-row_nr-1,IVP_FALSE);
    
    U_matrix[row_base + row_nr] = 1.0f;
    return IVP_OK;
}

void IVP_Incr_L_U_Matrix::exchange_rows_l_u(int pivot_col,int exchange) {
    int base_a = pivot_col*aligned_row_len;
    int base_b = exchange *aligned_row_len;

#if 0 /* single FPU */  
    IVP_DOUBLE temp;
    int i;
    for(i=n_sub-1;i>=pivot_col;i--) {
	temp=U_matrix[base_a + i];
	U_matrix[base_a + i]=U_matrix[base_b + i];
	U_matrix[base_b + i]=temp;
    }
    for(i=n_sub-1;i>=0;i--) {
	temp=L_matrix[base_a + i];
	L_matrix[base_a + i]=L_matrix[base_b + i];
	L_matrix[base_b + i]=temp;	
    }
#else
    IVP_VecFPU::fpu_exchange_rows(&U_matrix[base_a+pivot_col],&U_matrix[base_b+pivot_col],n_sub-pivot_col,IVP_FALSE);
    IVP_VecFPU::fpu_exchange_rows(&L_matrix[base_a],&L_matrix[base_b],n_sub,IVP_FALSE);
#endif    
}

// search from col_nr|colnr down to n_sub|colnr
void IVP_Incr_L_U_Matrix::pivot_search_l_u(int col_nr) {
    IVP_DOUBLE biggest = IVP_Inline_Math::fabsd( U_matrix[col_nr * aligned_row_len + col_nr] );
    int pivot_row = col_nr;

    for(int i=n_sub-1;i>col_nr;i--) {
	IVP_DOUBLE new_val = IVP_Inline_Math::fabsd( U_matrix[i * aligned_row_len + col_nr] );
	if( new_val > biggest ) {
	    pivot_row=i;
	    biggest = new_val;
	}
    }

    if(pivot_row!=col_nr) {
	exchange_rows_l_u(col_nr,pivot_row);
    }
}

// add row to a row more upwards
void IVP_Incr_L_U_Matrix::add_neg_row_upwards_l_u(int row_down,int dest_row,IVP_DOUBLE factor) {
    IVP_DOUBLE *base_a=&U_matrix[row_down * aligned_row_len];
    IVP_DOUBLE *base_b=&U_matrix[dest_row * aligned_row_len];
#if 0 /* single FPU */
    int i;
    for(i=row_down+1;i<n_sub;i++) {
	base_b[i] -= factor * base_a[i];
    }
#else
    IVP_VecFPU::fpu_add_multiple_row(&base_b[row_down+1],&base_a[row_down+1],-factor,n_sub-row_down-1,IVP_FALSE);
#endif    
    
    
    base_a=&L_matrix[row_down * aligned_row_len];
    base_b=&L_matrix[dest_row * aligned_row_len];
#if 0    
    for(i=n_sub-1;i>=0;i--) {
	base_b[i] -=factor * base_a[i];
    }
#else
    IVP_VecFPU::fpu_add_multiple_row(&base_b[0],&base_a[0],-factor,n_sub,IVP_TRUE); //should be TRUE
#endif    
    base_b[row_down] = 0.0f;
}

// add row to a row more downwards
void IVP_Incr_L_U_Matrix::add_neg_row_to_row_l_u(int pivot_row,int dest_row,IVP_DOUBLE factor) {
    int base_a = pivot_row * aligned_row_len;
    int base_b = dest_row * aligned_row_len;

#if 0 /* single FPU */    
    int i;
    for(i=n_sub-1;i>pivot_row;i--) {
	U_matrix[ base_b + i ] -= factor*U_matrix[ base_a + i];
    }
    for(i=n_sub-1;i>=0;i--) {
	L_matrix[ base_b + i ] -= factor*L_matrix[ base_a + i]; 
    }
#else
    IVP_VecFPU::fpu_add_multiple_row(&U_matrix[ base_b + pivot_row +1], &U_matrix[ base_a + pivot_row +1], -factor, n_sub-pivot_row-1,IVP_FALSE);
    IVP_VecFPU::fpu_add_multiple_row(&L_matrix[ base_b ], &L_matrix[ base_a ], -factor, n_sub, IVP_FALSE); //set to TRUE
#endif
    U_matrix[base_b + pivot_row] = 0.0f; 
}

void IVP_Incr_L_U_Matrix::subtract_row_L(int from,int dest,IVP_DOUBLE factor) {
    IVP_DOUBLE *base_a=&L_matrix[from*aligned_row_len];
    IVP_DOUBLE *base_b=&L_matrix[dest*aligned_row_len];
#if 0 /* single FPU */
    int i;
    for(i=n_sub-1;i>=0;i--) {
	base_b[i] -= base_a[i]*factor;
    }
#else
    IVP_VecFPU::fpu_add_multiple_row(&base_b[0],&base_a[0],-factor,n_sub,IVP_FALSE); //should be TRUE
#endif
    
    base_b[from]=0.0f;
}

IVP_RETURN_TYPE IVP_Incr_L_U_Matrix::l_u_decomposition_with_pivoting() {
    if( n_sub == 0 ) {
	return IVP_OK;
    }
    
    int i;
    for(i=n_sub-1;i>=0;i--) {
	IVP_DOUBLE *base=&L_matrix[ i*aligned_row_len ];
	IVP_VecFPU::fpu_set_row_to_zero(&base[0],n_sub,IVP_FALSE); //should be TRUE
	base[i]=1.0f;
    }
    
    int rows_to_do;
    for(rows_to_do=1;rows_to_do<n_sub;rows_to_do++) {
	//debug_print_l_u();
	int pivot_elem=rows_to_do-1;
	pivot_search_l_u(pivot_elem);
	//L_matrix[pivot_elem * n_max + pivot_elem] = 1.0f;
	if(normize_row(pivot_elem)==IVP_FAULT) {
	    return IVP_FAULT;
	}
	
	for(int rows_become_zero=n_sub-1;rows_become_zero>=rows_to_do;rows_become_zero--) {
	    IVP_DOUBLE factor = U_matrix[ rows_become_zero * aligned_row_len + pivot_elem ];
	    if( factor != 0.0f ) { //original matrix may be filled with many zeros
		add_neg_row_to_row_l_u(pivot_elem,rows_become_zero,factor);
	    }
	}
    }
    //L_matrix[ (n_sub-1)*n_max + n_sub-1 ] = 1.0f;
    if(normize_row(n_sub-1)==IVP_FAULT) {
	return IVP_FAULT;
    }
    //debug_print_l_u();
    return IVP_OK;
}

// incremental l_u_decomposition: matrix is correctly decomposed to n_sub*n_sub
// last row of new U was already added, row has length n_sub+1
// last column of new U is stored in input_vec, column has length n_sub
IVP_RETURN_TYPE IVP_Incr_L_U_Matrix::increment_l_u() {
    int i;
    for(i=n_sub-1;i>=0;i--) {
	int original_index = i; //index_pos_contains[ i ];
	mult_vec[i] = input_vec[ original_index ];
    }
    
    mult_vec_with_L();

    for(i=n_sub-1;i>=0;i--) {
	U_matrix[ i * aligned_row_len + n_sub ] = temp_vec[ i ];
    }
    
    //clear row and column of L
    for(i=n_sub-1;i>=0;i--) {
	    L_matrix[ i * aligned_row_len + n_sub ] = 0.0f;
    }
	IVP_VecFPU::fpu_set_row_to_zero(&L_matrix[n_sub*aligned_row_len],n_sub,IVP_FALSE); //should be TRUE
    
    L_matrix[ n_sub * aligned_row_len + n_sub ] = 1.0f;

    //inv_index_pos_contains[ n_sub ] = n_sub;
    //index_pos_contains[ n_sub ] = n_sub;

    int base_row = n_sub * aligned_row_len;

    n_sub++;
    for(i=0;i<n_sub-1;i++) {
	IVP_DOUBLE factor = U_matrix[ base_row + i ];
	add_neg_row_to_row_l_u(i,n_sub-1,factor);
    }

    IVP_RETURN_TYPE ret=normize_row( n_sub-1 );
    //debug_print_l_u();
    return ret;
}

// variable with nr del_nr is removed from linear equation systems
// first U is transformed in a way only 0 0 1 0 0 is found in the row
// then L is transformed in a way only 0 0 1 0 0 is found in the column
// then column and row is deleted from L and U
IVP_RETURN_TYPE IVP_Incr_L_U_Matrix::decrement_l_u(int del_nr) {
    int i;
    //exchange_rows_l_u(del_nr,n_sub-1);
    exchange_columns_L(del_nr,n_sub-1);
    exchange_columns_U(del_nr,n_sub-1);

    for(i=n_sub-2;i>del_nr;i--) {
	IVP_DOUBLE factor=U_matrix[ i * aligned_row_len + del_nr ];
	if(factor != 0.0f) {
	    add_neg_row_L(n_sub-1,i,factor);
	}
	U_matrix[ i * aligned_row_len + del_nr ] = 0.0f;
    }
    if( normize_row( del_nr ) != IVP_OK ) {
	//printf("special_case_decrement\n");
	add_neg_row_L(n_sub-1,del_nr,-1.0f);
	U_matrix[ del_nr * aligned_row_len + del_nr ] = 1.0f;
    }
    
    for(i=del_nr;i<n_sub-1;i++) {
	IVP_DOUBLE factor=U_matrix[ (n_sub-1) * aligned_row_len + i];
	if(factor != 0.0f) {
	    add_neg_row_to_row_l_u(i,n_sub-1,factor);
	}
    }
    //normize_row(n_sub-1);
    
    if( normize_row_L(n_sub-1) != IVP_OK ) {
	n_sub--;
	return IVP_FAULT;
    }
    
    del_nr = n_sub-1;
    for(i=del_nr-1;i>=0;i--) {
	IVP_DOUBLE factor=L_matrix[i*aligned_row_len + del_nr];
	if(factor != 0.0f) {
	    subtract_row_L(del_nr,i,factor);
	}
    }
    

    n_sub--;
    return IVP_OK;
}

void IVP_Incr_L_U_Matrix::add_neg_col_L(int col0,int col_dest,IVP_DOUBLE factor) {
    //Vector FPU not possible
    int i;
    for(i=n_sub-1;i>=0;i--) {
	L_matrix[ i * aligned_row_len + col_dest ] -= factor * L_matrix[ i * aligned_row_len + col0 ];
    }
}

void IVP_Incr_L_U_Matrix::exchange_columns_L(int col0,int col1) {
    //Vector FPU not possible
    int i;
    for(i=0;i<n_sub;i++) {
	IVP_DOUBLE temp_val;
	//temp_val=L_matrix[i * n_max + col0];
	//L_matrix[i * n_max + col0]=L_matrix[i * n_max + col1];
	//L_matrix[i * n_max + col1]=temp_val;
	
	temp_val=L_matrix[i * aligned_row_len + col0];
	L_matrix[i * aligned_row_len + col0]=L_matrix[i * aligned_row_len + col1];
	L_matrix[i * aligned_row_len + col1]=temp_val;
    }
}

void IVP_Incr_L_U_Matrix::exchange_columns_U(int col0,int col1) {
    //Vector FPU not possible
    int i;
    for(i=0;i<n_sub;i++) {
	IVP_DOUBLE temp_val;
	//temp_val=L_matrix[i * n_max + col0];
	//L_matrix[i * n_max + col0]=L_matrix[i * n_max + col1];
	//L_matrix[i * n_max + col1]=temp_val;
	
	temp_val=U_matrix[i * aligned_row_len + col0];
	U_matrix[i * aligned_row_len + col0]=U_matrix[i * aligned_row_len + col1];
	U_matrix[i * aligned_row_len + col1]=temp_val;
    }
}

void IVP_Incr_L_U_Matrix::mult_vec_with_L() {
    int i;
    for(i=n_sub-1;i>=0;i--) {
	IVP_DOUBLE sum=IVP_VecFPU::fpu_large_dot_product(&L_matrix[i*aligned_row_len],&mult_vec[0],n_sub,IVP_TRUE);
	temp_vec[i]=sum;
    }
}

void IVP_Incr_L_U_Matrix::solve_vec_with_U() {
    int i;
    for(i=n_sub-1;i>=0;i--) {
//#if !defined(IVP_WILLAMETTE)
	    IVP_DOUBLE leftsum=0.0f;
	    int j;
	    for(j=n_sub-1;j>i;j--) {
	        leftsum += U_matrix[ i * aligned_row_len + j ] * temp_vec[ j ];
		}
#if 0 /*doesnt work, but why?*/
		//WARNING !!!!!
		//this is dirty: vector multiplication starts in middle of datastructure
		//               put one zero in front -> zero mult something is zero
		IVP_DOUBLE remember=U_matrix[i*aligned_row_len+i];
		U_matrix[i*aligned_row_len+i]=0.0;
		IVP_DOUBLE leftsum = IVP_VecFPU::fpu_large_dot_product(&U_matrix[i*aligned_row_len+i+1],&temp_vec[i+1],n_sub-i-1,IVP_FALSE);
		U_matrix[i*aligned_row_len+i]=remember;
		if( fabs(leftsum-oldleftsum) > 0.0001 ) {
			leftsum=oldleftsum;
		}
#endif 
	    IVP_DOUBLE res = temp_vec[ i ] - leftsum;
	    temp_vec[ i ] = res;
	    out_vec[ i ] = res;
    }
} 

void IVP_Incr_L_U_Matrix::solve_lin_equ() {
    int i;
    for(i=n_sub-1;i>=0;i--) {
	mult_vec[i] = input_vec[ i ];
    }
    mult_vec_with_L();
    solve_vec_with_U(); 
}

void IVP_Incr_L_U_Matrix::add_neg_row_L(int source_row,int dest_row,IVP_DOUBLE factor) {
    int base_a = source_row * aligned_row_len;
    int base_b = dest_row * aligned_row_len;

#if 0 /* single FPU */    
    int i;
    for(i=n_sub-1;i>=0;i--) {
	L_matrix[ base_b + i ] -= factor*L_matrix[ base_a + i]; 
    }
#else
    IVP_VecFPU::fpu_add_multiple_row(&L_matrix[base_b],&L_matrix[base_a],-factor,n_sub,IVP_FALSE); //should be TRUE
#endif    
}

void IVP_Incr_L_U_Matrix::debug_print_l_u() {
    int i;
#if 0
    printf("\nindex  ");
    for(i=0;i<n_sub;i++) {
	printf("%d  ",index_pos_contains[i]);
    }
    printf("\n");
    printf("\ninvindex  ");
    for(i=0;i<n_sub;i++) {
	printf("%d  ",inv_index_pos_contains[i]);
    }
    printf("\n");
#endif    
    printf("  L                                      U\n");
    for(i=0;i<n_sub;i++) {
	int j;
	for(j=0;j<n_sub;j++) {
	    printf("%.5f  ",L_matrix[i*aligned_row_len+j]);
	}
	printf("          ");
	for(j=0;j<n_sub;j++) {
	    printf("%.5f  ",U_matrix[i*aligned_row_len+j]);
	}
	printf("\n");
    }
}

void IVP_Incr_L_U_Matrix::debug_print_a() {
	IVP_Great_Matrix_Many_Zero *was_l=new IVP_Great_Matrix_Many_Zero(n_sub);
	IVP_Great_Matrix_Many_Zero *was_u=new IVP_Great_Matrix_Many_Zero(n_sub);

	int i,j;
	for(i=0;i<n_sub;i++) {
	  for(j=0;j<n_sub;j++) {
	    was_l->matrix_values[i*n_sub+j]=L_matrix[i*aligned_row_len+j];
	    was_u->matrix_values[i*n_sub+j]=U_matrix[i*aligned_row_len+j];	    
	  }
	}

	IVP_Great_Matrix_Many_Zero *inv_l=new IVP_Great_Matrix_Many_Zero(n_sub);
	was_l->invert(inv_l);

	IVP_Great_Matrix_Many_Zero *new_a=new IVP_Great_Matrix_Many_Zero(n_sub);
	new_a->matrix_multiplication(inv_l->matrix_values,was_u->matrix_values);

	new_a->print_great_matrix("orig_A");
}

// when matrix is singular give dependences of a variable
void IVP_Linear_Constraint_Solver::debug_dep_var(int full_var_nr) {
    int col_nr=0;
    int sub_var_nr=variable_is_found_at[ full_var_nr ];
    if( sub_var_nr >= r_actives ) {
	printf("works_only_with_active\n");
	return;
    }
    int i,j;
    sub_solver_mat.columns = r_actives;
    sub_solver_mat.calc_aligned_row_len();
    
    for(i=0;i<sub_var_nr;i++) {
	for(j=0;j<r_actives;j++) {
	    int full_i,full_j;
	    full_i=actives_inactives_ignored[i];
	    full_j=actives_inactives_ignored[j];
	    sub_solver_mat.matrix_values[ j*sub_solver_mat.aligned_row_len + col_nr] = full_solver_mat.matrix_values[ full_j*aligned_size + full_i];
	}
	col_nr++;
    }
    for(i=sub_var_nr+1;i<r_actives;i++) {
	for(j=0;j<r_actives;j++) {
	    int full_i,full_j;
	    full_i=actives_inactives_ignored[i];
	    full_j=actives_inactives_ignored[j];
	    sub_solver_mat.matrix_values[ j*sub_solver_mat.aligned_row_len + col_nr] = full_solver_mat.matrix_values[ full_j*aligned_size + full_i];
	}
	col_nr++;
    }
    for(j=0;j<r_actives;j++) {
	sub_solver_mat.matrix_values[ j*r_actives + r_actives-1 ] = 0.0f;
	int full_i,full_j;
	full_i=actives_inactives_ignored[sub_var_nr];
	full_j=actives_inactives_ignored[j];
	sub_solver_mat.desired_vector[j] = full_solver_mat.matrix_values[ full_j*aligned_size + full_i];
    }

    if( sub_solver_mat.solve_great_matrix_many_zero() == IVP_OK ) {
	printf("dependency %d is ",full_var_nr);
	col_nr=0;
	for(i=0;i<sub_var_nr;i++) {
	    printf("%d:%f ",actives_inactives_ignored[i],sub_solver_mat.result_vector[col_nr]);
	    col_nr++;
	}
	for(i=sub_var_nr+1;i<r_actives;i++) {
	    printf("%d:%f ",actives_inactives_ignored[i],sub_solver_mat.result_vector[col_nr]);
	    col_nr++;
	}
	printf("\n");
    } else {
	printf("variable %d is independent\n",full_var_nr);
    }
}

