// Copyright (C) Ipion Software GmbH 1999-2000. All rights reserved.

// IVP_EXPORT_PUBLIC

#ifndef WIN32
#	pragma interface
#endif


/**********************************************************************************
 * Name:        IVP_MI_VECTOR_ALLOCA
 * Description: Makro used to allocate memory on the stack a IVP_MI_Vector object
 **********************************************************************************/
#define IVP_MI_VECTOR_ALLOCA( var, size ){									\
    IVP_MI_Vector *result = (IVP_MI_Vector *)alloca( sizeof( IVP_MI_Vector ) + sizeof(IVP_FLOAT) * (size-1));	\
    result->nr_of_elements = size;										\
    for (int l_i=0; l_i< int(size); l_i++) {  										\
	result->set(l_i, 0.0f);  		 									\
    }  														\
    result->weight_statistic = 0.0f;										\
    var = result;												\
    }

/**********************************************************************************
 * Name:        IVP_MI_VECTOR_ALLOCA
 * Description: Makro used to allocate memory on the stack a IVP_MI_Vector object
 **********************************************************************************/
#define IVP_MI_VECTOR_CLEAR( result, size ){									\
    result->nr_of_elements = size;										\
    for (int l_i=0; l_i< int(size); l_i++) {  										\
	result->set(l_i, 0.0f);  		 									\
    }  														\
    result->weight_statistic = 0.0f;										\
    }


/**********************************************************************************
 * Name:        IVP_MI_VECTOR_ALLOCA_AND_SET
 * Description: Makro used to allocate memory on the stack a IVP_MI_Vector object
 *              Additionally the vector elements are preset with the values obtained
 *              from 'values'
 **********************************************************************************/
#define IVP_MI_VECTOR_ALLOCA_AND_SET( var, size, values ){							\
    IVP_MI_Vector *result = (IVP_MI_Vector *)alloca( sizeof( IVP_MI_Vector ) + sizeof(IVP_FLOAT) * (size-1));	\
    result->nr_of_elements = size;										\
    for (int l_i=0; l_i<size; l_i++) {										\
	result->set(l_i, values[l_i]);										\
    }														\
    result->weight_statistic = 0.0f;										\
    var = result;												\
}



/********************************************************************************
 * Name:        Class IVP_MI_Vector_Base
 * Description: Super class from which 'IVP_MI_Vector' and 'IVP_Buoyancy_Input'/
 *              'IVP_Buoyancy_Output' derive. It provides additional member
 *              variables.
 ********************************************************************************/
class IVP_MI_Vector_Base {
public:
    int nr_of_elements;      //describes the number of elements in the array of floats 'element'
    IVP_FLOAT weight_statistic;  //a weight value that can be used by a sort algorithm (e.g. 'IVP_Multidimensional_Interpolator::sort_vectors(...)' )
    IVP_Time time_stamp;
};


/********************************************************************
 * Name:        Class IVP_MI_Vector
 * Description: Describes a vector of float values with additional
 *              properties used by the Multidimensional Interpolator
 ********************************************************************/
class IVP_MI_Vector: public IVP_MI_Vector_Base {
private:
    IVP_MI_Vector();  //deactivate this contructor
public:
    IVP_FLOAT element[1];        //array of elements

    /***********************************************************************************
     * Name:        malloc_mi_vector(...)
     * Description: Method used to allocate heap memory for a instance of IVP_MI_Vector
     ***********************************************************************************/
    static IVP_MI_Vector *malloc_mi_vector( int size ){
	IVP_MI_Vector *result = (IVP_MI_Vector *)p_malloc( sizeof( IVP_MI_Vector ) + sizeof(IVP_FLOAT) * (size-1));
	result->nr_of_elements = size;
	for (int i=0; i<size; i++) {
	    result->element[i] = 0.0f;
	}
	result->weight_statistic = 0.0f;
	return result;
    }

    /*****************************************************************************************
     * Name:        malloc_and_set_mi_vector(...)
     * Description: Method used to allocate heap memory for a instance of IVP_MI_Vector
     *              Additionally the vector elements are preset with the values from 'values'
     *****************************************************************************************/
    static IVP_MI_Vector *malloc_and_set_mi_vector( int size, IVP_FLOAT *values ){
	IVP_MI_Vector *result = (IVP_MI_Vector *)p_malloc( sizeof( IVP_MI_Vector ) + sizeof(IVP_FLOAT) * (size-1));
	result->nr_of_elements = size;
	for (int i=0; i<size; i++) {
	    result->element[i] = values[i];
	}
	result->weight_statistic = 0.0f;
	return result;
    }

    /**********************************************************************
     * Name:        print()
     * Description: Method to print the values of the members of 'element'
     **********************************************************************/
    void print() const {
	for (int i=0; i < nr_of_elements; i++) {
	    printf("v[%d]=%1.3e ",i, element[i]);
	}
	printf("\n");
    }

    void set(const IVP_MI_Vector *v);                          //copies values of 'v' to 'this'
    inline void set(const int pos, IVP_FLOAT value);           //sets 'element[pos]' to 'value'

    void subtract(const IVP_MI_Vector *v);                     //subtracts the values of 'v->element' from 'element'
    void add(const IVP_MI_Vector *v);                          //adds the values of 'v->element' to 'element'
    void add_multiple(const IVP_MI_Vector *v, const IVP_FLOAT d);  //adds the values of 'v->element' multiplied by 'd' to 'element'
    void mult(const IVP_FLOAT d);                                  //multiplies the values of 'element' by 'd'

    IVP_FLOAT length() const;     //returns the number of elements of this IVP_MI_Vector

    void set_time_stamp(IVP_Time current_time);  //sets the time stamp of 'IVP_MI_Vector_Base'
};

void IVP_MI_Vector::set(int pos, IVP_FLOAT value) {
    IVP_ASSERT( (pos < nr_of_elements) && (pos >= 0) );
    element[pos] = value;
}



/******************************************************************************
 * Name:        Class IVP_Multidimensional_Interpolator
 * Description: Provides mechanism to re-use previous solutions to a problem
 *              by inter-/extrapolating them to achieve a new solution.
 *              This has potential to dramatically reduce computation overhead
 ******************************************************************************/
class IVP_Multidimensional_Interpolator {
private:
    //arrays of vectors of input-/solution values which are used to compute interpolations of new vectors
    IVP_MI_Vector **previous_inputs;
    IVP_MI_Vector **previous_solutions;
    
    int nr_of_vectors:8;            //denotes the length of the arrays of vectors 'previous_inputs' and 'previous_solutions' stored for the interpolation
    int nr_of_elements_input:8;     //denotes the number of elements within an input vector
    int nr_of_elements_solution:8;  //denotes the number of elements within a sulution vector
    
    IVP_FLOAT MI_eps;   //epsilon value used internally
    //IVP_FLOAT max_res;  //maximum value the residuum of the LINFIT algortihm is allowed to have; It describes the maximum deviation of LINFIT's result from the optimum
    
    IVP_FLOAT influence_of_old_weight;     //describes the influence of an old weight value on the calculation of the new weight of an input vector

    int nr_occupied;         //contains the number of occupied vectors in previous_inputs/-solutions
    int scratch_area_index;  //denotes the 'scratch element' of 'previous_solutions'

    int nr_of_vectors_involved;  //denotes the number of vectors needed for the last interpolation, will be reused in the next PSI step

    //counter that makes sure that after a defined number of interpolation attempts (max_tries_nr_of_vectors_involved)
    // with the same or higher nr_of_vectors_involved it is again tried with a smaller number of vectors
    int counter_tries_nr_of_vectors_involved;

    //counter that is needed to determine the next vector to be replaced by a new one when using 'add_new_input_solution_combination_stochastic(...)'
    int initial_value_vector_replacement;
    int counter_vector_replacement;

    /***********************************************************************************
     *  Name:         sort_vectors()
     *  Description:  An implementation of bubble sort
     *  Note:         Although it is O(n^2), the data to be sorted is in 
     *                most cases already presorted, so bubble sort is efficient enough
     ***********************************************************************************/
    void sort_vectors(int nr_to_be_sorted);

    /**************************************************************************
     * Name:         linfit()                                             
     * Description:  Algorithm used to solve the "linear compensation problem"
     * Note:         'A' is a matrix of input vectors, 'm' its number of lines,
     *               'n' its number of columns. 'X' is used to hold the solution
     *               of the algorithm, 'res' is its residuum (the deviation from
     *               the "ideal" solution)
     **************************************************************************/
    IVP_RETURN_TYPE linfit(int m, int n, IVP_MI_Vector **A_, IVP_FLOAT *X, IVP_FLOAT *res);

public:
    /*********************
     * constructor method
     *********************/
    IVP_Multidimensional_Interpolator(int nr_of_vectors_, int nr_of_elements_input_, int nr_of_elements_solution_);

    /********************
     * destructor method
     ********************/
    ~IVP_Multidimensional_Interpolator() {
#ifdef DEBUG
	P_DELETE_ARRAY(nr_res_over_limit);
	P_DELETE_ARRAY(nr_int_weight_over_limit);
	P_DELETE_ARRAY(nr_of_linfit_failure);
	P_DELETE_ARRAY(nr_of_success);
#endif
	int i;
	for (i=0; i<nr_of_vectors; i++) {
	    P_FREE(previous_inputs[i]);
	}
	for (i=0; i<=nr_of_vectors; i++) {
	    P_FREE(previous_solutions[i]);
	}
	P_FREE(previous_inputs);
	P_FREE(previous_solutions);
    }

    /********************
     * selector methods
     ********************/
    inline int get_nr_of_vectors() { return nr_of_vectors; }   //returns the length of the arrays of vectors "previous_inputs" and "previous_solutions" stored for the interpolation
    inline int get_nr_occupied() { return nr_occupied; }       //returns the number of array elements in "previous_inputs" ("previous_solutions") which already hold a vector
    
    
    /******************************************************************************************************
     * Name:        check_interpolation(...)
     * Description: Main method of the Multidimensional Interpolator
     * Note:        Receives a new vector of input values and tries to find a reasonable interpolation
     *              with the help of input vectors already received in previous runs. If successful the
     *              calculated parameters are used afterwards to inter-/extrapolate a new vector of
     *              solution values from solution vectors of previous runs.
     *              The return value denotes if this attempt was successful
     ******************************************************************************************************/
    IVP_RETURN_TYPE check_interpolation(const IVP_MI_Vector *new_input,
					const int max_tries_nr_of_vectors_involved,
					const IVP_FLOAT max_res,
					IVP_MI_Vector *output);


    /**************************************************************************************
     * Name:        add_new_input_solution_combination_...
     * Description: Methods used for inserting new input- and solution vectors into the
     *              appropriate data structures 'previous_inputs' and 'previous_solutions'
     **************************************************************************************/
    void add_new_input_solution_combination_conventional(const IVP_MI_Vector *new_input, const IVP_MI_Vector *new_solution);  //adds new values in a FIFO like manner
    void add_new_input_solution_combination_stochastic(const IVP_MI_Vector *new_input, const IVP_MI_Vector *new_solution);    //adds new values in "random" order

#ifdef DEBUG
    //begin debug variables !!!!!!!!!
    int *nr_res_over_limit;
    int *nr_int_weight_over_limit;
    int *nr_of_linfit_failure;
    int *nr_of_success;
    int nr_one_vector_sufficient;
    //end debug variables !!!!!!!!
#endif
};
