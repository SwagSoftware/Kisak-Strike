
#include <ivp_physics.hxx>

#if defined(LINUX)||defined(SUN)||(__MWERKS__ && __POWERPC__)
#   include <alloca.h>
#endif


#ifndef WIN32
#	pragma implementation "ivp_multidimensional_interp.hxx"
#endif

#include "ivp_multidimensional_interp.hxx"

//#define WITH_DEBUG_OUTPUT          1
#define LOWER_LIMIT    -0.2f
#define UPPER_LIMIT     1.2f

void IVP_MI_Vector::set(const IVP_MI_Vector *v) {
    IVP_ASSERT( nr_of_elements >= v->nr_of_elements);
    nr_of_elements = v->nr_of_elements;
    weight_statistic = v->weight_statistic;
    for (int i=0; i<nr_of_elements; i++) {
	element[i] = v->element[i];
    }
}


void IVP_MI_Vector::subtract(const IVP_MI_Vector *v) {
    IVP_ASSERT( nr_of_elements == v->nr_of_elements);
    for (int i=0; i<nr_of_elements; i++) {
	element[i] -= v->element[i];
    }
}


void IVP_MI_Vector::add(const IVP_MI_Vector *v) {
    IVP_ASSERT( nr_of_elements == v->nr_of_elements);
    for (int i=0; i<nr_of_elements; i++) {
	element[i] += v->element[i];
    }
}


void IVP_MI_Vector::add_multiple(const IVP_MI_Vector *v, const IVP_FLOAT d) {
    IVP_ASSERT( nr_of_elements == v->nr_of_elements);
    for (int i=0; i<nr_of_elements; i++) {
	element[i] += (v->element[i] * d);
    }
}


void IVP_MI_Vector::mult(const IVP_FLOAT d) {
    for (int i=0; i<nr_of_elements; i++) {
	element[i] *= d;
    }
}


IVP_FLOAT IVP_MI_Vector::length() const {
    IVP_FLOAT result = 0.0f;
    for (int i=0; i<nr_of_elements; i++) {
	result += element[i] * element[i];
    }
    return (IVP_FLOAT)(IVP_Inline_Math::ivp_sqrtf(result));
}


void ivp_print_mi_matrix(int n, int m, IVP_MI_Vector **A) {
    int i, j;

    for (i=0; i<m; i++) {
	    printf("     [ ");
	for (j=0; j<=n; j++) {
	    printf("% 1.3e ", A[j]->element[i]);
	}
	printf("] \n");
    }
    printf("\n");
}


void IVP_MI_Vector::set_time_stamp(IVP_Time current_time) {
    time_stamp = current_time;
}


/************************************************************************
 * Name:         linfit()                                             
 * Description:  Algorithm that solves the "linear compensation problem" 
 ************************************************************************/
IVP_RETURN_TYPE IVP_Multidimensional_Interpolator::linfit(int m, int n, IVP_MI_Vector **A, IVP_FLOAT *X, IVP_FLOAT *res) {

    //m = X->nr_of_elements;  //number of lines in the matrix A
    //n = nr_of_vectors;      //number of columns in the matrix A

#if WITH_DEBUG_OUTPUT
    //print_matrix(n, m, A);
#endif
    
    //calc cancel condition value f
    IVP_FLOAT s;
    IVP_FLOAT f = 0.0f;
    for (int k=0; k<n; k++) {
	s = 0.0f;
	for (int i=0; i<m; i++) {
	    s += A[k]->element[i]*A[k]->element[i];
	}
	if (s > f) f = s;
    }
    f = (IVP_FLOAT) IVP_Inline_Math::ivp_sqrtf(f) * MI_eps;
    
    //orthogonal elimination
    //int n1 = n+1;
    for (int j=0; j<n; j++) {
	for (int i=j+1; i<m; i++) {
	    if(IVP_Inline_Math::fabsd(A[j]->element[i]) > MI_eps/*P_DOUBLE_RES*/) {
	    //if (A[j]->element[i] != 0.0f) {
		IVP_FLOAT q = (IVP_FLOAT) IVP_Inline_Math::ivp_sqrtf(A[j]->element[i]*A[j]->element[i] + A[j]->element[j]*A[j]->element[j]);
		IVP_ASSERT(q != 0.0f);
		IVP_FLOAT c = A[j]->element[j] / q;
		s = -A[j]->element[i] / q;
		A[j]->element[j] = q;

		for (int k=j+1; k<=n; k++) {
		    IVP_FLOAT b = s*A[k]->element[j];
		    A[k]->set(j, c*A[k]->element[j] - s*A[k]->element[i]);
		    A[k]->set(i, c*A[k]->element[i] + b);
		}
	    }
	}
	if (IVP_Inline_Math::fabsd(A[j]->element[j]) <= f) return(IVP_FAULT);  //matrix is not invertible (it is singularian), so we cannot substitute back later
    }
    
    //substitute back
    X[n-1] = A[n]->element[n-1] / A[n-1]->element[n-1];

    for (int i=n-2; i>=0; i--) {
	X[i] = A[n]->element[i] - (X[n-1] * A[n-1]->element[i]);
	for (int j=i+1; j<=n-2; j++) {
	    X[i] -= A[j]->element[i]*X[j];
	}
	IVP_ASSERT(A[i]->element[i] != 0.0f);
	X[i] /= A[i]->element[i];
    }
    
    s = 0;
    for (int l=n; l<m; l++) {
	s += A[n]->element[l]*A[n]->element[l];
    }
    *res = (IVP_FLOAT) IVP_Inline_Math::ivp_sqrtf(s);
    
    return(IVP_OK);
}



/***********************************************************************************
 *  Name:         sort_vectors()
 *  Description:  Implementation of bubble sort
 *                Although it is O(n**2), the data to be sorted is in 
 *                most cases already presorted, so bubble sort is efficient enough
 ***********************************************************************************/
void IVP_Multidimensional_Interpolator::sort_vectors(int nr_to_be_sorted) {
    
    int first = 0;
    while (true) {
	int second = first+1;
	//if (second >= nr_of_vectors) break;
	if (second >= nr_to_be_sorted) break;
	
	IVP_FLOAT second_weight = previous_inputs[second]->weight_statistic;
	if (previous_inputs[first]->weight_statistic < second_weight) {
	    
	    //exchange input vectors
	    IVP_MI_Vector *tmp_vec = previous_inputs[second];
	    previous_inputs[second] = previous_inputs[first];
	    previous_inputs[first] = tmp_vec;

	    //exchange solution vectors
	    tmp_vec = previous_solutions[second];
	    previous_solutions[second] = previous_solutions[first];
	    previous_solutions[first] = tmp_vec;
	    
	    int test_pos = first;
	    while ( (test_pos > 0) && (previous_inputs[test_pos-1]->weight_statistic < second_weight) ) {

		//exchange input vectors
		tmp_vec = previous_inputs[test_pos];
		previous_inputs[test_pos] = previous_inputs[test_pos-1];
		previous_inputs[test_pos-1] = tmp_vec;

		//exchange solution vectors
		tmp_vec = previous_solutions[test_pos];
		previous_solutions[test_pos] = previous_solutions[test_pos-1];
		previous_solutions[test_pos-1] = tmp_vec;
		
		test_pos--;
	    }
	}
	first = second;
    }
}


#if !defined(IVP_NO_MD_INTERPOLATION)
/******************************************************************************
 *  Name:         check_interpolation()
 *  Description:  checks, if previous input vectors can be used to interpolate
 *                a solution for a new input vector 
 ******************************************************************************/
IVP_RETURN_TYPE IVP_Multidimensional_Interpolator::check_interpolation(const IVP_MI_Vector *new_input,
								       const int max_tries_nr_of_vectors_involved,
								       const IVP_FLOAT max_res,
								       IVP_MI_Vector *output) {

    if (nr_occupied < 2) {  //solver has to compute the solution for itself, because there is not enough data yet
	return(IVP_FAULT);
    }
    
    
    //at this point it is already provided for enough data from previous solver runs

    //try to interpolate a new solution via resolving the linear compensation problem with an increasing number of input vectors
    IVP_MI_Vector **scratchboard_A = (IVP_MI_Vector **)alloca( sizeof(void *) * (nr_occupied+1) );  //add one element because scratchboard_A also has to hold the new_input
    for (int i=0; i<=nr_occupied; i++) {
	IVP_MI_VECTOR_ALLOCA(scratchboard_A[i],nr_of_elements_input);
    }

    IVP_FLOAT *interpolation_weights = (IVP_FLOAT *)alloca( sizeof(IVP_FLOAT) * nr_occupied );
    IVP_FLOAT interpolation_weigh_first = 0.0f;  //interpolation weigh of the first vector, which is (1.0f - (all other weights))
    IVP_FLOAT res;
    //int nr_of_vectors_involved;
    IVP_RETURN_TYPE success = IVP_FAULT;

    //create difference of new_input and the first of the old input vectors
    IVP_MI_Vector *new_input_difference;
    IVP_MI_VECTOR_ALLOCA(new_input_difference, nr_of_elements_input);
    
    new_input_difference->set(new_input);
    new_input_difference->subtract(previous_inputs[0]);

    //make sure that after some time we try the interpolation again with a smaller number of vectors
    if (counter_tries_nr_of_vectors_involved++ >= max_tries_nr_of_vectors_involved) {
	nr_of_vectors_involved = 2;
	counter_tries_nr_of_vectors_involved = 0; //reset counter
    }
    
    //check if new_input_difference has the length 0
    if (new_input_difference->length() < MI_eps) {
	//the first member of the old input vectors is already sufficient
	nr_of_vectors_involved=1;
#ifdef DEBUG
	nr_one_vector_sufficient++;
#endif
	success = IVP_OK;
    } else {
	//more than 1 input vector needed for the interpolation
	if ((nr_of_vectors_involved < 2) || (nr_of_vectors_involved > nr_occupied)) {
	    //in the previous interpolation run one vector was sufficient (but not now)
	    nr_of_vectors_involved = 2;
	}
	
	while (nr_of_vectors_involved <= nr_occupied) {
	    //initialize the scratchboard
	    for (int i=0; i<(nr_of_vectors_involved-1); i++) {
		scratchboard_A[i]->set(previous_inputs[i+1]);
		scratchboard_A[i]->subtract(previous_inputs[0]);
	    }
	    scratchboard_A[nr_of_vectors_involved-1]->set(new_input_difference); //now the matrix [A|b] is ready for LINFIT

	    //run the linfit algorithm
	    success = linfit(nr_of_elements_input, nr_of_vectors_involved-1, scratchboard_A, interpolation_weights, &res);

#ifdef WITH_DEBUG_OUTPUT
	    printf("nr_of_vectors_involved = %d, nr_occupied = %d, res = %1.4e\n", nr_of_vectors_involved, nr_occupied, res);
#endif
	    
	    if (success) {

		interpolation_weigh_first = 1.0f;
		
		if (res > max_res) {
		    success = IVP_FAULT;
#ifdef WITH_DEBUG_OUTPUT
		    printf("res is over limit!\n");
		    nr_res_over_limit[nr_of_vectors_involved-1]++;
#endif
		} else {
		
		    //ensure that the interpolation_weights are in the interval [-0.2f; 1.2f]
		    for (int ix=0; ix<(nr_of_vectors_involved-1); ix++) {
			if (IVP_Inline_Math::fabsd(interpolation_weights[ix] - 0.5f) > 0.7f) {
			    success = IVP_FAULT;
#ifdef WITH_DEBUG_OUTPUT
			    printf("interpolation_weigh is over limit!\n");
			    nr_int_weight_over_limit[nr_of_vectors_involved-1]++;
#endif
			    break;
			}
			interpolation_weigh_first -= interpolation_weights[ix];
		    }

		    if ((IVP_Inline_Math::fabsd(interpolation_weigh_first - 0.5f) > 0.7f) && success) {
			success = IVP_FAULT;
#ifdef WITH_DEBUG_OUTPUT
			printf("interpolation_weigh_first is over limit!\n");
			nr_int_weight_over_limit[nr_of_vectors_involved-1]++;
#endif
		    }
		}

#ifdef WITH_DEBUG_OUTPUT
		printf("int_w[0] = %e ", interpolation_weigh_first);
		for (int g=0; g<nr_of_vectors_involved-1; g++) {
		    printf("int_w[%d] = %e ", g+1, interpolation_weights[g]);
		}
		printf("\n");
#endif
		
		if (success) {
#ifdef DEBUG
		  nr_of_success[nr_of_vectors_involved-1]++;
#endif
		  break;
		}
	    } else {
		//linfit could not solve the interpolation
#ifdef DEBUG
		nr_of_linfit_failure[nr_of_vectors_involved-1]++;
#endif
	    }
	    nr_of_vectors_involved++;
	}
    }
    
    	
    if (success) {  //new_input was successfully interpolated, the weights of the old input vectors involved can be found in interpolation_weights
	
	//clear the space for the new solution
	for (int j=0; j<nr_of_elements_solution; j++) {
	    previous_solutions[scratch_area_index]->set(j, 0.0f);
	}

	
	//calc weight statistics for every vector involved and
	//calc the output as result of the linear combination of the solution vectors
	for (int i=1; i<nr_of_vectors_involved; i++) {
	    previous_inputs[i]->weight_statistic = previous_inputs[i]->weight_statistic * influence_of_old_weight + interpolation_weights[i-1];

	    for (int k=0; k<nr_of_elements_solution; k++) {
		previous_solutions[scratch_area_index]->set(k, previous_solutions[scratch_area_index]->element[k] + interpolation_weights[i-1] * previous_solutions[i]->element[k]);
	    }

	}

	//calc weigh statistics for the first vector and add first vector's share to the output
	previous_inputs[0]->weight_statistic = (previous_inputs[0]->weight_statistic * influence_of_old_weight) + interpolation_weigh_first;
	for (int k=0; k<nr_of_elements_solution; k++) {
	    previous_solutions[scratch_area_index]->set(k, previous_solutions[scratch_area_index]->element[k] + interpolation_weigh_first * previous_solutions[0]->element[k]);
	}
	
	output->set(previous_solutions[scratch_area_index]);

#ifdef WITH_DEBUG_OUTPUT
	{
	    IVP_MI_Vector *test_vector;
	    IVP_MI_VECTOR_ALLOCA(test_vector, nr_of_elements_input);
	    for (int i=1; i<nr_of_vectors_involved; i++) {
		for (int k=0; k<nr_of_elements_input; k++) {
		    test_vector->set(k, test_vector->element[k] + interpolation_weights[i-1] * previous_inputs[i]->element[k]);
		}
	    }

	    for (int k=0; k<nr_of_elements_input; k++) {
		test_vector->set(k, test_vector->element[k] + interpolation_weigh_first * previous_inputs[0]->element[k]);
	    }

	    printf("interpolated_output_vector:\n");
	    output->print();
	}
#endif
	//sort the vectors (i.e. index[]) in declining order of their weight statistics
	sort_vectors(nr_occupied);
    }
	
#ifdef WITH_DEBUG_OUTPUT
    for (int y=0; y<nr_occupied; y++) {
	printf("timestamp = %f  ", previous_inputs[y]->time_stamp);
	previous_inputs[y]->print();
    }
#endif
    
    return(success);
}
#endif


void IVP_Multidimensional_Interpolator::add_new_input_solution_combination_conventional(const IVP_MI_Vector *new_input, const IVP_MI_Vector *new_solution) {

    //move all vectors one position further, the new vectors will be copied to the first position
    IVP_MI_Vector *tmp_vec_input = previous_inputs[nr_of_vectors-1];
    IVP_MI_Vector *tmp_vec_solution = previous_solutions[nr_of_vectors-1];
    for (int i=nr_of_vectors-2; i>=0; i--) {
	previous_inputs[i+1] = previous_inputs[i];
	previous_solutions[i+1] = previous_solutions[i];
    }
    previous_inputs[0] = tmp_vec_input;
    previous_solutions[0] = tmp_vec_solution;
    
    //copy new vectors into first position
    previous_inputs[0]->set(new_input);
    previous_solutions[0]->set(new_solution);

    //debugging!!!!!!!!!!!!!!!!!!!!!!!!!
    previous_inputs[0]->set_time_stamp(new_input->time_stamp);
    previous_solutions[0]->set_time_stamp(new_solution->time_stamp);
    

    //give the new input vector a weigh that ensures it will be positioned very high after a sort
    previous_inputs[0]->weight_statistic = 1.0f / (1.0f - influence_of_old_weight);

    if (nr_occupied < nr_of_vectors)
	nr_occupied++;
}


void IVP_Multidimensional_Interpolator::add_new_input_solution_combination_stochastic(const IVP_MI_Vector *new_input, const IVP_MI_Vector *new_solution) {

    //define the location of the new pair of values by stochastic means
    int solution_slot = (counter_vector_replacement-- * 101) % nr_of_vectors;
    
    previous_inputs[solution_slot]->set(new_input);
    previous_solutions[solution_slot]->set(new_solution);

    //debugging!!!!!!!!!!
    previous_inputs[solution_slot]->set_time_stamp(new_input->time_stamp);
    previous_solutions[solution_slot]->set_time_stamp(new_solution->time_stamp);

    //give the new input vector a weigh that ensures it will be positioned very high after a sort
    previous_inputs[solution_slot]->weight_statistic = 1.0f / (1.0f - influence_of_old_weight);

    sort_vectors(nr_occupied);
    
    //update the counter
    if (counter_vector_replacement < 0)
	counter_vector_replacement = initial_value_vector_replacement;
}



/************************
 * Constructor
 ************************/
IVP_Multidimensional_Interpolator::IVP_Multidimensional_Interpolator(int nr_of_vectors_, int nr_of_elements_input_, int nr_of_elements_solution_) {

    int i;
    nr_of_vectors = nr_of_vectors_;
    nr_of_elements_input = nr_of_elements_input_;
    nr_of_elements_solution = nr_of_elements_solution_;
    //max_tries_nr_of_vectors_involved = max_tries_nr_of_vectors_involved_;

    //initialize counter_tries_nr_of_vectors_involved
    counter_tries_nr_of_vectors_involved = 0;

    //initialize counter_vector_replacement
    initial_value_vector_replacement = 50;
    counter_vector_replacement = initial_value_vector_replacement;

    //previous_inputs = new IVP_MI_Vector[nr_of_vectors](nr_of_elements_input);  //contains vectors of input data from previous runs of a solver
    previous_inputs = (IVP_MI_Vector **)p_malloc( sizeof(void*) * (nr_of_vectors) );
    for (i=0; i<nr_of_vectors; i++) {
	previous_inputs[i] = IVP_MI_Vector::malloc_mi_vector(nr_of_elements_input);
    }
    
    //IMPORTANT: The vectors have to be linearly unrelated
    previous_solutions = (IVP_MI_Vector **)p_malloc( sizeof(void*) * (nr_of_vectors+1) );
    for (i=0; i<=nr_of_vectors; i++) {
	previous_solutions[i] = IVP_MI_Vector::malloc_mi_vector(nr_of_elements_solution);
    }
    
    influence_of_old_weight = 0.8f;
	
    nr_occupied = 0;
    nr_of_vectors_involved = 2;

    scratch_area_index = nr_of_vectors;
    MI_eps = 1.0e-14f;
    //max_res = 1.0e-2f;

    //begin debug variables!!!!!!!!!!!!
#ifdef DEBUG
    nr_one_vector_sufficient = 0;
    nr_res_over_limit = new int[nr_of_vectors];
    nr_int_weight_over_limit = new int[nr_of_vectors];
    nr_of_linfit_failure = new int[nr_of_vectors];
    nr_of_success = new int[nr_of_vectors];
    for (i=0; i<nr_of_vectors; i++) {
	nr_res_over_limit[i] = 0;
	nr_int_weight_over_limit[i] = 0;
	nr_of_linfit_failure[i] = 0;
	nr_of_success[i] = 0;
    }
#endif
    //end debug variables!!!!!!!!!!!!!
}
