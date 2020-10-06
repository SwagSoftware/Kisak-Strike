#ifndef HK_MATH_MATRIX_INVERTER
#define HK_MATH_MATRIX_INVERTER

#include <hk_math/vector_fpu/vector_fpu.h>


class hk_Matrix_Inverter
{
public:
	static inline void invert_no_pivot_search( hk_real *in_matrix, int lda, int num_rows, hk_real *out_matrix)
	{
		hk_real *pr_in = in_matrix;
		hk_real *pr_out = out_matrix;

		{ // zero out_matrix
			hk_real *p = pr_out;
			for (int i = num_rows * (lda>>1) -1; i>=0 ;i--){
				p[0] = 0.0f;
				p[1] = 0.0f;
				p += 2;
			}
			p = pr_out;
			for (int k = num_rows-1; k>=0; k--){
				p[0] = 1.0f;
				p += lda + 1;
			}
		}
		// forward step
		{
			for ( int i = 0 ; i < num_rows; i ++ ){
				hk_real f = pr_in[i];
				//HK_CHECK( hk_Math::fabs(f) >> HK_FLOAT_EPS);

				f = 1.0f / f;
				{ // scale this row
					int s = i;
					do {
						pr_in[s] *= f;
					} while ( ++s < num_rows );

					int t = 0;
					do {
						pr_out[t] *= f;
					} while ( ++t <= i);
				}

				hk_real *row_to_zero = pr_in;
				hk_real *inverted_row = pr_out;

				for ( int row = i+1; row < num_rows; row++){

					row_to_zero += lda;
					inverted_row += lda;

					hk_real factor = row_to_zero[i];
					if (factor == 0.0f) { // matrix elem is already zero
						continue;
					}
					int k = 0;
					do {
						inverted_row[k] -= pr_out[k] * factor ;
					} while ( ++k <= i);

					k = i;
					do {
						row_to_zero[k] -= pr_in[k] * factor;
					} while ( ++k < num_rows );

				}
				pr_in += lda;
				pr_out += lda;
			}
		}
		// reverse step (digonal is already scaled)

		{
			for ( int i = num_rows-1 ; i >=0; i -- ){
				pr_in -= lda;
				pr_out -= lda;

				hk_real *row_to_zero = pr_in;
				hk_real *inverted_row = pr_out;

				for ( int row = i-1; row >= 0; row--){
					row_to_zero -= lda;
					inverted_row -= lda;

					hk_real factor = row_to_zero[i];

					if (factor == 0.0f) { // matrix elem is already zero
						continue;
					}
					
					//	hk_VecFPU::fpu_add_multiple_row( inverted_row, pr_out, -factor, lda, HK_TRUE);

					int k = num_rows-1;
					do {
						inverted_row[k] -= pr_out[k] * factor;
					}while( --k >=0);
					k = num_rows-1;
					do {
						row_to_zero[k] -= pr_in[k] * factor;
					} while (--k >= i );
				}

			}
		}
	}
};

#endif /* HK_MATH_MATRIX_INVERTER */
