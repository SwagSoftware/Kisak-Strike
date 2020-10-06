// TKBMS v1.0 -----------------------------------------------------
//
// PLATFORM		: ALL
// PRODUCT		: VPHYSICS
// VISIBILITY	: INTERNAL
//
// ------------------------------------------------------TKBMS v1.0

#include <hk_math/vecmath.h>
#include <hk_math/densematrix.h>
#include <hk_math/densematrix_util.h>
#include <hk_math/dense_vector.h>
#include <hk_math/matrix/matrix_inverter.h>

/*
 * MULT
 */
void hk_Dense_Matrix_Util::mult( const hk_Dense_Matrix& m, const hk_Dense_Vector& in, hk_Dense_Vector& out )
{
	const int rows = m.get_num_rows();
	const int cols = m.get_num_cols();

	HK_ASSERT( cols == in.get_size() );
	HK_ASSERT( in.get_size() == out.get_size() );

	out.set_zero();

	for(int c=0; c<cols; ++c)
	{
		for(int r=0; r<rows; ++r)
		{
			out(r) += m(r,c) * in(c);
		}
	}
}

/*
 * MULT
 */
void hk_Dense_Matrix_Util::mult( const hk_Dense_Matrix& m, const hk_real * in, hk_real * out )
{
	const int rows = m.get_num_rows();
	const int cols = m.get_num_cols();


	for(int r=0; r<rows; ++r)
	{
		hk_real sum = 0.0f;
		int c = 0;
		do {
			sum += m(r,c) * in[c];
		}while ( ++c < cols );
		out[r] = sum;
	}
}


/*
 * INVERT
 */

// matrix invert with pivoting - we exchange rows if the pivot
// is less than the tolerance
/*
hk_result hk_Dense_Matrix_Util::invert(hk_Dense_Matrix& m, hk_real tolerance)
{
	const int num_rows = m.get_num_rows();
	const int num_cols = m.get_num_cols();
	const int lda_size = m.get_lda();

	HK_ASSERT( num_cols == num_rows );
	HK_ASSERT( tolerance>= 0.0f );

	HK_BREAKPOINT(); // this inverter does not work

	int* permute = hkAllocate<int>( num_cols, HK_MEMORY_CLASS_UNKNOWN);

	for(int cur_row=0; cur_row<num_rows; ++cur_row)
	{
		// find best pivot

		hk_real f_pivot = 0;	// hkMath::fabs pivot
		int i_pivot;			// index pivot
		{
			for( int i=cur_row; i<num_cols; ++i)
			{
				hk_real fp = hk_Math::fabs(m(i,cur_row));

				if(fp > f_pivot)
				{
					i_pivot = i;
					f_pivot = fp;
				}
			}

			if( f_pivot < tolerance)
			{
				return HK_FAULT;
			}
		}

		// permute rows

		{
			permute[cur_row] = i_pivot;
			if(i_pivot != cur_row )
			{
				for( int i=0; i<num_cols; ++i)
				{
					hk_real t_ri = m(cur_row,i);
					m(cur_row,i) = m(i_pivot, i);
					m(i_pivot,i) = t_ri;
				}
			}
		}

		// scale this row
		{
			hk_real inv_pivot = 1.0f / m(cur_row,cur_row);
			m(cur_row,cur_row) = 1.0f;
			for( int i=0; i<num_cols; ++i)
			{
				m(cur_row, i) *= inv_pivot;
			}
		}

		// add to all other rows bar itself
		{
			for(int i=0; i<num_rows; ++i)
			{
				if( i != cur_row )
				{
					hk_real factor = m(i, cur_row);
					m(i, cur_row) = 0;

					for( int j=0; j<num_cols; ++j)
					{
						m(i,j) -= m(cur_row,j)*factor;
					}
				}
			}
		}
	}

	// now permute columns back
	for(int i=num_rows-2; i>=0; --i)
	{
		int perm = permute[i];
		if( perm != i )
		{
			for(int j=i+1; j<num_cols; j++)
			{
				hk_real t_jx = m(j,perm);
				m(j,perm) = m(j,i);
				m(j,i) = t_jx;
			}
		}
	}

	hkDeallocate(permute);

	return HK_OK;
}
*/

hk_result hk_Dense_Matrix_Util::invert_6x6(hk_Fixed_Dense_Matrix<6>& m, hk_real tolerance)
{
	hk_real inverted[ 6 * 8 ];

	hk_Matrix_Inverter::invert_no_pivot_search( m.get_elems(), m.get_lda(), m.get_num_rows(), inverted);

	hk_real *s = &inverted[0];
	hk_real *d = m.get_elems();
	for (int i = 6-1; i>=0; i--){
		d[0] = s[0];
		d[1] = s[1];
		d[2] = s[2];
		d[3] = s[3];
		d[4] = s[4];
		d[5] = s[5];
		s += m.get_lda();
		d += m.get_lda();
	}
	return HK_OK;
}

hk_result hk_Dense_Matrix_Util::invert_5x5(hk_Fixed_Dense_Matrix<5>& m, hk_real tolerance)
{
	hk_real inverted[ 5 * 8 ];

	hk_Matrix_Inverter::invert_no_pivot_search( m.get_elems(), m.get_lda(), m.get_num_rows(), inverted);

	hk_real *s = &inverted[0];
	hk_real *d = m.get_elems();
	for (int i = 5-1; i>=0; i--){
		d[0] = s[0];
		d[1] = s[1];
		d[2] = s[2];
		d[3] = s[3];
		d[4] = s[4];
		s += m.get_lda();
		d += m.get_lda();
	}
	return HK_OK;
}

hk_result hk_Dense_Matrix_Util::invert_4x4(hk_Fixed_Dense_Matrix<4>& m, hk_real tolerance)
{
	hk_real inverted[ 4 * 4 ];

	hk_Matrix_Inverter::invert_no_pivot_search( m.get_elems(), m.get_lda(), m.get_num_rows(), inverted);

	hk_real *s = &inverted[0];
	hk_real *d = m.get_elems();
	for (int i = 4-1; i>=0; i--){
		d[0] = s[0];
		d[1] = s[1];
		d[2] = s[2];
		d[3] = s[3];
		s += m.get_lda();
		d += m.get_lda();
	}
	return HK_OK;
}


hk_result hk_Dense_Matrix_Util::invert_3x3_symmetric(hk_Dense_Matrix& m, hk_real tolerance)
{
	hk_real *co0 = m.getRealPointer();
	hk_real *co1 = co0 + m.getLda();
	hk_real *co2 = co1 + m.getLda();

	hk_Vector3 r0; r0.set_cross( *(hk_Vector3 *)co1, *(hk_Vector3 *)co2 );
    hk_Vector3 r1; r1.set_cross( *(hk_Vector3 *)co2, *(hk_Vector3 *)co0 );
    hk_Vector3 r2; r2.set_cross( *(hk_Vector3 *)co0, *(hk_Vector3 *)co1 );

    hk_real D = ((hk_Vector3 *)co0)->dot(r0);	// main determinant
    
    if( hk_Math::fabs(D)< tolerance * tolerance * tolerance ){
		return HK_FAULT;  // cannot invert, may happen
    }

    hk_real DI = 1.0f/D;

	((hk_Vector3 *)co0)->set_mul( DI, r0 );
	((hk_Vector3 *)co1)->set_mul( DI, r1 );
	((hk_Vector3 *)co2)->set_mul( DI, r2 );
	return HK_OK;
}

hk_result hk_Dense_Matrix_Util::invert_2x2(const hk_Dense_Matrix& m, hk_Dense_Matrix& out, hk_real tolerance)
{
	const hk_real *co0 = m.get_const_real_pointer();
	const hk_real *co1 = co0 + m.getLda();

	hk_real D =  (co0[0] * co1[1]) - (co0[1] * co1[0]);

    if(D*D < tolerance * tolerance){
		return HK_FAULT;  // may be an ordinary result
    }
    
    hk_real DI = 1.0f/D;
    hk_real a1 =  co1[1] * DI;
    hk_real b1 = -co0[1] * DI;
    hk_real a2 = -co1[0] * DI;
    hk_real b2 =  co0[0] * DI;

	hk_real *o_co0 = out.getRealPointer();
	hk_real *o_co1 = o_co0 + out.getLda();

	o_co0[0] = a1;
	o_co1[0] = b1;
	o_co0[1] = a2;
	o_co1[1] = b2;
	return HK_OK;
}


/*
 * SOLVE
 */
hk_result hk_Dense_Matrix_Util::solve(
		hk_Dense_Matrix& m,
		hk_Dense_Vector& v,
		hk_real tolerance)
{
	int rows = m.get_num_rows();
	int cols = m.get_num_cols();

	HK_ASSERT(rows==cols);
	HK_ASSERT(rows==v.get_size());
  
	for(int elim_row=0; elim_row<rows; elim_row++)
	{
		hk_real pivot = m(elim_row,elim_row);
		hk_real fpivot = hk_Math::fabs(pivot);
	
		// pivot too small, try to find another
		if( fpivot < tolerance )
		{
			int swap_row = -1;
			for(int i=elim_row+1; i<rows; i++)
			{
				if( hk_Math::fabs(m(i,elim_row)) > fpivot)
				{
					swap_row = i;
					pivot = m(i,elim_row);
					fpivot = hk_Math::fabs(pivot);
				}
			}

			// if didnt find good row to swap with
			if(swap_row < 0)
			{
				return HK_FAULT;
			}

			// now swap rows in a=matrix
			for(int j=elim_row; j<cols; j++)
			{
				hk_real h = m(elim_row,j);
				m(elim_row,j) = m(swap_row,j);
				m(swap_row,j) = h;
			}
			// and swap elts in v
			{
				hk_real h = v(elim_row);
				v(elim_row) = v(swap_row);
				v(swap_row) = h;
			}
		}
	
		hk_real inv_pivot = 1.0f/pivot;
		for(int i=elim_row+1; i<rows; i++)
		{
			hk_real factor = -inv_pivot * m(i,elim_row);
			m(i, elim_row) = 0.0f;
			for(int j=elim_row+1; j<cols; j++)
			{
				m(i,j) += factor * m(elim_row,j);
			}
			v(i) += factor * v(elim_row);
		}
	}

	// now back substitute
	v(rows-1) /= m(rows-1,rows-1);
	for(int i=rows-2; i>=0; --i)
	{
		hk_real sum = 0.0f;
		for(int j=i+1; j<rows; j++)
		{
			sum += m(i,j) * v(j);
		}
		v(i) = (v(i)-sum) / m(i,i);
	}

	return HK_OK;
}


void hk_Dense_Matrix_Util::print(const hk_Dense_Matrix &m)
{
	for (int row = 0; row < m.get_num_rows(); row ++)
	{
		hkprintf("%2i: ", row);
		for (int col = 0; col < m.get_num_cols(); col ++)
		{
			hkprintf("%3.3f, ", m( row,col));
		}
		hkprintf("\n");
	}
}
