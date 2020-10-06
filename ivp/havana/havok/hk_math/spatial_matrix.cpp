#include <hk_math/vecmath.h>
#include <hk_math/spatial_matrix.h>
#include <hk_math/spatial_vector.h>

// construct spatial isolated inertia matrix from I and m
// [ 0 M ]
// [ I 0 ]
void hk_Spatial_Matrix::set_spatial_inertia_tensor( const hk_Matrix3& inertia_tensor, const hk_real mass )
{
	m_Block[0][0].set_zero();
	m_Block[0][1].set_diagonal( mass, mass, mass );
	m_Block[1][0] = inertia_tensor;
	m_Block[1][1].set_zero();
}


// spatial transform from frame F to G = 
// [ R    0 ]   where R is rotation taking vector from F to G
// [ -r~R R ]   and r is vector from origin of F to origin of G in G's frame of reference
// 
void hk_Spatial_Matrix::set_spatial_transform( const hk_Matrix3& R, const hk_Vector3& r )
{
	hk_Matrix3 rR;
	hk_Vector3 r_neg( -r.x, -r.y, -r.z );
	rR.set_cross_skew( r_neg, R ); 
	m_Block[0][0] = R;
	m_Block[0][1].set_zero();
	m_Block[1][1] = R;
	m_Block[1][0] = rR;
}

// spatial transform from frame F to G = 
// [ R    0 ]   where R is transform taking vector from F to G
// [ -r~R R ]   and r is vector from origin of F to origin of G in G's frame of reference
// 
void hk_Spatial_Matrix::set_spatial_transform( const hk_Transform& cartesian_frame )
{
	HK_ASSERT(0);
/*	hk_Matrix3 rR;
	hk_real raw[16];
	cartesian_frame.get_4x4_column_major( raw );
	hk_Matrix3 R;
//!me
	R.set_cols( hk_Vector3(raw[0], raw[1], raw[2]  ),
				hk_Vector3(raw[4], raw[5], raw[6]  ),
				hk_Vector3(raw[8], raw[9], raw[10] ));
	
//!me ack figure out which it should be!!!
	R.set_cols( hk_Vector3(raw[0], raw[4], raw[8]  ),
				hk_Vector3(raw[1], raw[5], raw[9]  ),
				hk_Vector3(raw[2], raw[6], raw[10] ));

	hk_Vector3 r;
	r.set_rotated_dir( cartesian_frame, cartesian_frame.get_translation() );

	hk_Vector3 r_neg( -r.x, -r.y, -r.z );
	rR.set_cross_skew( r_neg, R ); 
	m_Block[0][0] = R;
	m_Block[0][1].set_zero();
	m_Block[1][1] = R;
	m_Block[1][0] = rR;*/
}

void hk_Spatial_Matrix::set_mul( hk_Spatial_Matrix& a, hk_Spatial_Matrix& b )
{
	hk_Matrix3 work;

	work.set_mul3( a.m_Block[0][0], b.m_Block[0][0] );
	m_Block[0][0].set_mul3( a.m_Block[0][1], b.m_Block[1][0] );
	m_Block[0][0] += work;

	work.set_mul3( a.m_Block[0][0], b.m_Block[0][1] ); 
	m_Block[0][1].set_mul3( a.m_Block[0][1], b.m_Block[1][1] );
	m_Block[0][1] += work;

	work.set_mul3( a.m_Block[1][0], b.m_Block[0][0] );
	m_Block[1][0].set_mul3( a.m_Block[1][1], b.m_Block[1][0] );
	m_Block[1][0] += work;

	work.set_mul3( a.m_Block[1][0], b.m_Block[0][1] );
	m_Block[1][1].set_mul3( a.m_Block[1][1], b.m_Block[1][1] );
	m_Block[1][1] += work;

}


void hk_Spatial_Matrix::set_add( hk_Spatial_Matrix& a, hk_Spatial_Matrix& b )
{

	m_Block[0][0] = a.m_Block[0][0];
	m_Block[0][0] += b.m_Block[0][0];

	m_Block[1][0] = a.m_Block[1][0];
	m_Block[1][0] += b.m_Block[1][0];

	m_Block[0][1] = a.m_Block[0][1];
	m_Block[0][1] += b.m_Block[0][1];

	m_Block[1][1] = a.m_Block[1][1];
	m_Block[1][1] += b.m_Block[1][1];

}


void hk_Spatial_Matrix::set_sub( hk_Spatial_Matrix& a, hk_Spatial_Matrix& b )
{

	m_Block[0][0] = a.m_Block[0][0];
	m_Block[0][0] -= b.m_Block[0][0];

	m_Block[1][0] = a.m_Block[1][0];
	m_Block[1][0] -= b.m_Block[1][0];

	m_Block[0][1] = a.m_Block[0][1];
	m_Block[0][1] -= b.m_Block[0][1];

	m_Block[1][1] = a.m_Block[1][1];
	m_Block[1][1] -= b.m_Block[1][1];

}

void hk_Spatial_Matrix::operator += ( const hk_Spatial_Matrix& Ma )
{

	m_Block[0][0] += Ma.m_Block[0][0];
	m_Block[1][0] += Ma.m_Block[1][0];
	m_Block[0][1] += Ma.m_Block[0][1];
	m_Block[1][1] += Ma.m_Block[1][1];

}

void hk_Spatial_Matrix::operator -= ( const hk_Spatial_Matrix& Ma )
{

	m_Block[0][0] -= Ma.m_Block[0][0];
	m_Block[1][0] -= Ma.m_Block[1][0];
	m_Block[0][1] -= Ma.m_Block[0][1];
	m_Block[1][1] -= Ma.m_Block[1][1];

}

void hk_Spatial_Matrix::operator *= ( hk_real a )
{
	int idr,idc;

	for( idr = 0; idr < 3; idr++ ){
		for( idc = 0; idc < 3; idc++ ){
		  m_Block[0][0].set_elem(idr, idc, m_Block[0][0](idr, idc)*a);
		  m_Block[1][0].set_elem(idr, idc, m_Block[1][0](idr, idc)*a);
		  m_Block[0][1].set_elem(idr, idc, m_Block[0][1](idr, idc)*a);
		  m_Block[1][1].set_elem(idr, idc, m_Block[1][1](idr, idc)*a);
		} 
	}
}

void hk_Spatial_Matrix::set_vector_mul_vector( const hk_Spatial_Vector &a, const hk_Spatial_Vector &b )
{
	int idr, idc;
	
	for( idr = 0; idr < 3; idr++ ){
		for( idc = 0; idc < 3; idc++ ){
		  m_Block[0][0].set_elem(idr, idc, a.top(idr)*b.bottom(idc));
		} 
	}
	
	for( idr = 0; idr < 3; idr++ ){
		for( idc = 0; idc < 3; idc++ ){
		  m_Block[0][1].set_elem(idr, idc, a.top(idr)*b.top(idc));
		} 
	}
	
	for( idr = 0; idr < 3; idr++ ){
		for( idc = 0; idc < 3; idc++ ){
		  m_Block[1][0].set_elem(idr, idc, a.bottom(idr)*b.bottom(idc));
		} 
	}
	
	for( idr = 0; idr < 3; idr++ ){
		for( idc = 0; idc < 3; idc++ ){
		  m_Block[1][1].set_elem(idr, idc, a.bottom(idr)*b.top(idc));
		} 
	}
}

void hk_Spatial_Matrix::set_identity()
{
	m_Block[0][0].set_identity_rotation();
	m_Block[1][1].set_identity_rotation();
	m_Block[0][1].set_zero();
	m_Block[1][0].set_zero();
}


#define MIN_REAL 0.0000001f
#define MAX_REAL 10000000.0f

//!me temporary..... Steve, you probably have a better solver
void hk_Spatial_Matrix::gausse_siedel_solve( hk_real A[][6], int dim, hk_real *x, hk_real *b )
{

static int *order;
static int order_size = 0;	
	
hk_real max_pivot, factor;
int elimcol, elimrow, check_pivot, backcol, backrow, target_row, target_col;
int max_order_row_index;

	if( dim > order_size ){
		delete order;
		order = new int[ dim ]; //(int *)malloc( dim * sizeof(real) );
		order_size = dim;
	}

	for( target_row = 0; target_row < dim; target_row++ ){
		order[target_row] = target_row;
	}

	// eliminate all columns
	for( elimcol = 0; elimcol < dim; elimcol++ ){
		
		max_order_row_index = elimcol;

		// get row with largest value at pivot from set of rows not eliminated already
		for( max_pivot = 0, check_pivot = elimcol; check_pivot < dim; check_pivot++ ){
			if( max_pivot < hk_Math::fabs(A[order[check_pivot]][elimcol]) ){
				max_pivot = hk_Math::fabs(A[order[check_pivot]][elimcol]);	
				max_order_row_index = check_pivot;
			}
		}

		// swap rows in order
		elimrow = order[max_order_row_index];
		order[max_order_row_index] = order[elimcol];
		order[elimcol] = elimrow;
		
		// elimination:  only eliminate all rows that haven't been reduced already
		for( target_row = elimcol+1; target_row < dim; target_row++ ){
			if( A[elimrow][elimcol] == 0.0f ){ //!me should report errors
        //!me instead of failing I should try to fix it the best I can.
        //!me much better to have wierd results than to fail completely.
        //!me Here's what to do:  look up cholesky factorization
        //!me or do this: try to figure out what vector is missing in 
        //!me order to get b.  take dot product with b and subract from
        //!me b for every row ( column? ) then whatever is left is 
        //!me what this 0 row should be ( might have to take it from 
        //!me column space to row space first ). Cool!?!?!


				factor = 0.0f;  // the whole row was 0.0, so fail nicely
//				printf( "shit singular matrix\n");
//				exit(1);
			}else{
				if( hk_Math::fabs(A[elimrow][elimcol]) < MIN_REAL ){  
	//			exit(1);
					factor = MAX_REAL;
				}else{
					factor = A[order[target_row]][elimcol]/A[elimrow][elimcol];
				}
			}

			// do each column for this elimanation step that hasn't been eliminated already
			for( target_col = elimcol; target_col < dim; target_col++ ){
				A[order[target_row]][target_col] -= factor*A[elimrow][target_col];
			}

			// modify b as well for the elimination
			b[order[target_row]] -= factor*b[elimrow];
		}
	
	}

	// find solutions with back substitution
	for( backcol = dim-1; backcol >= 0; backcol-- ){
		
		x[backcol] = b[order[backcol]];
		for( backrow = dim-1; backrow > backcol; backrow-- ){
			x[backcol] -= A[order[backcol]][backrow]*x[backrow];
		}

		if( A[order[backcol]][backcol] == 0 ){
		//		exit(1);
			x[backcol] = 0;  //!me fail nicely
		}else{
			x[backcol] = x[backcol]/A[order[backcol]][backcol];
		}

	}

}



void hk_Spatial_Matrix::linear_solve( hk_Spatial_Vector &sx, const hk_Spatial_Vector &sb )
{
	hk_real x[6];
    hk_real b[6];
	hk_real A[6][6];

    int idx, jdx;  
	
    for( idx = 0; idx < 3; idx++ ){
		x[idx] = sx.top(idx);
		b[idx] = sb.top(idx);
		x[idx+3] = sx.bottom(idx);
		b[idx+3] = sb.bottom(idx);
	    for( jdx = 0; jdx < 3; jdx++ ){
			A[idx][jdx] = m_Block[0][0](idx,jdx);
			A[idx+3][jdx] = m_Block[1][0](idx,jdx);
			A[idx+3][jdx+3] = m_Block[1][1](idx,jdx);
			A[idx][jdx+3] = m_Block[0][1](idx,jdx);
		}

	}

    // solve this sucker
    gausse_siedel_solve( A, 6, x, b );

    for( idx = 0; idx < 6; idx++ ){
		sx(idx) = x[idx];
	}
	
}

