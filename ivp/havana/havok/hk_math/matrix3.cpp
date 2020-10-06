#include <hk_math/vecmath.h>

void hk_Matrix3::set_rows( const hk_Vector3& r0,
							const hk_Vector3& r1,
							const hk_Vector3& r2)
{
	_set_rows( r0,r1,r2);
}

void hk_Matrix3::set_cols( const hk_Vector3& c0,			   
						    const hk_Vector3& c1,
							const hk_Vector3& c2)
{
	get_column(0) = c0;
	get_column(1) = c1;
	get_column(2) = c2;
}

void hk_Matrix3::set_identity_rotation()
{
	get_column(0).set(1,0,0);
	get_column(1).set(0,1,0);
	get_column(2).set(0,0,1);
}

bool hk_Matrix3::is_identity_rotation() const
{
	const hk_Matrix3& m = *this;
	return	m(0,0)==1 && m(0,1)==0 && m(0,2)==0 &&
			m(1,0)==0 && m(1,1)==1 && m(1,2)==0 &&
			m(2,0)==0 && m(2,1)==0 && m(2,2)==1;
}

void hk_Matrix3::set_zero()
{
	get_column(0).set_zero();
	get_column(1).set_zero();
	get_column(2).set_zero();
}

void hk_Matrix3::set_diagonal( hk_real m00, hk_real m11, hk_real m22 )
{
	get_column(0).set(m00,0,0);
	get_column(1).set(0,m11,0);
	get_column(2).set(0,0,m22); 
}

hk_result hk_Matrix3::invert(hk_real epsilon)
{
	hk_Vector3 r0; r0.set_cross( get_column(1), get_column(2) );
    hk_Vector3 r1; r1.set_cross( get_column(2), get_column(0) );
    hk_Vector3 r2; r2.set_cross( get_column(0), get_column(1) );

    hk_real D = get_column(0).dot(r0);	// main determinant
    
    if( hk_Math::fabs(D)< epsilon ){
		return HK_FAULT;  // cannot invert, may happen
    }

    hk_real DI = 1.0f/D;
    r0 *= DI;
    r1 *= DI;
    r2 *= DI;

	_set_rows( r0, r1, r2 );
	return HK_OK;
}

void hk_Matrix3::transpose()
{
	hk_real swap;
	swap = get_column(0)(1);	get_column(0)(1) = get_column(1)(0); get_column(1)(0) = swap;
	swap = get_column(0)(2);	get_column(0)(2) = get_column(2)(0); get_column(2)(0) = swap;
	swap = get_column(1)(2);	get_column(1)(2) = get_column(2)(1); get_column(2)(1) = swap;
}

// r~ = [ 0 -z  y ]
//      [ z  0 -x ]
//      [-y  x  0 ]
void hk_Matrix3::set_cross_skew( const hk_Vector3& r, const hk_Matrix3& R )
{
	for( int col = 0; col < 3; col++ )
	{
		hk_real a = -r.z*R(1,col) + r.y*R(2,col);
		hk_real b =  r.z*R(0,col) - r.x*R(2,col);
		hk_real c = -r.y*R(0,col) + r.x*R(1,col);
		get_column(col).set( a,b,c );
	}
}


void hk_Matrix3::set_mul3( const hk_Matrix3& Ma, const hk_Matrix3& Mb )
{
	HK_ASSERT(this!=&Ma);
	HK_ASSERT(this!=&Mb);

	for( int i = 0; i < 3; i++ )
	{
		hk_real x = Ma(0,0)*Mb(0,i) + Ma(0,1)*Mb(1,i) + Ma(0,2)*Mb(2,i);
		hk_real y = Ma(1,0)*Mb(0,i) + Ma(1,1)*Mb(1,i) + Ma(1,2)*Mb(2,i);
		hk_real z = Ma(2,0)*Mb(0,i) + Ma(2,1)*Mb(1,i) + Ma(2,2)*Mb(2,i);

		get_column(i).set(x,y,z);
	}
}

void hk_Matrix3::set_mul3_inv2( const hk_Matrix3& Ma, const hk_Rotation& Mb )
{
	for( int i = 0; i < 3; i++ ){
		hk_real x = Ma(0,0)*Mb(i,0) + Ma(0,1)*Mb(i,1) + Ma(0,2)*Mb(i,2);
		hk_real y = Ma(1,0)*Mb(i,0) + Ma(1,1)*Mb(i,1) + Ma(1,2)*Mb(i,2);
		hk_real z = Ma(2,0)*Mb(i,0) + Ma(2,1)*Mb(i,1) + Ma(2,2)*Mb(i,2);

		get_column(i).x = x;
		get_column(i).y = y;
		get_column(i).z = z;
	}
}

void hk_Matrix3::set_mul3_inv( const hk_Rotation& Ma, const hk_Matrix3& Mb )
{
	for( int i = 0; i < 3; i++ ){
		hk_real x = Ma(0,0)*Mb(0,i) + Ma(1,0)*Mb(1,i) + Ma(2,0)*Mb(2,i);
		hk_real y = Ma(0,1)*Mb(0,i) + Ma(1,1)*Mb(1,i) + Ma(2,1)*Mb(2,i);
		hk_real z = Ma(0,2)*Mb(0,i) + Ma(1,2)*Mb(1,i) + Ma(2,2)*Mb(2,i);

		get_column(i).x = x;
		get_column(i).y = y;
		get_column(i).z = z;
	}
}


void hk_Matrix3::operator += ( const hk_Matrix3& Ma )
{
	for( int i = 0; i < 3; i++ ){
		get_column(i) += Ma.get_column(i);
	}
}

void hk_Matrix3::operator -= ( const hk_Matrix3& Ma )
{
	for( int i = 0; i < 3; i++ ){
		get_column(i) -= Ma.get_column(i);
	}
}

void hk_Matrix3::rotate ( int axis, hk_real angle )
{
	hk_Matrix3 rotation;
	rotation.set_zero();

	int x = axis;
	int y = x+1;
	if ( y == 3) y = 0;
	int z = y + 1;
	if ( z == 3) z = 0;

	hk_real cos_alpha = hk_Math::cos(angle);
	hk_real sin_alpha = hk_Math::sin(angle);

	rotation.get_column( y )( y ) = cos_alpha;
	rotation.get_column( z )( z ) = cos_alpha;

	rotation.get_column( y )( z ) = sin_alpha;
	rotation.get_column( z )( y ) = -sin_alpha;

	rotation.get_column( x )( x ) = 1.0f;

	hk_Matrix3 rotated;
	rotated.set_mul3( *this, rotation );
	*this = rotated;
}


void hk_Matrix3::set_rotated_diagonal_matrix( const hk_Rotation &r, const hk_Vector3 &diagonal_matrix3 )
{
	hk_Matrix3 t;
	t.get_column(0).set_mul( diagonal_matrix3(0) , r.get_column(0) );
	t.get_column(1).set_mul( diagonal_matrix3(1) , r.get_column(1) );
	t.get_column(2).set_mul( diagonal_matrix3(2) , r.get_column(2) );

	this->set_mul3_inv2( t, r );

}

