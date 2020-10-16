
hk_Spatial_Matrix::hk_Spatial_Matrix()
{
}



// not very quick.  could probably do some whacked direct memory access.
//!me todo: whacked memory access for better performance
hk_real hk_Spatial_Matrix::operator() (int r, int c) const
{
	int blockr, blockc;
	if( r < 3 ){
		blockr = 0;
	}else{
		blockr = 1;
		r -= 3;
	}

	if( c < 3 ){
		blockc = 0;
	}else{
		blockc = 1;
		c -= 3;
	}

	return m_Block[blockr][blockc]( r, c );
}



