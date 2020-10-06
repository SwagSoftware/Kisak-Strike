#include <hk_math/vecmath.h>

// BSD rand function
unsigned int hk_Math::hk_random_seed = 'h'+'a'+'v'+'o'+'k';

void hk_Math::srand01( unsigned seedVal )
{
	hk_random_seed = seedVal;
}

hk_real hk_Math::rand01()
{
	const unsigned a = 1103515245;
	const unsigned c = 12345;
	const unsigned m = unsigned(-1) >> 1;
	hk_random_seed = (a * hk_random_seed + c ) & m;
	return hk_real(hk_random_seed) / m;
}
