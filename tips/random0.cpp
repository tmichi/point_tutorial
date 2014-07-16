#include <iostream>
// using XorShift RNG.
#ifndef uint32_t
typedef unsigned int uint32_t;
#endif
uint32_t xor128(void) { 
	static uint32_t x = 123456789;
	static uint32_t y = 362436069;
	static uint32_t z = 521288629;
	static uint32_t w = 88675123; 
	uint32_t t;
 
	t = x ^ (x << 11);
	x = y;
        y = z; 
        z = w;
	return w = (w ^ (w >> 19)) ^ (t ^ (t >> 8)); 
}

int main ( int argc, char** argv ) {
	for( int i = 0 ; i < 100 ; ++ i ) {
		int v = xor128() % 100;
		std::cerr<<v<< " ";
	}
	std::cerr<<std::endl;
	return 0;
}
