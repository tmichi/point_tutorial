#include <iostream>
#include <vector>
#include <algorithm>

int main ( int argc, char** argv ) {
	std::vector<int> v;
	for( int i = 0 ; i < 100 ; ++ i ) {
		v.push_back(i);
	}
	std::random_shuffle(v.begin(), v.end());

	for( int i = 0 ; i < 100 ; ++ i ) {
		std::cerr<<v[i]<<" ";
	}
	std::cerr<<std::endl;
	return 0;
}
