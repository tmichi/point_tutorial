/*
 * ポインタの使い方
 */
#include <iostream>
#include <cstdlib>

void dynamic_array_allocation ( const int n ) {
        int *a = new int [n]; // allocation
        for( int i = 0 ; i < n ; ++i ) {
               a[i] = 100 - i;
        }
        for( int i = 0 ; i < n ; ++i ) {
               std::cerr<<a[i]<<" ";
        }
        std::cerr<<std::endl;

        delete[] a; // free memory 
}

// n 点ランダムに発生させて最大と最小を返す
void multiple_return_values ( int* min_value, int* max_value ) {
        int n = 1000;
        for( int i = 0 ; i < n; ++i ) {
                int value = rand();
                if ( i == 0 ) {
                        *min_value = value;
                        *max_value = value;
                }
                else {
                        if ( value < *min_value ) *min_value = value;
                        if ( value > *max_value ) *max_value = value;
                }
        }
        std::cerr<<"min_value :"<<*min_value<<std::endl;
        std::cerr<<"max_value :"<<*max_value<<std::endl;

}

int main ( int argc, char** argv ) {
        dynamic_array_allocation(100) ;

        int min_value, max_value;
        multiple_return_values ( &min_value, &max_value);
        std::cerr<<"min_value :"<<min_value<<std::endl;
        std::cerr<<"max_value :"<<max_value<<std::endl;

        return 0;
}
