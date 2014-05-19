#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
int main(){
	Eigen::Matrix3d m;
	m<<3, 1, 2, 
		1, 3,  7,
		2, 7, 4;

	Eigen::EigenSolver<Eigen::Matrix3d> solver( m );

	for( int i = 0 ; i < 3 ; ++i ) {
		double ev = solver.eigenvalues()( i,0 ).real();
		Eigen::Vector3d v;
		v.x() = solver.eigenvectors().col( i ).x().real();
		v.y() = solver.eigenvectors().col( i ).y().real();
		v.z() = solver.eigenvectors().col( i ).z().real();
		std::cerr<<i<<":"<<ev<<"( "<<v.x()<<" "<<v.y()<<" "<<v.z()<<")"<<std::endl;
	}
	return 0;
}