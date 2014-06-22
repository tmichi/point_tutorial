/**
 * @brief Eigenをつかって 3x3 行列の固有値、固有ベクトルと取り出すサンプルプログラム
 * @author Takashi Michikawa@acm.org
 */
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <iostream>
int main(int argc, char** argv) {
	Eigen::Matrix3d m;
	m<<3, 1, 2, 
           1, 3, 7,
	   2, 7, 4;

	Eigen::EigenSolver<Eigen::Matrix3d> solver( m );
	for( int i = 0 ; i < 3 ; ++i ) {
		const double ev = solver.eigenvalues()( i,0 ).real(); //固有値
		const Eigen::Vector3d v =  solver.eigenvectors().col(i).real(); //固有ベクトル
		std::cerr<<i<<" : "<<ev<<" ( "<<v.x()<<", "<<v.y()<<", "<<v.z()<<")"<<std::endl;
	}
	return 0;
}