/**
 * @brief Eigen�������� 3x3 �s��̌ŗL�l�A�ŗL�x�N�g���Ǝ��o���T���v���v���O����
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
		const double ev = solver.eigenvalues()( i,0 ).real(); //�ŗL�l
		const Eigen::Vector3d v =  solver.eigenvectors().col(i).real(); //�ŗL�x�N�g��
		std::cerr<<i<<" : "<<ev<<" ( "<<v.x()<<", "<<v.y()<<", "<<v.z()<<")"<<std::endl;
	}
	return 0;
}