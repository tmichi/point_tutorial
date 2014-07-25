#include <iostream>
#include <Eigen/Dense>
class Projector {
private:
	Projector ( const Projector& that);
	void operator =  ( const Projector& that);
private:
	Eigen::Vector3d _normal;
	Eigen::Vector3d _point;
	Eigen::Vector3d _b0;
	Eigen::Vector3d _b1;
public : 
	Projector ( const Eigen::Vector3d& normal, const Eigen::Vector3d& point) {
		this->_normal = normal.normalized();
		this->_point  = point;
		const Eigen::Vector3d random = Eigen::Vector3d::Random().normalized();
		this->_b0 = this->_normal.cross ( random ).normalized();
		this->_b1 = this->_b0.cross ( this->_normal).normalized();
		return;
	}

	Eigen::Vector3d project ( const Eigen::Vector3d &p ) const {
		const Eigen::Vector3d v = p - this->_point;
		const double u = this->_normal.dot(v);
		const double s = this->_b0.dot(v);
		const double t = this->_b1.dot(v);
		return Eigen::Vector3d(s,t,u);
	}

	Eigen::Vector3d unproject( const double s, const double t, const double u = 0 ) const {
		return this->_point + s * this->_b0 + t * this->_b1 + u * this->_normal;
	}
};

int main ( int argc, char** argv ) {

	Eigen::Vector3d nrm(1, 3, 2);
	nrm.normalize();
	Eigen::Vector3d p0 ( 1, 3, 4);

	Eigen::Vector3d p1 ( 3, 4, 3 ) ;

	Projector projector(nrm, p0);
	Eigen::Vector3d p2 = projector.project(p1);
	std::cerr<<p2<<std::endl;
	Eigen::Vector3d p3 = projector.unproject( p2.x(), p2.y(), p2.z());

	std::cerr<<p3<<std::endl;

	return 0;
}
