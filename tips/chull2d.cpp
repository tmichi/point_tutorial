// Convex-hull code is based on : 
// http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
#include <Eigen/Dense>
#include <vector>
#include <algorithm>
#include <iostream>
#include <fstream>
class Vector3dCompare {
public:
	bool operator () (const Eigen::Vector3d &p0 ,const Eigen::Vector3d &p1 ) const {
		return p0.x() < p1.x();
	}
};

double cross( const Eigen::Vector3d& p0,  const Eigen::Vector3d& p1,  const Eigen::Vector3d& p2)
{
	return (p1.x() - p0.x()) * (p2.y() - p0.y()) - 
	       (p1.y() - p0.y()) * (p2.x() - p0.x());
}
 
// Returns a list of points on the convex hull in counter-clockwise order.
// Note: the last point in the returned list is the same as the first one.
void convex_hull(std::vector<Eigen::Vector3d>& p, std::vector<Eigen::Vector3d>& chull)
{
	int n = static_cast<int>(p.size());
	int k = 0;
	chull.assign ( 2 * n , Eigen::Vector3d()) ;
	std::sort(p.begin(), p.end(), Vector3dCompare());
 
	// Build lower hull
	for (int i = 0; i < n; ++i) {
		while (k >= 2 && cross(chull[k-2], chull[k-1], p[i]) <= 0) {
			k--;
		}
		chull[k++] = p[i];
	}
 
	// Build upper hull
	for (int i = n - 2, t = k+1; i >= 0; i--) {
		while (k >= t && cross(chull[k-2], chull[k-1], p[i]) <= 0) k--;
		chull[k++] = p[i];
	}
 
	chull.resize(k);
	return ;
}
int main ( int argc, char** argv ) {
	std::vector<Eigen::Vector3d> p;
	for( int i = 0 ; i < 1000; ++i ) {
		double x = (rand() % 601)+200;
		double y = (rand() % 601)+200;
		p.push_back( Eigen::Vector3d(x, y, 0) );
	}
	
	std::vector<Eigen::Vector3d> chull;
	convex_hull (p, chull);
	
	std::ofstream fout("out.svg");
	fout<<"<?xml version=\"1.0\" standalone=\"no\"?>"<<std::endl
		 <<"<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">"<<std::endl
		 <<"<svg width=\"1000pt\" height=\"1000pt\" viewBox=\"0 0 1000 1000\" xmlns=\"http://www.w3.org/2000/svg\">"<<std::endl;
	fout<<"<polygon points=\"";
	for( int i = 0 ; i < chull.size()-1; i++ ) {
		fout<<chull[i].x()<<","<<chull[i].y()<<" ";
	}
	fout<<"\" "<<"stroke-width=\"4\" stroke=\"indianred\" fill=\"mistyrose\"/>"<<std::endl;

		
	for( int i = 0 ; i < 1000 ; i++ ) {
		fout<<"<circle cx=\""<<p[i].x()<<"\" cy=\""<<p[i].y()<<"\" r=\"3\" fill=\"black\"/>"<<std::endl;
	}
	for( int i = 0 ; i < chull.size() ; i++ ) {
		fout<<"<circle cx=\""<<chull[i].x()<<"\" cy=\""<<chull[i].y()<<"\" r=\"3\" fill=\"red\"/>"<<std::endl;
	}

	fout<<"</svg>"<<std::endl;
	return 0;
}
