#pragma once 
#ifndef NORMAL_H
#define NORMAL_H 1
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <PointCloud.h>
#include <Kdtree.h>

Eigen::Vector3d getCenter( const std::vector<Eigen::Vector3d>& p )  {
        Eigen::Vector3d c;
        c.setZero();
        for( size_t i = 0 ; i < p.size() ; ++i ) {
                 c += p[i];
        }
        c *= 1.0/p.size();
        return c;
}

Eigen::Vector3d estimate_normal_at(const Eigen::Vector3d& p, Kdtree<Eigen::Vector3d>& kdtree, const int num ) {
        std::vector<Eigen::Vector3d> result;
        kdtree.find( p, num, result, 0.01 ); // p‚É‹ß‚¢“_NŒÂŽæ“¾
        const Eigen::Vector3d c = getCenter(result);

        Eigen::Matrix3d m;
        m.setZero();

        for( size_t j = 0 ; j < result.size(); ++j ) {
                const Eigen::Vector3d v = result[j]- c;
                m += v * v.transpose();
        }

        Eigen::EigenSolver<Eigen::Matrix3d> solver( m );
        std::vector<std::pair<double, int> > evalue;
        for( int j = 0 ; j < 3 ; ++j ) {
                const double ev = solver.eigenvalues()( j,0 ).real(); //eigen value
                evalue.push_back( std::make_pair( ev * ev , j ) );
        }
        std::sort( evalue.begin(), evalue.end() );
        const int id = evalue[0].second; //Å¬ŒÅ—L’l‚Ì”Ô†
        return solver.eigenvectors().col(id).real().normalized();
}

void estimateNormal( PointCloud& cloud, const int num ) {
        std::cerr<<"Estimating normal vectors ... ";
        std::vector<Eigen::Vector3d> pnts;
        for( int i = 0 ; i < cloud.getNumPoints() ; ++i ) {
                pnts.push_back( cloud.getPoint( i ) );
        }
        Kdtree<Eigen::Vector3d> kdtree( pnts );
	
        for( int i = 0 ; i < cloud.getNumPoints() ; ++i ) {
                const Eigen::Vector3d n = estimate_normal_at( cloud.getPoint( i ), kdtree, num);
                cloud.setNormal( i, n );
        }
        std::cerr<<"done."<<std::endl;
}
#endif
