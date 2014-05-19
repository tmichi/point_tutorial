#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <iostream>
#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H 1

class PointCloud
{
private:
        PointCloud ( const PointCloud& that ) ;
        void operator = ( const PointCloud& that );
public:
        explicit PointCloud ( const int num = 0 );
        virtual ~PointCloud( void );
        int getNumPoints( void ) const;
        Eigen::Vector3d getPoint( int id ) const;
        Eigen::Vector3d getNormal( int id ) const;
        int addPoint( const Eigen::Vector3d& p );
        void setPoint ( const int id , const Eigen::Vector3d& p );
        void setNormal ( const int id , const Eigen::Vector3d& n, const bool normal = false );
        void getBoundingBox( Eigen::Vector3d& bmin, Eigen::Vector3d& bmax );

        bool readXyz ( const std::string& filename, bool normal = false );
        bool writeXyz ( const std::string& filename );
        bool readPnt ( const std::string& filename );
        bool writePnt ( const std::string& filename );
private:
        std::vector<Eigen::Vector3d> points_;
        std::vector<Eigen::Vector3d> normal_;
};
#endif// POINT_CLOUD
