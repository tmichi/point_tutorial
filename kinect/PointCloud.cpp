#include "PointCloud.h"
#include <fstream>
#include <iostream>


PointCloud::PointCloud ( const int num )
{
        this->points_.reserve( num );
        this->normal_.reserve( num );
        return;
}

PointCloud::~PointCloud( void )
{
        return;
}

int
PointCloud::getNumPoints( void ) const
{
        return static_cast<int>(this->points_.size());
}

Eigen::Vector3d
PointCloud::getPoint( int id ) const
{
        return this->points_.at( id );
}

Eigen::Vector3d
PointCloud::getNormal( int id ) const
{
        return this->normal_.at( id );
}

int
PointCloud::addPoint( const Eigen::Vector3d& p )
{
        const int newId = static_cast<int>(this->points_.size());
        this->points_.push_back( p );
        this->normal_.push_back( Eigen::Vector3d( 1, 0, 0 ) );
        return newId;
}

void
PointCloud::setPoint ( const int id , const Eigen::Vector3d& p )
{
        this->points_.at( id ) = p;
        return;
}

void
PointCloud::setNormal ( const int id , const Eigen::Vector3d& n, const bool normalize )
{
        Eigen::Vector3d nrm = n;
        if ( normalize ) nrm.normalize();
        this->normal_.at( id ) = nrm;
        return;
}


void PointCloud::getBoundingBox( Eigen::Vector3d& bmin, Eigen::Vector3d& bmax )
{
        const int num = this->getNumPoints();
        if ( num < 1 ) return;

        bmin = this->getPoint( 0 );
        bmax = this->getPoint( 0 );

        for( int i = 0 ; i < num; ++i ) {
                Eigen::Vector3d p = this->getPoint( i );
                if ( p.x() < bmin.x() ) bmin.x() = p.x();
                if ( p.y() < bmin.y() ) bmin.y() = p.y();
                if ( p.z() < bmin.z() ) bmin.z() = p.z();
                if ( p.x() > bmax.x() ) bmax.x() = p.x();
                if ( p.y() > bmax.y() ) bmax.y() = p.y();
                if ( p.z() > bmax.z() ) bmax.z() = p.z();
        }
        return;
}
bool
PointCloud::readXyz ( const std::string& filename , bool normal)
{
        std::ifstream fin ( filename.c_str() ) ;
        if ( !fin ) {
                std::cerr<<filename<<" read failed."<<std::endl;
                return false;
        }
        int num;
        fin >> num;
        std::cerr<<"num. of points:"<<num<<std::endl;
        for( int i = 0 ; i < num ; ++i ) {
                if ( fin.bad() ) return false;
                double x, y, z, nx = 0, ny = 0, nz = 1;
                fin >>x>>y>>z;
		int id = this->addPoint( Eigen::Vector3d( x,y,z ) );
                if ( normal ) {
			fin>>nx>>ny>>nz;
		}
		this->setNormal( id, Eigen::Vector3d( nx,ny,nz ), true );
		 
        }
        fin.close();
        return true;
}

bool
PointCloud::writeXyz ( const std::string& filename )
{
        std::ofstream fout ( filename.c_str() ) ;
        if ( !fout ) {
                std::cerr<<filename<<" read failed."<<std::endl;
                return false;
        }
        const int num = this->getNumPoints();
        fout << num <<std::endl;
        for( int i = 0 ; i < num ; ++i ) {
                if ( fout.bad() ) return false;
                Eigen::Vector3d p = this->getPoint( i );
                Eigen::Vector3d n = this->getNormal( i );
                fout<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<n.x()<<" "<<n.y()<<" "<<n.z()<<std::endl;
        }
        fout.close();
        return true;
}

bool
PointCloud::readPnt ( const std::string& filename )
{
        std::ifstream fin ( filename.c_str() ) ;
        if ( !fin ) {
                std::cerr<<filename<<" read failed."<<std::endl;
                return false;
        }
        while ( 1 ) {
                if ( fin.bad() ) return false;
                double x, y, z;
                fin >>x>>y>>z;
                if ( fin.eof() ) break;
                int id = this->addPoint( Eigen::Vector3d( x,y,z ) );
        }
        std::cerr<<this->getNumPoints()<<" points read."<<std::endl;
        fin.close();
        return true;
}

bool
PointCloud::writePnt ( const std::string& filename )
{
        std::ofstream fout ( filename.c_str() ) ;
        if ( !fout ) {
                std::cerr<<filename<<" read failed."<<std::endl;
                return false;
        }
        const int num = this->getNumPoints();
        for( int i = 0 ; i < num ; ++i ) {
                if ( fout.bad() ) return false;
                Eigen::Vector3d p = this->getPoint( i );
                fout<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<std::endl;
        }
        fout.close();
        return true;
}
