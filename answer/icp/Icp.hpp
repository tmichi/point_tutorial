#ifndef ICP_HPP
#define ICP_HPP 1                                                                                                                                                                                                                          
#include <Eigen/Dense>
#include <Eigen/Eigen>
#include <vector>
#include <cassert>
#include "Kdtree.h"
#include "PointCloud.h"
#include <ctime>
class Icp {
private:
        const PointCloud &_src;
        const PointCloud &_trg;
        int _num_samples;
        Kdtree<Eigen::Vector3d> _kdtree;

        std::vector<Eigen::Vector3d> _sampled_target;
        std::vector<Eigen::Vector3d> _sampled_points;
        PointCloud _result;

        Eigen::Matrix4d _matrix;

        bool _isTransformed;
public:
        Icp ( const PointCloud& src, const PointCloud& trg, const int n = 10000 ) : _src(src), _trg(trg), _num_samples(n) {
                std::vector<int> index, index2;
                this->get_random_ids( this->_src.getNumPoints(), this->_num_samples, index);
                this->get_random_ids( this->_trg.getNumPoints(), this->_num_samples, index2);
              
                for( int i = 0 ; i < n ; ++i ) {
                        this->_sampled_points.push_back( this->_src.getPoint( index[i] ) ) ;
                        this->_sampled_target.push_back( this->_trg.getPoint( index2[i] ) ) ;
               }
                this->_kdtree.build(this->_sampled_target);


                for( int i = 0 ; i < this->_src.getNumPoints() ; ++i ) {
                        this->_result.addPoint(this->_src.getPoint(i));
                }
                this->_matrix.setIdentity();
                this->_isTransformed = true;
                return;
        }

        PointCloud& getCurrentResult ( void ) {
                if ( !this->_isTransformed) {
                        for( int i = 0 ; i < this->_result.getNumPoints() ; ++i )
                        {
                                const Eigen::Vector3d p = this->transform_point( this->_matrix, this->_result.getPoint(i));
                                this->_result.setPoint(i, p ) ;        
                        }
                        this->_matrix.setIdentity();
                        this->_isTransformed = true;
                }
                return this->_result;
        }

        double compute ( void ) {
                const int &n = this->_num_samples;
                std::vector<Eigen::Vector3d> &P = this->_sampled_points;
                std::vector<Eigen::Vector3d> X( n, Eigen::Vector3d());
              

                for( int i = 0 ; i < n ; ++i ) {
                        X[i] = this->_kdtree.closest(P[i], 5);
                }
              
              
                Eigen::Matrix4d m;
                this->compute_transformation(P,X, m);
                
                double error = 0;
                for( int i = 0 ; i < n ; ++i ) {
                        Eigen::Vector3d newp =  this->transform_point(m, P[i]);
                        P[i] = newp;
                        X[i] = this->_kdtree.closest(P[i], 5);
                        error += ( X[i] - P[i] ).norm();
                }          


                this->_matrix = m * this->_matrix;
                this->_isTransformed = false;
                return error;
        }

        void compute_transformation ( std::vector<Eigen::Vector3d>& P, std::vector<Eigen::Vector3d>& Q, Eigen::Matrix4d &R) {
                const int N = static_cast<int>(P.size());
                const Eigen::Vector3d cp = get_center_point(P); // center of source
                const Eigen::Vector3d cq = get_center_point(Q); // center of target  


                Eigen::Matrix3d M; // 1/N * É∞ ( pi - xi^t )  - cp * cx^t
                M.setZero();
                for( int  i = 0 ; i < N ; ++i ) {
                        M += P[i] * Q[i].transpose();
                }
                M *= 1.0 / N;
                M -= cp * cq.transpose();

                const double trace = M.trace();
                Eigen::Matrix3d A = M - M.transpose();
                Eigen::Matrix3d I;
                I.setIdentity();
                Eigen::Matrix3d S = M + M.transpose()  - trace * I;

                // Q = (trace  A(1,2) A(2,0) A(0, 1)
                //     (A(1,2) 
                //     (A(2,0)  S = M + M^t - trace * I
                //     (A(0,1)
                Eigen::Matrix4d X;
                X.setZero();
                X(0, 0) = trace;

                X(1, 0) = A(1,2);
                X(2, 0) = A(2,0);
                X(3, 0) = A(0,1);

                X(0, 1) = A(1,2);
                X(0, 2) = A(2,0);
                X(0, 3) = A(0,1);

                for( int i = 0 ; i < 3 ; ++i) {
                        for( int j = 0 ; j < 3 ; ++j) {
                                X(i+1, j+1) = S(i, j); 
                        }
                }

                Eigen::EigenSolver<Eigen::Matrix4d> solver(X);

                //ê‚ëŒílÇ™ç≈ëÂÇ∆Ç»ÇÈå≈óLílÇíTÇ∑
                std::vector<std::pair<double, int> > evalue;
                for( int i = 0 ; i < 4 ; ++i) {
                        double ev = solver.eigenvalues()(i,0).real();
                        evalue.push_back(std::make_pair(ev * ev , i));
                }
                std::sort( evalue.begin(), evalue.end());
                const int maxId = evalue.at(3).second;

                // the order of eigenvectors are (x, y, z, w)
                Eigen::Quaterniond q;
                Eigen::Vector4d evec = solver.eigenvectors().col(maxId).real();

                q.w() = evec.x();
                q.x() = evec.y();
                q.y() = evec.z();
                q.z() = evec.w();
                Eigen::Matrix3d r = q.toRotationMatrix();
                Eigen::Vector3d t = cq - r * cp;

                R.setIdentity();
                for( int i = 0 ; i < 3 ; ++i) {
                        for( int j = 0 ; j < 3 ; ++j) {
                                R(i, j) = r(i, j); 
                        }
                        R(i, 3) = t[i];
                }
                return;
        }

        Eigen::Vector3d get_center_point( const std::vector<Eigen::Vector3d>& p ) {
                Eigen::Vector3d result;
                result.setZero();
                if ( p.size() > 0 ) {
                        for( size_t i = 0 ; i < p.size() ; i++ ) {
                                result += p[i];
                        }
                        result *= 1.0 / p.size();
                }
                return result;
        }

        Eigen::Vector3d transform_point (const Eigen::Matrix4d& m,  const Eigen::Vector3d& p ) {
                Eigen::Vector4d n0(p.x(), p.y(), p.z(), 1);
                n0 =  m *  n0;
                return  Eigen::Vector3d(n0.x(), n0.y(), n0.z());
        }

        void get_random_ids( const int maxId, const int n, std::vector<int>& index ) {
                for( int i = 0 ; i < maxId ; ++i ) {
                        index.push_back(i);
                }
                std::random_shuffle(index.begin(), index.end());
                index.resize( n );
        }
};
#endif ///ICP_H