#ifndef RANSAC_H
#define RANSAC_H 1
#include <PointCloud.h>
#include <Polygon3d.h>


class Ransac {
private:
        struct Point2d {
                double x, y;

                bool operator <(const Point2d &p) const {
                        return x < p.x || (x == p.x && y < p.y);
                }
        };

private:
        PointCloud& _cloud;
        std::vector<int> _label;

        std::vector<Eigen::Vector3d> _normals;
        std::vector<Eigen::Vector3d> _points;
        int _maxLabel;
        int _num_samples;
private:
        Ransac( const Ransac &that) ;
        void operator = ( const Ransac& that);
public:
        explicit Ransac ( PointCloud& cloud , const int num_samples = 10000 ) : _cloud (cloud){
                this->_label.assign( this->_cloud.getNumPoints(), -1);
                this->_num_samples = num_samples;
                this->_maxLabel = 0;
        }

        ~Ransac ( void ) {
                return;
        }

        int compute ( Eigen::Vector3d& n, Eigen::Vector3d& p, const double epsilon = 0.01, int min_score = 1000, const int num_test = 10000) {
                n.setZero();
                p.setZero();
                double maxScore = 0;

                Eigen::Vector3d bmin, bmax;
                this->_cloud.getBoundingBox(bmin, bmax) ;
                
                const double error0 = epsilon * ( bmax - bmin).norm(); //誤差　バウンディングボックスの対角線の長さの比
             
                std::vector<Eigen::Vector3d> points;
                std::vector<int> index;
                for( int i = 0 ; i < static_cast<int>(this->_cloud.getNumPoints()) ; ++i ) {
                        index.push_back(i);
                }
                std::random_shuffle(index.begin(), index.end());


                for( int i = 0 ; i < static_cast<int> ( this->_cloud.getNumPoints()) ; ++i ) { 
                        if ( points.size () >= this->_num_samples ) break;
                        if ( this->_label[ index[i] ] > -1 ) {
                                if ( i == this->_cloud.getNumPoints() - 1) return 0;
                        }
                        else points.push_back( this->_cloud.getPoint( index[i] ) );

                }
               
                 int iteration = 0 ; 
                 while ( iteration++ < 5 ) {
                         for( int i = 0 ; i < num_test ; ++i ) {
                                 const Eigen::Vector3d p0 = points[ rand() % points.size() ];
                                 const Eigen::Vector3d p1 = points[ rand() % points.size() ];
                                 const Eigen::Vector3d p2 = points[ rand() % points.size() ];
                                 const Eigen::Vector3d n0 =  (p1-p0).cross(p2-p0).normalized();

                                 const double score = this->get_ransac_score( points, n0, p0 , error0 ) ; 
                                 if ( score > maxScore ) {
                                         n = n0;
                                         p = p0;
                                         maxScore = score;
                                 }
                         }
                         
                         std::vector<Eigen::Vector3d> next;
                         for( int i = 0 ; i < points.size() ; ++i ) {
                                 if ( std::fabs( n.dot( points[i] - p ) ) < error0 ) {
                                         next.push_back(points[i]);
                                 }
                         }
                         if (maxScore < min_score) return 0;
                         
                         points.clear();
                         points.insert(points.end(), next.begin(), next.end());
             
                 }

                std::vector<Eigen::Vector3d> result;
                std::vector<int> labelList;
                for( int i = 0 ; i < this->_cloud.getNumPoints() ; ++i ) {
                        if ( this->_label[i] > -1) continue;
                        Eigen::Vector3d x = this->_cloud.getPoint(i);
                        if ( std::fabs( n.dot( x - p ) ) < error0 ) {
                                result.push_back(this->_cloud.getPoint(i));
                                labelList.push_back(i);
                        }
                }
              
                if ( min_score <= static_cast<int>( result.size() ) )  {
                        for( int i = 0 ; i < static_cast<int>(labelList.size()) ; ++i ) {
                                 this->_label[ labelList[i] ]  = this->_maxLabel;
                        }
                        this->_normals.push_back(n);
                        this->_points.push_back(p);
                        this->_maxLabel += 1;
                        return 1;
                }
                else return 0;
             
        }

        void getPolygon ( const int labelId, Polygon3d& poly) {
                if (labelId >= this->_maxLabel) return;
                std::vector<Eigen::Vector3d> point3d;
                for(int i = 0 ; i < this->_cloud.getNumPoints() ; ++i ) {
                        if ( this->_label.at(i)  == labelId ) point3d.push_back( this->_cloud.getPoint(i));
                }
                Eigen::Vector3d n;
                Eigen::Vector3d p;
                this->get_plane(labelId, n, p);

                Eigen::Vector3d b0 = Eigen::Vector3d::Random().normalized();
                Eigen::Vector3d b1 = n.cross(b0).normalized();
                Eigen::Vector3d b2 = b1.cross(n).normalized();

                const int N = static_cast<int>( point3d.size());

                std::vector<Point2d> point2d;
                for( int i = 0 ; i < N ; ++i ) {
                        Point2d p2;
                        p2.x =  b1.dot(point3d[i] - p);
                        p2.y =  b2.dot(point3d[i] - p);
                        point2d.push_back(p2);
                }
                std::vector<Point2d> results;
                this->compute_convex_hull_2d(point2d, results);

                poly.setNormal(n);
                for( int i = 0 ; i < static_cast<int>(results.size()) ; ++i ) {
                        const double s = results[i].x;
                        const double t = results[i].y;
                        poly.setPoint(p + s * b1 + t * b2);
                }
                return;   
        }
private:
        double get_ransac_score( std::vector<Eigen::Vector3d>& points, const Eigen::Vector3d& n, const Eigen::Vector3d& p, const double error) {
                double score = 0 ;
                for( int i = 0 ; i < static_cast<int>(points.size()) ; ++i ) {
                        if ( this->_label[i] > -1) continue;
                        if ( std::fabs( n.dot( points[i] - p ) ) < error ) score += 1;
                }
                return score;
        }


        void get_plane ( const int id, Eigen::Vector3d& n, Eigen::Vector3d& p ) {
                n = this->_normals.at(id);
                p = this->_points.at(id);
                return;
        }

        double cross ( const Point2d& o, const Point2d& a, const Point2d& b ) {
                return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
        }

        //reference : http://en.wikibooks.org/wiki/Algorithm_Implementation/Geometry/Convex_hull/Monotone_chain
        void compute_convex_hull_2d( std::vector<Point2d>& points, std::vector<Point2d>& results ) {
                const int N = static_cast<int>(points.size());
                std::sort( points.begin(), points.end());
                results.assign( 2 * N, Point2d() );

                int k = 0;
                // Build lower hull
                for (int i = 0; i < N ; ++i) {              
                        while ( k >= 2 && this->cross ( results[k-2], results[k-1], points[i] ) <= 0 ) k--;
                        results[k] = points[i];
                        ++k;
                }

                // Build upper hull
                for (int i = N-2, t = k+1; i >= 0; i--) {
                        while (k >= t && this->cross(results[k-2], results[k-1], points[i]) <= 0) k--;
                        results[k] = points[i];
                        ++k;
                }
                results.resize(k);
                return;
        }
};


#endif // RANSAC_H