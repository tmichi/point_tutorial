#include <vector>
#include <iostream>

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#include <windows.h>
#include <gl/GLU.h>
#pragma comment(lib, "glfw3dll.lib")
#pragma comment(lib, "opengl32.lib")
#pragma comment(lib, "glu32.lib")
#elif defined(__APPLE__)
#include <OpenGL/glu.h>
#else
#include <GL/glu.h>
#endif

#include <GLFW/glfw3.h>
#include <Eigen/Dense>

#include <Camera.h>
#include <Window.h>
#include <PointCloud.h>
#include <Kdtree.h>
#include <Light.h>

Eigen::Vector3d getCenter( const std::vector<Eigen::Vector3d>& p )  {
        Eigen::Vector3d c;
        c.setZero();
        //: TODO add code here
        return c;
}

Eigen::Vector3d estimate_normal_at(const Eigen::Vector3d& p, Kdtree<Eigen::Vector3d>& kdtree, const int num ) {
        return Eigen::Vector3d::Random().normalized(); //ランダムな法線を返す
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

int main( int argc, char** argv )
{
        if ( argc < 2 ) {
                std::cerr<<"Usage : "<<argv[0]<<" input.xyz"<<std::endl;
                return -1;
        }
        std::cerr<<"Press 'n' to enable/disable lighting."<<std::endl;
        std::cerr<<"Press 'v' to visualize normal vector."<<std::endl;

        //データ読み込み
        PointCloud cloud;
        const std::string filename( argv[1] );
        if ( !cloud.readXyz( filename ) ) {
                std::cerr<<filename<<" read failed."<<std::endl;
                return -1;
        }
        //TODO この関数の中身を作る
        estimateNormal(cloud, 30);

        Eigen::Vector3d bmin, bmax;
        cloud.getBoundingBox( bmin, bmax );

        Camera camera;
        camera.init( bmin, bmax );

        if ( glfwInit() == GL_FALSE ) return -1;
        Window win( 640, 480, "hello world" );
        if ( !win ) return -1;
        GLFWwindow* window = win.getWindow();

        glEnable( GL_DEPTH_TEST );
        glClearColor( 0,0,1,1 );
        glPointSize( 2.0 );
        Light light0(GL_LIGHT0);

        int toggleLighting = 1;
        int toggleNormalVisualize = 0;

        while ( !glfwWindowShouldClose( window ) ) {
                double fov, zNear, zFar;
                Eigen::Vector3d eye, center, up;
                camera.getPerspective( fov, zNear, zFar );
                camera.getLookAt( eye, center, up );
                
                //カメラ、投影情報の設定
                glMatrixMode( GL_PROJECTION );
                glLoadIdentity();
                gluPerspective( fov, win.getAspectRatio(),  zNear, zFar );

                glMatrixMode( GL_MODELVIEW );
                glLoadIdentity();
                gluLookAt( eye.x(), eye.y(), eye.z(), center.x(), center.y(), center.z(), up.x(), up.y(), up.z() );

                Eigen::Vector3d ray = center-eye;

                //オブジェクトの描画
                glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

                if ( toggleLighting ) {
                        glEnable( GL_LIGHTING );
                }
                else {
                        glDisable( GL_LIGHTING );
                }

                glBegin( GL_POINTS );
                for( int i = 0 ; i < cloud.getNumPoints(); ++i ) {
                        if ( toggleLighting ) {
                                Eigen::Vector3d n = cloud.getNormal(i );
                                if ( ray.dot(n) > 0 ) n *= -1;
                                glNormal3d(n.x(), n.y(), n.z());
                        }	
                        else {
                                glColor3f(1,1,1);
                        }

                        const Eigen::Vector3d p = cloud.getPoint( i );
                        glVertex3d(p.x(), p.y(), p.z() );
                }
                glEnd();

                // visualization of normal vector.
                if ( toggleNormalVisualize ) {
                        glDisable( GL_LIGHTING );
                        glBegin(GL_LINES);
                        glColor3f (1,0,0);
                        for( int i = 0 ; i < cloud.getNumPoints(); i += 100 ) {
                                Eigen::Vector3d p = cloud.getPoint(i);
                                Eigen::Vector3d n = cloud.getNormal(i);
                                if ( ray.dot(n) > 0 ) n *= -1;
                                n.normalize();
                                n *= 5;
                                glVertex3d( p.x(), p.y(), p.z() );
                                glVertex3d( p.x() + n.x(), p.y() + n.y(), p.z() + n.z());
                        }
                        glEnd();
                }

                glfwSwapBuffers( window );

                //マウスイベントの取得
                glfwWaitEvents();
                double oldx, oldy, newx, newy;
                win.getMousePosition( oldx, oldy, newx, newy );
                if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_1 ) != GLFW_RELEASE ) {
                        camera.rotate( oldx, oldy, newx, newy );
                }
                if ( ::glfwGetKey(window, GLFW_KEY_N ) == GLFW_PRESS ) {
                        toggleLighting = 1 - toggleLighting;
                        std::cerr<<"lighting : "<<toggleLighting<<std::endl;
                }
                if ( ::glfwGetKey(window, GLFW_KEY_V) == GLFW_PRESS) {
                        toggleNormalVisualize = 1 - toggleNormalVisualize;
                        std::cerr<<"viisualize : "<<toggleNormalVisualize<<std::endl;
                }
        }
        return 0;
}
