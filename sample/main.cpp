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

#include <Camera.h>
#include <Window.h>
#include <PointCloud.h>
#include <Kdtree.h>

void findNearPoints ( const Eigen::Vector3d& p, const int num, PointCloud& cloud, std::vector<bool>& isNear ) {
        isNear.assign( cloud.getNumPoints(), false );
  
        std::vector< IndexedVector<Eigen::Vector3d> > points; //IDをつけた点群データ
        for( int i = 0 ; i < cloud.getNumPoints() ; ++i) {
                IndexedVector<Eigen::Vector3d> p( cloud.getPoint(i), i);
                points.push_back( p );
        }
        Kdtree< IndexedVector<Eigen::Vector3d> > kdtree(points);
        std::vector<IndexedVector<Eigen::Vector3d> > result;
        kdtree.find( IndexedVector<Eigen::Vector3d>(p), num, result, 0.001); // 
        for ( int i = 0 ; i < num ; i++ ) {
                isNear[ result[i].id()] = true;
        }
        return;
}

int main( int argc, char** argv )
{
        if ( argc < 2 ) {
                std::cerr<<"Usage : "<<argv[0]<<" input.xyz"<<std::endl;
                return -1;
        }
        //データ読み込み
        PointCloud cloud;
        const std::string filename( argv[1] );
        if ( !cloud.readXyz( filename ) ) {
                std::cerr<<filename<<" read failed."<<std::endl;
                return -1;
        }
        std::vector<bool> isNear ;

        findNearPoints( cloud.getPoint(2050), 100, cloud, isNear);

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

                //オブジェクトの描画
                glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
                glBegin( GL_POINTS );
                for( int i = 0 ; i < cloud.getNumPoints(); ++i ) {
                        if( isNear[i] ) glColor3f( 1, 0, 0);
                        else            glColor3f( 1, 1, 1);
                        Eigen::Vector3d p = cloud.getPoint( i );
                        glVertex3d(p.x(), p.y(), p.z() );
                }
                glEnd();

                glfwSwapBuffers( window );

                //マウスイベントの取得
                glfwWaitEvents();
                double oldx, oldy, newx, newy;
                win.getMousePosition( oldx, oldy, newx, newy );
                //右ドラッグで回転
                if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_1 ) != GLFW_RELEASE ) {
                        camera.rotate( oldx, oldy, newx, newy );
                }
        }
        return 0;
}


