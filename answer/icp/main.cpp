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
#include <Eigen/Geometry>

#include <Camera.h>
#include <Window.h>
#include <PointCloud.h>
#include <Kdtree.h>
#include <Light.h>

#include "Icp.hpp"

int main( int argc, char** argv )
{
        if ( argc < 3) {
                std::cerr<<"Usage : "<<argv[0]<<" source.xyz target.xyz"<<std::endl;
                return -1;
        }
        std::cerr<<"press a for icp iteration"<<std::endl;
        //データ読み込み
        PointCloud source;
        const std::string filename( argv[1] );
        if ( !source.readXyz( filename ) ) {
                std::cerr<<filename<<" read failed."<<std::endl;
                return -1;
        }
        
        PointCloud target;
        const std::string filename2( argv[2] );
        if ( !target.readXyz( filename2 ) ) {
                std::cerr<<filename2<<" read failed."<<std::endl;
                return -1;
        }
    
        Eigen::Vector3d bmin, bmax;
        target.getBoundingBox( bmin, bmax );
       Icp icp(source, target, 1000);

        std::cerr<<"icp initialized."<<std::endl;

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


                
                glPointSize(1);
                PointCloud& result = icp.getCurrentResult();
                glBegin( GL_POINTS );
                for( int i = 0 ; i < result.getNumPoints(); ++i ) {     
                        glColor3f(1,1,0);
                        const Eigen::Vector3d p = result.getPoint( i );
                        glVertex3d(p.x(), p.y(), p.z() );
                }
                glEnd();
                
                glPointSize(1);
                glBegin(GL_POINTS);
                for( int i = 0 ; i < target.getNumPoints(); ++i ) {     
                        glColor3f(1,1,1);
                        const Eigen::Vector3d p = target.getPoint( i );
                        glVertex3d(p.x(), p.y(), p.z() );
                }
                glEnd();
                
                glfwSwapBuffers( window );

                //マウスイベントの取得
                glfwPollEvents();
                double oldx, oldy, newx, newy;
                win.getMousePosition( oldx, oldy, newx, newy );
                if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_1 ) != GLFW_RELEASE ) {
                        camera.rotate( oldx, oldy, newx, newy );
                }

                if ( ::glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) {
                       double error = icp.compute();
                        std::cerr<<"error : "<<error<<std::endl;
                }
        }
        return 0;
}
