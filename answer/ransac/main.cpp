#include <vector>
#include <algorithm>
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
#include <Polygon3d.h>
#include "Ransac.h"

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

        //Extracting plane by RANSAC
        std::vector<Polygon3d> polygons;
        Ransac ransac(cloud, 100000);
        std::cerr<<"initialized"<<std::endl;
        Eigen::Vector3d n0, p0;
        int count = 0;
        while ( ransac.compute(n0, p0, 0.005, 5000, 1000) ) {
               Polygon3d poly;
                ransac.getPolygon(count, poly);      
                polygons.push_back(poly);              
                ++count;
        }
        std::cerr<<count<<" planes extracted."<<std::endl;
      
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
        Light light(GL_LIGHT0);

        int togglePoint = 1;
        std::cerr<<"push 'p' button for rendering polygons"<<std::endl;
        while ( !glfwWindowShouldClose( window ) ) {

                glMatrixMode( GL_PROJECTION );
                glLoadIdentity();
                double fov, zNear, zFar;
                camera.getPerspective( fov, zNear, zFar );
                gluPerspective( fov, win.getAspectRatio(),  zNear, zFar );

                glMatrixMode( GL_MODELVIEW );
                glLoadIdentity();
                Eigen::Vector3d eye, center, up;
                camera.getLookAt( eye, center, up );
                gluLookAt( eye.x(), eye.y(), eye.z(), center.x(), center.y(), center.z(), up.x(), up.y(), up.z() );
                Eigen::Vector3d ray = center-eye;

                //オブジェクトの描画
                glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

                glEnable( GL_LIGHTING );

                if ( togglePoint) {
                        glBegin( GL_POINTS );
                        for( int i = 0 ; i < cloud.getNumPoints(); ++i ) {
                                Eigen::Vector3d n = cloud.getNormal(i );
                                if ( ray.dot(n) > 0 ) n *= -1;
                                glNormal3d(n.x(), n.y(), n.z());


                                const Eigen::Vector3d p = cloud.getPoint( i );
                                glVertex3d(p.x(), p.y(), p.z() );
                        }
                        glEnd();
                }
                else {      
                        for( int i = 0 ; i < static_cast<int>( polygons.size() ) ; ++i ) {                   
                                glBegin ( GL_POLYGON); 

                                Eigen::Vector3d n = polygons[i].getNormal();
                                if ( ray.dot(n) > 0 ) n *= -1;
                                glNormal3d(n.x(), n.y(), n.z());

                                for( int j = 0 ; j < polygons[i].getNumPoints() ; ++j ) {
                                        Eigen::Vector3d p = polygons[i].getPoint(j);
                                        glVertex3d(p.x(), p.y(), p.z() );
                                }
                                glEnd();
                        }
                }
                glfwSwapBuffers( window );


                //マウスイベントの取得
                glfwWaitEvents();
                double oldx, oldy, newx, newy;
                win.getMousePosition( oldx, oldy, newx, newy );
                if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_1 ) != GLFW_RELEASE ) {
                        camera.rotate( oldx, oldy, newx, newy );
                }
                if ( ::glfwGetKey(window, GLFW_KEY_P) ){
                        togglePoint = 1 - togglePoint;
                        std::cerr<<"point: "<<togglePoint<<std::endl;
                }
        }
        return 0;
}