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

#include "Camera.h"
#include "Window.h"
#include "PointCloud.h"
#include "Kdtree.h"
#include "KinectWrapper.h"
#include "Light.h"

void initGL( void );
int main( int argc, char** argv )
{
        //�f�[�^�ǂݍ���
        PointCloud cloud;

	int npoints = 640 * 480;
	for( int i = 0 ; i < npoints ; ++i ) {
		cloud.addPoint( Eigen::Vector3d(0,0,0));
	}

       
        Camera camera;
        KinectWrapper wrapper;
	if ( !wrapper.init() ) {
		return false;
	}
        if ( glfwInit() == GL_FALSE ) return -1;
	Window win( 640, 480, "hello world" );
        if ( !win ) return -1;
        GLFWwindow* window = win.getWindow();

        glEnable( GL_DEPTH_TEST );
        glClearColor( 0,0,1,1 );
        glPointSize( 2.0 );
        Light light0(GL_LIGHT0);

        while ( !glfwWindowShouldClose( window ) ) {
		std::vector<Eigen::Vector3d> points;
		wrapper.getPointSet(points);
		for( int i = 0 ; i < npoints ; ++i ) {
			cloud.setPoint(i, points[i]);
		}
		Eigen::Vector3d bmin, bmax;
	        cloud.getBoundingBox( bmin, bmax );
		camera.init( bmin, bmax );

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

                //�I�u�W�F�N�g�̕`��
                glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
                glBegin( GL_POINTS );
        
                for( int i = 0 ; i < cloud.getNumPoints(); ++i ) {
                        Eigen::Vector3d p = cloud.getPoint( i );
			glColor3f( 1,1, 1);
                        glVertex3d( p.x(), p.y(), p.z() );
                }
                glEnd();
                glfwSwapBuffers( window );
		
                //�}�E�X�C�x���g�̎擾
		glfwPollEvents();
                double oldx, oldy, newx, newy;
                win.getMousePosition( oldx, oldy, newx, newy );
                if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_1 ) != GLFW_RELEASE ) {
                        camera.rotate( oldx, oldy, newx, newy );
                }

                if ( glfwGetKey ( window, GLFW_KEY_SPACE) == GLFW_PRESS ) {
                        std::cerr<<"point set is saving to result.xyz ...";
                        std::stringstream ss;
                        ss<<__DATE__<<".xyz";
                        cloud.writeXyz(ss.str().c_str());
                        std::cerr<<"done"<<std::endl;
                }
        }
        return 0;
}
