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
void initGL( void );
int main( int argc, char** argv )
{
	if ( argc < 2 ) {
		std::cerr<<"Usage : "<<argv[0]<<" input.pnt"<<std::endl;
		return -1;
	}
	//データ読み込み
	PointCloud cloud;
	const std::string filename( argv[1] );
	if ( !cloud.readXyz( filename ) ) {
		std::cerr<<filename<<" read failed."<<std::endl;
		return -1;
	}


	std::vector<int> isNear( cloud.getNumPoints(), 0 ) ;
	std::vector< IndexedVector<Eigen::Vector3d> > points;
	for( int i = 0 ; i < cloud.getNumPoints() ; ++i) {
		IndexedVector<Eigen::Vector3d> p( cloud.getPoint(i), i);
		points.push_back( p );
	}
	Kdtree< IndexedVector<Eigen::Vector3d> > kdtree(points);
	std::list<IndexedVector<Eigen::Vector3d> > result;
	const int num = 100;

	Eigen::Vector3d v = cloud.getPoint(2050);
	IndexedVector<Eigen::Vector3d> trg(v, 0);

	kdtree.find(trg, num, result, 0.001);

	std::list<IndexedVector<Eigen::Vector3d> >::iterator iter = result.begin() ; // 最も近い点
	for ( int i = 0 ; i < num ; i++ ) {
		isNear[iter->id()] = true;
		++iter;
	}



	Eigen::Vector3d bmin, bmax;
	cloud.getBoundingBox( bmin, bmax );

	Camera camera;
	camera.init( bmin, bmax );


	if ( glfwInit() == GL_FALSE ) return -1;
	Window win( 640, 480, "hello world" );
	if ( !win ) return -1;
	GLFWwindow* window = win.getWindow();

	initGL();
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

		//オブジェクトの描画
		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
		glBegin( GL_POINTS );

		for( int i = 0 ; i < cloud.getNumPoints(); ++i ) {
			Eigen::Vector3d p = cloud.getPoint( i );
			if( isNear[i] ) glColor3f( 1,0, 0);
			else            glColor3f( 1,1, 1);
			glVertex3d( p.x(), p.y(), p.z() );
		}
		glEnd();

		glfwSwapBuffers( window );


		//マウスイベントの取得
		glfwWaitEvents();
		double oldx, oldy, newx, newy;
		win.getMousePosition( oldx, oldy, newx, newy );
		if ( glfwGetMouseButton( window, GLFW_MOUSE_BUTTON_1 ) != GLFW_RELEASE ) {
			camera.rotate( oldx, oldy, newx, newy );
		}
	}
	return 0;
}

void initGL( void )
{
	glEnable( GL_DEPTH_TEST );
	glClearColor( 0,0,1,1 );
	glPointSize( 2.0 );

	// ライト
	/*
	static float light0_ambient[] =  {0.1f, 0.1f, 0.1f, 1.0f};
	static float light0_diffuse[] =  {1.0f, 1.0f, 1.0f, 0.0f};
	static float light0_position[] = {0.0f, 0.0f,1000.0f, 0.0f};
	static float light0_specular[] = {0.4f, 0.4f, 0.4f, 1.0f};

	glLightfv( GL_LIGHT0, GL_AMBIENT, light0_ambient );
	glLightfv( GL_LIGHT0, GL_DIFFUSE, light0_diffuse );
	glLightfv( GL_LIGHT0, GL_SPECULAR, light0_specular );
	glLightfv( GL_LIGHT0, GL_POSITION, light0_position );
	glLightModelf( GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE );

	glEnable( GL_LIGHT0 );
	glEnable( GL_LIGHTING );
	*/
}
