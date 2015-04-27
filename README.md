point_tutorial
==============

学生向けにつくった点群処理入門プログラム(C++)

必要なライブラリ
------------------
- OpenGL/GLU
- GLFW (http://www.glfw.org)
- Eigen ( http://eigen.tuxfamily.org ) 
- Windows Kinect SDK


コンパイルにはCMake (http://www.cmake.org)を使っていますが，なくても自力でライブラリのリンクをつけられたらなんとかなります.

Programs
-------------
- sample : a sample program for reading and rendering point clouds. type sample data/point.xyz to show example.
- kinect : a sample program for scanning scene by Microsoft Kinect. type [SPACE] key to export point set as "result.xyz"
- tips : 
- normal : 法線の向きを推定するプログラム
- ransac : 簡単なRANSACプログラム
- icp    : ICPプログラム

Contact
-------
Takashi Michikawa ( michikawa at acm dot org) 
