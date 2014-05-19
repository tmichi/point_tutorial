if(WIN32)
INCLUDE_DIRECTORIES("C:/Program Files/Microsoft SDKs/Kinect/v1.8/inc")
LINK_DIRECTORIES("C:/Program Files/Microsoft SDKs/Kinect/v1.8/lib/x86")
ADD_EXECUTABLE(kinect KinectWrapper.h Camera.cpp Camera.h Kdtree.h PointCloud.cpp PointCloud.h Window.cpp Window.h main.cpp)
endif()

