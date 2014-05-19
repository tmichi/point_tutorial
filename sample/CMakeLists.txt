ADD_EXECUTABLE(sample Camera.cpp Camera.h Kdtree.h PointCloud.cpp PointCloud.h Window.cpp Window.h main.cpp)

if(APPLE)
TARGET_LINK_LIBRARIES(sample glfw3 "-framework OpenGL")
endif()

