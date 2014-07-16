ADD_EXECUTABLE(icp Icp.hpp main.cpp)

if(APPLE)
TARGET_LINK_LIBRARIES(normal glfw3 "-framework OpenGL")
endif()

