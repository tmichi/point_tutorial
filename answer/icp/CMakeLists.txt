ADD_EXECUTABLE(icp Icp.hpp main.cpp)

if(APPLE)
TARGET_LINK_LIBRARIES(icp glfw3 "-framework OpenGL")
endif()

