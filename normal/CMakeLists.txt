ADD_EXECUTABLE(normal normal.h main.cpp)
if(APPLE)
TARGET_LINK_LIBRARIES(normal glfw3 "-framework OpenGL")
endif()

