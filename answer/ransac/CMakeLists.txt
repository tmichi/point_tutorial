ADD_EXECUTABLE(ransac Ransac.h main.cpp)
if(APPLE)
TARGET_LINK_LIBRARIES(ransac glfw3 "-framework OpenGL")
endif()

