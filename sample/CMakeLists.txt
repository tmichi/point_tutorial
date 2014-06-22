ADD_EXECUTABLE(sample main.cpp)

if(APPLE)
TARGET_LINK_LIBRARIES(sample glfw3 "-framework OpenGL")
endif()

