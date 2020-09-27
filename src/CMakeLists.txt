
include_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib/model_generator)
link_directories(${CMAKE_BINARY_DIR}/lib/gl_window)

## main
add_executable(test_display main.cpp)

target_link_libraries(test_display 
    libGLWindow.a
    libModelGenerator.a
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
)