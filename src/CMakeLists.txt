
include_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib/model_generator)
link_directories(${CMAKE_BINARY_DIR}/lib/gl_window)

## main
add_executable(test_ply test_ply.cpp)

target_link_libraries(test_ply
    libGLWindow.a
    libModelGenerator.a
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
)

## test_disparity
add_executable(test_disparity test_disparity.cpp)

target_link_libraries(test_disparity 
    libGLWindow.a
    libModelGenerator.a
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
    ${OpenCV_LIBS}
)