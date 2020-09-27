
include_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib/ply_viewer)

## main
add_executable(test_ply main.cpp)

target_link_libraries(test_ply 
    libPlyViewer.a
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
)