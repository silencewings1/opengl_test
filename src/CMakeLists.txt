
include_directories(${CMAKE_SOURCE_DIR}/lib)
link_directories(${CMAKE_BINARY_DIR}/lib/model_generator)
link_directories(${CMAKE_BINARY_DIR}/lib/gl_window)

add_subdirectory(global_sfm)

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

## test_global_sfm
add_executable(test_global_sfm 
    test_global_sfm.cpp
    global_sfm/global_sfm.cpp)

target_link_libraries(test_global_sfm 
    libGLWindow.a
    libModelGenerator.a
    ${OPENGL_LIBRARIES} 
    ${GLUT_LIBRARY} 
    ${OpenCV_LIBS}

    OpenMVG::openMVG_camera
    OpenMVG::openMVG_exif
    OpenMVG::openMVG_features
    OpenMVG::openMVG_geodesy
    OpenMVG::openMVG_geometry
    OpenMVG::openMVG_graph
    OpenMVG::openMVG_image
    OpenMVG::openMVG_linearProgramming
    OpenMVG::openMVG_matching
    OpenMVG::openMVG_matching_image_collection
    OpenMVG::openMVG_multiview
    OpenMVG::openMVG_numeric
    OpenMVG::openMVG_robust_estimation
    OpenMVG::openMVG_sfm
    OpenMVG::openMVG_system
    OpenMVG::vlsift
)