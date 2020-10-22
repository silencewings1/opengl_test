
add_executable(global_sfm 
    main.cpp
    global_sfm.cpp)

target_link_libraries(global_sfm
PRIVATE
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
  # OpenMVG::openMVG_kvld
  OpenMVG::openMVG_numeric
  OpenMVG::openMVG_robust_estimation
  OpenMVG::openMVG_sfm
  OpenMVG::openMVG_system
  OpenMVG::vlsift

  ${OpenCV_LIBS}
)