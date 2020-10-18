cmake_minimum_required(VERSION 3.0)
project(OPENGL_PLY_TEST)

set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

## GLUT
find_package(GLUT REQUIRED)
include_directories(${GLUT_INCLUDE_DIRS})
link_directories(${GLUT_LIBRARY_DIRS})
add_definitions(${GLUT_DEFINITIONS})

## OPENGL
find_package(OpenGL REQUIRED)
include_directories(${OpenGL_INCLUDE_DIRS})
link_directories(${OpenGL_LIBRARY_DIRS})
add_definitions(${OpenGL_DEFINITIONS})

## OPENCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

## EIGEN
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

## OPENMVG
find_package(OpenMVG REQUIRED)
include_directories(${OPENMVG_INCLUDE_DIRS})
message(STATUS "OpenMVG: " ${OPENMVG_INCLUDE_DIRS})
include_directories("/usr/local/include/openMVG_dependencies/")

## sub
add_subdirectory(lib)
add_subdirectory(src)