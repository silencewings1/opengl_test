
include_directories(${CMAKE_SOURCE_DIR}/lib)
aux_source_directory(ply/ PLY_SRC)
aux_source_directory(disparity/ DISP_SRC)

add_library(ModelGenerator
    ${PLY_SRC}
    ${DISP_SRC}
)
