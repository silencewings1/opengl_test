
aux_source_directory(gls/ GLS_SRC)
aux_source_directory(io/ IO_SRC)

## lib
add_library(PlyViewer
    ${GLS_SRC}
    ${IO_SRC}
)
