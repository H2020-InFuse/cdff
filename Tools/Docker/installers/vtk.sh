#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

function install4infuse_vtk {
if [[ ! -n $(find $PKG_DIR -name 'vtk*') ]]; then
    fetchgit_function vtk v8.1.0 https://github.com/Kitware/vtk.git
    cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
        -D BUILD_TESTING=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_DOCUMENTATION=OFF \
        $SOURCE_DIR/vtk

    make --jobs=${CPUS}
    install_function vtk 8.1.0
    clean_function vtk
fi
}
