#!/usr/bin/env bash

#xma@spaceapplications.com
#This file is needed by ../get-cdff-dependencies.sh
# Version 1.0

function install4infuse_vtk {
if [[ ! -n $(find $PKG_DIR -name 'vtk*') ]]; then
    cdff_gitclone vtk v8.1.0 https://github.com/Kitware/vtk.git
    mkdir build
    cd build
    cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
        -D BUILD_TESTING=OFF \
        -D BUILD_EXAMPLES=OFF \
        -D BUILD_DOCUMENTATION=OFF \
        -D VTK_ENABLE_KITS=ON \
        $SOURCE_DIR/vtk

    make --jobs=${CPUS}
    cdff_makeinstall vtk 8.1.0
    cdff_makedistclean vtk
fi
}
