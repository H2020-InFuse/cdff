#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## CloudCompare-core (CCLib) 2.9.1+git20181104 ==============================
#
# Homepage        https://www.cloudcompare.org
# Repository      https://github.com/CloudCompare/CloudCompare
# Release notes   https://github.com/CloudCompare/CloudCompare/releases
#
# Installation    https://github.com/CloudCompare/CloudCompare/blob/master/BUILD.md
# Documentation   https://github.com/CloudCompare/CloudCompare/blob/master/README.md
#
# CloudCompare is a 3D point cloud processing software. Its core algorithms,
# located in the CC/ subdirectory of its code base, can be built in isolation
# of the rest of the code base. The result is a library called CCLib and
# installed as follow:
#
# ${INSTALL_DIR}
# ├── include
# │   └── cloudcompare
# │       └── .h header files
# └── lib
#     └── cloudcompare
#         └── libCC_CORE_LIB.so
#
# ### Dependencies ------------------------------------------------------------
#
# * Required:
#   + C++ Standard Library
#   + Standard Template Library (STL)
#
# * Optional:
#   + Qt 5, to enable parallel processing
#   + Computational Geometry Algorithms Library (CGAL), to enable Delaunay 2.5D
#     triangulation with a GPL-compliant licence
#   + Intel Threading Building Blocks (TBB), to enable some parallel processing
#
# ### Dependants --------------------------------------------------------------
#
# CDFF::DFN::Registration3D and any DFPC using it

# TODO patch to rename library, put it into lib/, use CMAKE_INSTALL_PREFIX
# TODO patch to install headers into include/

function install4infuse_cloudcompare-core {
if [[ ! -d "${INSTALL_DIR}/include/cloudcompare" ]]; then

    # Download source code and change to resulting directory
    fetchgit_function cloudcompare master https://github.com/CloudCompare/CloudCompare.git 2122acf8128ad4eb3be3baf143cf737ffc81a67e
    cd "${SOURCE_DIR}/cloudcompare/CC"

    # Build
    mkdir build
    cd build

    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D COMPILE_CC_CORE_LIB_SHARED=ON \
        -D COMPILE_CC_CORE_LIB_WITH_QT=OFF \
        -D COMPILE_CC_CORE_LIB_WITH_CGAL=OFF \
        -D COMPILE_CC_CORE_LIB_WITH_TBB=OFF \
        -D CMAKE_INSTALL_LIBDIR="${INSTALL_DIR}/lib" \
        "${SOURCE_DIR}/cloudcompare/CC"

    make --jobs=${CPUS}

    # Install the compiled library
    make --jobs=${CPUS} install

    # Install the headers
    install -m 0644 -D -t "${INSTALL_DIR}"/include/cloudcompare/ "${SOURCE_DIR}"/cloudcompare/CC/include/*.h

    # Remove source and build directories
    clean_function cloudcompare

fi
}
