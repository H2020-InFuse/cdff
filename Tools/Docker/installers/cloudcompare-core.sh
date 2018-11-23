#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## CloudCompare core algorithms (CCLib) 2.9.1+git20181115 ===================
#
# Homepage        https://www.cloudcompare.org
# Repository      https://github.com/CloudCompare/CloudCompare
# Release notes   https://github.com/CloudCompare/CloudCompare/releases
#
# Installation    https://github.com/CloudCompare/CloudCompare/blob/master/BUILD.md
# Documentation   https://github.com/CloudCompare/CloudCompare/blob/master/README.md
#
# CloudCompare is a 3D point cloud processing software. Its core algorithms,
# located in the CC/ subdirectory of its code base, make up a library called
# CCLib that can be built in isolation of the rest of the code base. The result
# can be installed as follow:
#
# ${INSTALL_DIR}
# ├── include
# │   └── cloudcompare-core
# │       └── .h header files
# └── lib
#     └── libcloudcompare_core.so
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

function install4infuse_cloudcompare-core {
if [[ ! -d "${INSTALL_DIR}/include/cloudcompare-core" ]]; then

    # Download source code and change to resulting directory
    fetchgit_function cloudcompare master https://github.com/CloudCompare/CloudCompare.git e1b281c2b229f8aa7dd961853cf93e130e1cfa5c
    cd "${SOURCE_DIR}/cloudcompare"

    # Patch
    patch -p0 < "${DIR}/../patches/cloudcompare-core-2.9.1+git20181115-customize_installation.patch"

    # Build
    mkdir CC/build
    cd CC/build

    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D COMPILE_CC_CORE_LIB_SHARED=ON \
        -D COMPILE_CC_CORE_LIB_WITH_QT=OFF \
        -D COMPILE_CC_CORE_LIB_WITH_CGAL=OFF \
        -D COMPILE_CC_CORE_LIB_WITH_TBB=OFF \
        -D CMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
        "${SOURCE_DIR}/cloudcompare/CC"

    make --jobs=${CPUS}

    # Install
    install_function libcloudcompare-core 2.9.1+git20181115+cdff

    # Remove source and build directories
    clean_function cloudcompare

fi
}
