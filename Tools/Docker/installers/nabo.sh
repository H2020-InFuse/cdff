#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## libnabo 1.0.6+git20180418 ================================================
#
# Repository      https://github.com/ethz-asl/libnabo
# Release notes   https://github.com/ethz-asl/libnabo/releases
#
# Installation    https://github.com/ethz-asl/libnabo/blob/master/README.md
# Documentation   https://github.com/ethz-asl/libnabo/blob/master/README.md
#
# ### Dependencies ------------------------------------------------------------
#
# libnabo has the following dependencies, as of v1.0.6+git20180418:
#
# * Required:
#   + Eigen 2+ or 3+
#   + For compilation only: CMake 2.6.0+ (apt:cmake) and grep (apt:grep)
#
# * Optional:
#   + Doxygen (apt:doxygen): if found, HTML documentation will be generated
#   + Python 2+ or 3+ (apt:python python3), NumPy (apt:python-numpy
#     python3-numpy), and Boost.Python: if found, Python bindings will be
#     generated
#
# Make sure your system includes the aforementioned dependencies before
# building, installing, and using libnabo.
#
# ### Dependants --------------------------------------------------------------
#
# CDFF::DFN::Registration3D and any DFPC using it

function install4infuse_nabo {
if [[ ! -d "${INSTALL_DIR}/include/nabo" ]]; then

    # Download source code and change to resulting directory
    fetchgit_function nabo master https://github.com/ethz-asl/libnabo.git 3b15ebca87bcb242cd3a919929cc301953552197
    cd "${SOURCE_DIR}/nabo"

    # Patch
    patch < "${DIR}/patches/nabo-1.0.6+git20180418-no_python_bindings.patch"

    # Build
    mkdir build
    cd build

    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D SHARED_LIBS=ON \
        -D CMAKE_PREFIX_PATH="${INSTALL_DIR}" \
        -D USE_OPEN_MP=OFF \
        -D USE_OPEN_CL=OFF \
        -D CMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
        "${SOURCE_DIR}/nabo"

    make --jobs=${CPUS}

    # Install
    install_function libnabo 1.0.6+git20180418+cdff

    # Remove source and build directories
    clean_function nabo

fi
}
