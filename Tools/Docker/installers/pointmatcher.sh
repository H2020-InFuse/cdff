#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## libpointmatcher 1.3.0+git20181030 ========================================
#
# Repository      https://github.com/ethz-asl/libpointmatcher
# Release notes   https://github.com/ethz-asl/libpointmatcher/releases
#
# Installation    https://github.com/ethz-asl/libpointmatcher/blob/master/README.md
# Documentation   https://github.com/ethz-asl/libpointmatcher/blob/master/README.md
#
# ### Dependencies ------------------------------------------------------------
#
# libpointmatcher has the following dependencies, as of v1.3.0+git20181030:
#
# * Required:
#   + Eigen 3+
#   + Boost 1.48+
#     - Compiled Boost libraries: Thread Filesystem System ProgramOptions
#       DateTime Chrono
#     - Header-only Boost libraries: undocumented, but assumedly a number of
#       them, used either directly or by the aforementioned compiled libraries
#   + libnabo 1.0.6+
#   + For compilation only: CMake 2.8.11+
#
# * Required and included:
#   + yaml-cpp 0.3.x at most, i.e. an old-API release of yaml-cpp, as opposed
#     to versions 0.5.0+ which are new-API releases: libpointmatcher ships with
#     a 0.3.0 version of yaml-cpp, renamed yaml-cpp-pm to avoid name conflicts,
#     and it is that version which is used unless USE_SYSTEM_YAML_CPP=ON
#
# * Optional:
#   + Doxygen (apt:doxygen): if found, and if GENERATE_API_DOC=ON, HTML
#     documentation will be generated, in which case a LaTeX distribution must
#     be installed (for the equations)
#
# Make sure your system includes the aforementioned dependencies before
# building, installing, and using libpointmatcher.
#
# ### Dependants --------------------------------------------------------------
#
# CDFF::DFN::Registration3D and any DFPC using it

function install4infuse_pointmatcher {
if [[ ! -d "${INSTALL_DIR}/include/pointmatcher" ]]; then

    # Download source code and change to resulting directory
    fetchgit_function pointmatcher master https://github.com/ethz-asl/libpointmatcher.git fdec1377eb3e16919cbf156e3bf7d92f803d8405
    cd "${SOURCE_DIR}/pointmatcher"

    # Patch
    patch -p0 < "${DIR}/patches/pointmatcher-1.3.0+git20181030-fix_eigen_lookup.patch"

    # Build (defaults as follow)
    #   CMAKE_BUILD_TYPE       Release
    #   SHARED_LIBS            ON
    #   CMAKE_MODULE_PATH      empty
    #   CMAKE_PREFIX_PATH      empty
    #   USE_SYSTEM_YAML_CPP    OFF
    #   GENERATE_API_DOC       OFF
    #   USE_OPEN_MP            OFF
    #   USE_OPEN_CL            OFF
    #   CMAKE_INSTALL_PREFIX   /usr/local
    mkdir build
    cd build

    cmake \
        -D CMAKE_BUILD_TYPE=Release \
        -D SHARED_LIBS=ON \
        -D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules" \
        -D CMAKE_PREFIX_PATH="${INSTALL_DIR}" \
        -D USE_SYSTEM_YAML_CPP=OFF \
        -D GENERATE_API_DOC=OFF \
        -D USE_OPEN_MP=OFF \
        -D USE_OPEN_CL=OFF \
        -D CMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
        "${SOURCE_DIR}/pointmatcher"

    make --jobs=${CPUS}

    # Install
    install_function libpointmatcher 1.3.0+git20181030+cdff

    # Remove source and build directories
    clean_function pointmatcher

fi
}
