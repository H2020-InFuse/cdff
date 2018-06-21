#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## Ceres 1.14.0 =============================================================
#
# Download        http://ceres-solver.org/ceres-solver-1.14.0.tar.gz
# Repository      https://ceres-solver.googlesource.com/ceres-solver
# Release notes   http://ceres-solver.org/version_history.html
#
# Installation    http://ceres-solver.org/installation.html
# Documentation   http://ceres-solver.org
#
# ### Dependencies ------------------------------------------------------------
#
# Ceres has the following dependencies:
#
# * Required: Eigen 3.1.0+, CMake 2.8.0+
#
# * Recommended: Eigen 3.2.2+, glog 0.3.1+ (apt: libgoogle-glog-dev),
#     SuiteSparse (apt: libsuitesparse-dev)
#
# * Optional but required for SuiteSparse: BLAS and LAPACK, both provided by
#     ATLAS for instance (apt: libatlas-base-dev)
#
# * Optional: gflags to build examples and tests (apt: libgflags-dev),
#     Threading Building Blocks (apt: libtbb-dev) to have multithreading
#     support provided by Threading Building Blocks instead of OpenMP or
#     C++11 primitives (all multithreading is off by default)
#
# Make sure your system includes the aforementioned libraries before building,
# installing, and using Ceres. Both the headers and the runtime libraries are
# required.
#
# ### Dependants --------------------------------------------------------------
#
# Only CDFF

function install4infuse_ceres {
if [[ ! -d "${INSTALL_DIR}/include/ceres" ]]; then

  # Download source code, extract, and change to resulting directory
  fetchsource_function ceres ceres-solver-1.14.0.tar.gz http://ceres-solver.org/

  # Build
  mkdir build
  cd build

  cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D BUILD_SHARED_LIBS=ON \
    -D EIGEN_INCLUDE_DIR="${INSTALL_DIR}/include/" \
    -D BUILD_TESTING=OFF -D BUILD_EXAMPLES=OFF \
    -D CMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    ../

  make --jobs=${CPUS}

  # Install
  install_function ceres 1.14.0

  # Remove source and build directories
  clean_function ceres
fi
}
