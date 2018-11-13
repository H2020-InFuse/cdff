#!/usr/bin/env bash

# xma@spaceapplications.com, romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# * Compiled Boost libraries used by the PCL (sources: https://github.com/
#   PointCloudLibrary/pcl/blob/master/cmake/pcl_find_boost.cmake and http://
#   www.pointclouds.org/documentation/tutorials/building_pcl.php):
#
#   + Required: Date_Time Filesystem Iostreams System Thread
#   + Required if WITH_OPENNI2: Chrono
#   + Optional: MPI Serialization
#
#   TODO: OpenNI2 support by the PCL (it is an optional dependency of pcl_io)
#   is unlikely to be necessary for us
#
#   TODO: Message-Parsing Interface disabled but check that we are not using a
#   part of the PCL that could make use of it
#
# * Boost header-only libraries required by the PCL:
#
#   + Undocumented, but at least SmartPtr

function install4infuse_pcl {
if [[ ! -d "${INSTALL_DIR}/include/pcl-1.8" ]]; then

  # Download source code and change to resulting directory
  fetchgit_function pcl pcl-1.8.1 https://github.com/PointCloudLibrary/pcl.git
  cd "${SOURCE_DIR}/pcl"

  # Build: compiling the PCL is memory-intensive, so run less jobs (commands)
  # than there are processors, to make it less likely to run out of memory
  mkdir build
  cd build

  cmake \
    -D CMAKE_BUILD_TYPE=Release \
    -D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules" \
    -D CMAKE_PREFIX_PATH=../../install \
    -D CMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
    ../

  JOBS=$((CPUS/4))
  JOBS=$((JOBS > 0 ? JOBS : 1))
  make --jobs=${JOBS}

  # Install
  install_function pcl 1.8.1

  # Remove source and build directories
  clean_function pcl
fi
}
