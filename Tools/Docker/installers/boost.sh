#!/usr/bin/env bash

# xma@spaceapplications.com, romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## Boost 1.66.0 =============================================================
#
# Download        https://dl.bintray.com/boostorg/release/1.66.0/source/
# Release notes   http://www.boost.org/users/history/version_1_66_0.html
#
# Installation    http://www.boost.org/doc/libs/1_66_0/more/getting_started/unix-variants.html
# Documentation   http://www.boost.org/doc/libs/1_66_0/
#
# Most Boost.* libraries are header-only. Some must be built, and some others
# have optional binary components: see the installation webpage for a list,
# or run ./bootstrap.sh --show-libraries in the Boost source directory.
#
# ### Dependencies ------------------------------------------------------------
#
# Most Boost.* libraries depend only on:
#
# * The C/C++ standard libraries (apt:libc6-dev apt:libstdc++-5-dev)
# * The GCC support library (apt:libgcc-5-dev)
# * Other Boost.* libraries
#
# In particular, AFAICT:
#
# * Beast optionally requires OpenSSL
# * Compute requires OpenGL
# * GraphParallel requires Open MPI (apt:libopenmpi-dev)
# * Iostreams requires zlib, libbzip2, liblzma (apt:zlib1g-dev libbz2-dev liblzma-dev)
# * Locale recommends ICU v3.6+ (apt:libicu-dev)
# * MPI requires Open MPI (apt:libopenmpi-dev)
# * Python requires Python v2.2+
# * Regex optionally requires ICU (apt:libicu-dev)
#
# Make sure your system includes the aforementioned libraries, in decently
# recent versions (the Boost documentation rarely mentions which minimum
# version of a dependency is required), before building, installing, and using
# the corresponding Boost.* libraries. Assumedly, both the headers and the
# runtime libraries are required.
#
# ### Dependants --------------------------------------------------------------
#
# * PCL
# * CDFF
#
# Neither we nor the PCL need *all* the Boost.* libraries that require building:
#
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
#   + undocumented, but at least SmartPtr
#
# * Compiled Boost libraries used by the CDFF:
#
#   + Serialization
#     Used by MAG
#     Used by EnviRe in CDFF-dev: EnviRe uses it to serialize its graphs
#   + Timer
#     Used by MAG MiddlewareSupport
#
# * Boost header-only libraries used by the CDFF:
#
#   + SmartPtr
#   + String_Algo

function install4infuse_boost {
if [[ ! -d "${INSTALL_DIR}/include/boost" ]]; then

  # Download source code, extract, and change to resulting directory
  fetchsource_function boost boost_1_66_0.tar.gz https://dl.bintray.com/boostorg/release/1.66.0/source/

  # Build and install
  mkdir build
  ./bootstrap.sh \
    --with-libraries=date_time,filesystem,iostreams,system,thread,chrono,serialization,timer \
    --prefix="${INSTALL_DIR}"
  ./b2 --build-dir=build -q -j ${CPUS} link=shared install

  # Patch: support for Boost 1.66.0 in the CMake find module FindBoost.cmake is
  # only available from the module shipped with CMake 3.11: download that module
  mkdir -p "${INSTALL_DIR}/share/cmake-3.11.4/Modules"
  cd "${INSTALL_DIR}/share/cmake-3.11.4/Modules"
  wget https://gitlab.kitware.com/cmake/cmake/raw/v3.11.4/Modules/FindBoost.cmake

  # Remove source and build directories
  clean_function boost
fi
}
