#!/bin/bash

# xma@spaceapplications.com, romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh
# Version 1.1

# # Boost 1.66.0
#
# Download               https://dl.bintray.com/boostorg/release/1.66.0/source/
# Release notes          http://www.boost.org/users/history/version_1_66_0.html
# General documentation  http://www.boost.org/doc/libs/1_66_0/more/getting_started/unix-variants.html
# Library documentation  http://www.boost.org/doc/libs/1_66_0/
#
# Most Boost.* libraries are header-only. Some must be built, and some others
# have optional binary components: see the general documentation for a list,
# or run ./bootstrap.sh --show-libraries in the Boost source directory.
#
# ## Dependencies
#
# Most Boost.* libraries depend only on:
#
# * The C/C++ standard libraries (apt:libc6-dev apt:libstdc++-5-dev)
# * The GCC support library (apt:libgcc-5-dev)
# * Other Boost.* libraries
#
# In particular, AFAICT:
#
# * Boost.Beast optionally requires OpenSSL
# * Boost.Compute requires OpenGL
# * Boost.GraphParallel requires Open MPI (apt:libopenmpi-dev) *DISABLED*
# * Boost.Iostreams requires zlib (apt:zlib1g-dev), libbzip2 (apt:libbz2-dev), liblzma (apt:liblzma-dev)
# * Boost.Locale recommends ICU v3.6+ (apt:libicu-dev) *DISABLED*
# * Boost.MPI requires Open MPI (apt:libopenmpi-dev) *DISABLED*
# * Boost.Python requires Python v2.2+
# * Boost.Regex optionally requires ICU (apt:libicu-dev)
#
# Make sure your system includes all these aforementioned libraries, in decently
# recent versions (the Boost documentation rarely mentions which minimum version
# of a dependency is required), before building, installing, and using Boost.
# Assumedly, both the headers and the runtime libraries are required.
#
# ## Dependants
#
# * PCL
# * CDFF
#
# Neither we nor the PCL need *all* the Boost.* libraries that require building.
# We must aim at maintaining a list of those that are actually needed, and only
# build and install those.
#
# Currently disabled libraries:
#
# * Boost.GraphParallel
# * Boost.Locale
# * Boost.MPI

function install4infuse_boost {
if [[ ! -d "${INSTALL_DIR}/include/boost" ]]; then

  # Download source code, extract, and change to resulting directory
  fetchsource_function boost boost_1_66_0.tar.gz https://dl.bintray.com/boostorg/release/1.66.0/source/

  # Build and install
  ./bootstrap.sh --without-libraries=graph_parallel,locale,mpi --prefix="${INSTALL_DIR}"
  ./b2 install

  # Patch: support for Boost 1.66.0 in the CMake find module FindBoost.cmake is
  # only available from the module shipped with CMake 3.11: download that module
  mkdir -p "${INSTALL_DIR}/share/cmake-3.11.0-rc4/Modules"
  cd "${INSTALL_DIR}/share/cmake-3.11.0-rc4/Modules"
  wget https://gitlab.kitware.com/cmake/cmake/raw/v3.11.0-rc4/Modules/FindBoost.cmake

  # Remove source/build directory
  clean_function boost
fi
}
