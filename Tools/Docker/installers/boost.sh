#!/usr/bin/env bash

# xma@spaceapplications.com, romain.michalec@strath.ac.uk
# This file is required by ../get-cdff-dependencies.sh

# ## Boost 1.66.0 =============================================================
#
# Download        https://dl.bintray.com/boostorg/release/1.66.0/source/
# Release notes   http://www.boost.org/users/history/version_1_66_0.html
#
# Installation    http://www.boost.org/doc/libs/1_66_0/more/getting_started/unix-variants.html
# Documentation   http://www.boost.org/doc/libs/1_66_0/
#
# Most Boost.* libraries are header-only. Some must be built, and others
# have optional binary components: see the installation webpage for a list,
# or run ./bootstrap.sh --show-libraries in the Boost source directory.
#
# ### Dependencies ------------------------------------------------------------
#
# Most Boost.* libraries depend only on:
#
# * The C/C++ standard libraries (apt:libc6-dev libstdc++-5-dev)
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
# Make sure your system includes the aforementioned libraries (both headers and
# shared objects), in decently recent versions (the Boost documentation rarely
# mentions which minimum version of a dependency is required), before building
# and installing the corresponding Boost.* libraries.
#
# ### Dependants --------------------------------------------------------------
#
# * libpointmatcher
# * PCL
# * CDFF
#
# Neither the CDFF nor its dependencies require *all* the Boost.* libraries.
# The following list are those required by the CDFF. For those required by the
# dependencies, see their respective installers.
#
# * Compiled Boost libraries used by the CDFF:
#
#   + Serialization (in topic branches only)
#     - Used by MAG
#     - Used by DFKI's base-boost_serialization, a library for serializing the
#       base types of EnviRe
#   + Timer (in topic branches only)
#     - Used by MAG MiddlewareSupport
#   + Atomic
#     - Dependency of Ogre3D, which is a dependency of VISP, used for the model-based tracking DFNs
#
# * Header-only Boost libraries used by the CDFF:
#
#   + SmartPtr
#   + String_Algo

function install4infuse_boost {

# Which compiled Boost libraries should be installed?
compiled_libraries=(date_time filesystem iostreams system thread chrono serialization timer program_options atomic)

# Check if all desired Boost libraries are present
if [[ -d "${INSTALL_DIR}/include/boost" ]]; then
  # Boost headers and header-only Boost libraries are present
  boost_ok=true
  for i in "${compiled_libraries[@]}"; do
    if [[ ! (-f "${INSTALL_DIR}/lib/libboost_${i}.so" || -f "${INSTALL_DIR}/lib/libboost_${i}.a") ]]; then
      # Missing compiled Boost library: libboost_${i}.(so|a)
      boost_ok=false
    fi
  done
else
  # Missing Boost headers and header-only Boost libraries
  boost_ok=false
fi

# Install Boost if missing, reinstall it if a desired library is missing
if [[ ${boost_ok} = false ]]; then

  # Download source code, extract, and change to resulting directory
  cdff_wget boost boost_1_66_0.tar.gz https://dl.bintray.com/boostorg/release/1.66.0/source/
  cd boost_1_66_0
  # Build and install
  mkdir build
  ./bootstrap.sh \
    --with-libraries="$(echo "${compiled_libraries[@]}" | tr " " ,)" \
    --prefix="${INSTALL_DIR}"
  ./b2 --build-dir=build -q -j ${CPUS} link=shared install

  # Patch: support for Boost 1.66.0 in the CMake find module FindBoost.cmake is
  # only available from the module shipped with CMake 3.11: download that module
  mkdir -p "${INSTALL_DIR}/share/cmake-3.11.4/Modules"
  cd "${INSTALL_DIR}/share/cmake-3.11.4/Modules"
  wget https://gitlab.kitware.com/cmake/cmake/raw/v3.11.4/Modules/FindBoost.cmake

  # Remove source and build directories
  cdff_makedistclean boost

fi
}
