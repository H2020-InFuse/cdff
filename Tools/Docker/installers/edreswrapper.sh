#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## Edres-wrapper 1.0.0 ======================================================
#
# Download        http://web.magellium.fr/~H2020INFUSE/edreswrapper-sdk-1.0.0.tar.gz
# Release notes   ???
#
# Installation    ???
# Documentation   ???
#
# ### Dependencies ------------------------------------------------------------
#
# Edres-wrapper has the following dependencies:
#
# * Required: ...
#
# * Recommended: ...
#
# * Optional: ...
#
# ### Dependants --------------------------------------------------------------
#
# ...

function install4infuse_edres-wrapper {
if [[ ! -d "${INSTALL_DIR}/include/edres-wrapper" ]]; then

  # Download source code, extract, and change to resulting directory
  fetchsource_function edres-wrapper edreswrapper-sdk-1.0.0.tar.gz http://web.magellium.fr/~H2020INFUSE/

  # Install
  mkdir -p "${INSTALL_PREFIX}"/{include,lib,share/edres-wrapper/cmake}
  cp -rp include/edres-wrapper "${INSTALL_DIR}/include/"
  cp -p  lib/* "${INSTALL_DIR}/lib/"
  cp -p FindEDRESWRAPPER.cmake "${INSTALL_PREFIX}/share/edres-wrapper/cmake/"

  # Remove source directory
  clean_function edres-wrapper
fi
}
