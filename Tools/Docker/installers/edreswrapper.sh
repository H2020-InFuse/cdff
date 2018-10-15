#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## Edres-wrapper 1.0.0 ======================================================
#
# Download   http://web.magellium.fr/~H2020INFUSE/edreswrapper-sdk-1.0.0.tar.gz
#
# This is a wrapper around the edres library to expose core functions for the 
# CDFF.
#
# ### Dependencies ------------------------------------------------------------
#
# This is a stub version of edreswrapper which has no dependency.
#
# ### Dependants --------------------------------------------------------------
#
# CDFF

function install4infuse_edres-wrapper {
if [[ ! -d "${INSTALL_DIR}/include/edres-wrapper" ]]; then

  # Download source code, extract, and change to resulting directory
  fetchsource_function edres-wrapper edreswrapper-sdk-1.0.0.tar.gz http://web.magellium.fr/~H2020INFUSE/

  # Install
  install -m 0644 -D -t "${INSTALL_DIR}/share/edres-wrapper/" EDRESWRAPPERConfig.cmake
  install -m 0644 -D -t "${INSTALL_DIR}/include/edres-wrapper/" include/edres-wrapper/*
  install -m 0644 -D -t "${INSTALL_DIR}/lib/edres-wrapper/" lib/*

  # Remove source directory
  clean_function edres-wrapper
fi
}
