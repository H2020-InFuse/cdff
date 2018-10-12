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
# Edres-wrapper has the following dependencies:
#
# * Required: boost1.53, gsl, jpeg.62, png15, atlas3, opencv2.4.3 
#
# The previous libraries have to be bundled with edreswrapper because we don't 
# have full control on the version used by the edres library. 
#
# ### Dependants --------------------------------------------------------------
#
# CDFF

function install4infuse_edres-wrapper {
if [[ ! -d "${INSTALL_DIR}/include/edres-wrapper" ]]; then

  # Download source code, extract, and change to resulting directory
  fetchsource_function edres-wrapper edreswrapper-sdk-1.0.0.tar.gz http://web.magellium.fr/~H2020INFUSE/

  # Install
  install -m 0644 -D -t "${INSTALL_DIR}/include/edres-wrapper/" include/edres-wrapper/*
  install -m 0644 -D -t "${INSTALL_DIR}/lib/edres-wrapper/" lib/*
  install -m 0644 -D -t "${INSTALL_DIR}/share/edreswrapper/" EDRESWRAPPERConfig.cmake

  # Remove source directory
  clean_function edres-wrapper
fi
}
