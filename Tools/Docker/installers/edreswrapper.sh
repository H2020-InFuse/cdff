#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../fetch_compile_install_dependencies.sh

# ## Edres-Wrapper 1.0.0 ======================================================
#
# Download   http://web.magellium.fr/~H2020INFUSE/edreswrapper-sdk-1.0.0.tar.gz
#
# Edres (Environnement de Développement pour la Robotique d'Exploration
# Spatiale) is a proprietary C library developped by CNES as a framework for
# autonomous space robotics.
#
# Edres-Wrapper is a C++ overlay that contains Edres and exposes core functions
# of Edres for the CDFF to use.
#
# This version of Edres-Wrapper is a stub which exposes the ASN.1 interfaces,
# but does nothing else. It allows Edres-dependent code to build. Such code
# will not be operational.
#
# ### Dependencies ------------------------------------------------------------
#
# The stub version has no dependency
#
# ### Dependants --------------------------------------------------------------
#
# CDFF

function install4infuse_edres-wrapper {
if [[ ! -d "${INSTALL_DIR}/include/edres-wrapper" ]]; then

  # Download library, extract, and change to resulting directory
  fetchsource_function edres-wrapper edreswrapper-sdk-1.0.0.tar.gz http://web.magellium.fr/~H2020INFUSE/

  # Install
  install -m 0644 -D -t "${INSTALL_DIR}/share/edres-wrapper/" Edres-WrapperConfig.cmake
  install -m 0644 -D -t "${INSTALL_DIR}/include/edres-wrapper/" include/edres-wrapper/*.h
  install -m 0644 -D -t "${INSTALL_DIR}/lib/edres-wrapper/" lib/libedres-wrapper.so

  # Remove extracted library
  clean_function edres-wrapper
fi
}
