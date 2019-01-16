#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../get-cdff-dependencies.sh

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
  cdff_wget edres-wrapper edreswrapper-sdk-1.0.0.tar.gz http://web.magellium.fr/~H2020INFUSE/
  cd edreswrapper-sdk-1.0.0

  # Install
  if [[ ${INSTALL_AS_ROOT} == yes ]]; then
    sudo install -m 0644 -D -t "${INSTALL_DIR}/share/edres-wrapper/" Edres-WrapperConfig.cmake
    sudo install -m 0644 -D -t "${INSTALL_DIR}/include/edres-wrapper/" include/edres-wrapper/*.h
    sudo install -m 0644 -D -t "${INSTALL_DIR}/lib/edres-wrapper/" lib/libedres-wrapper.so
  else
    install -m 0664 -D -t "${INSTALL_DIR}/share/edres-wrapper/" Edres-WrapperConfig.cmake
    install -m 0664 -D -t "${INSTALL_DIR}/include/edres-wrapper/" include/edres-wrapper/*.h
    install -m 0664 -D -t "${INSTALL_DIR}/lib/edres-wrapper/" lib/libedres-wrapper.so
  fi

  # Remove extracted library
  cdff_makedistclean edres-wrapper
fi
}
