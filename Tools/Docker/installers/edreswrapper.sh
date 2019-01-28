#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../get-cdff-dependencies.sh

# ## Edres-Wrapper 1.0.0 ======================================================
#
# Download   ftp://ftp.magellium.fr/edreswrapper-sdk-1.0.0.tar.gz
#
# Edres (Environnement de Développement pour la Robotique d'Exploration
# Spatiale) is a proprietary C library developped by CNES as a framework for
# autonomous space robotics.
#
# Edres-Wrapper is a C++ overlay that contains Edres and exposes core functions
# of Edres for the CDFF to use.
#
# ### Dependencies ------------------------------------------------------------
#
# OpenCV-2.4.3, libgsl-0, libgslblas-0
#
# ### Dependants --------------------------------------------------------------
#
# CDFF

function install4infuse_edres-wrapper {
if [[ ! -d "${INSTALL_DIR}/include/edres-wrapper" ]]; then

  # Extract library and change to resulting directory
  tar xvzf "${SOURCE_DIR}/edreswrapper-sdk-1.0.0.tar.gz" -C ${SOURCE_DIR}  
  rm "${SOURCE_DIR}/edreswrapper-sdk-1.0.0.tar.gz"
  cd "${SOURCE_DIR}/edreswrapper-sdk-1.0.0"

  # Install
  if [[ ${INSTALL_AS_ROOT} == yes ]]; then
    sudo install -m 0644 -D -t "${INSTALL_DIR}/share/edres-wrapper/" EDRES_License_Signed_SpaceApps.pdf
    sudo install -m 0644 -D -t "${INSTALL_DIR}/share/edres-wrapper/" Edres-WrapperConfig.cmake
    sudo install -m 0644 -D -t "${INSTALL_DIR}/include/edres-wrapper/" include/edres-wrapper/*.h
    sudo install -m 0644 -D -t "${INSTALL_DIR}/lib/edres-wrapper/" lib/*
    sudo ln -s libopencv_calib3d.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_calib3d.so.2.4"
    sudo ln -s libopencv_core.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_core.so.2.4"
    sudo ln -s libopencv_features2d.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_features2d.so.2.4"
    sudo ln -s libopencv_flann.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_flann.so.2.4"
    sudo ln -s libopencv_highgui.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_highgui.so.2.4"
    sudo ln -s libopencv_imgproc.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_imgproc.so.2.4"
    sudo ln -s libopencv_legacy.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_legacy.so.2.4"
    sudo ln -s libopencv_ml.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_ml.so.2.4"
    sudo ln -s libopencv_nonfree.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_nonfree.so.2.4"
    sudo ln -s libopencv_video.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_video.so.2.4"

  else
    install -m 0664 -D -t "${INSTALL_DIR}/share/edres-wrapper/" EDRES_License_Signed_SpaceApps.pdf
    install -m 0664 -D -t "${INSTALL_DIR}/share/edres-wrapper/" Edres-WrapperConfig.cmake
    install -m 0664 -D -t "${INSTALL_DIR}/include/edres-wrapper/" include/edres-wrapper/*.h
    install -m 0664 -D -t "${INSTALL_DIR}/lib/edres-wrapper/" lib/*
    ln -s libopencv_calib3d.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_calib3d.so.2.4"
    ln -s libopencv_core.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_core.so.2.4"
    ln -s libopencv_features2d.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_features2d.so.2.4"
    ln -s libopencv_flann.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_flann.so.2.4"
    ln -s libopencv_highgui.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_highgui.so.2.4"
    ln -s libopencv_imgproc.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_imgproc.so.2.4"
    ln -s libopencv_legacy.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_legacy.so.2.4"
    ln -s libopencv_ml.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_ml.so.2.4"
    ln -s libopencv_nonfree.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_nonfree.so.2.4"
    ln -s libopencv_video.so.2.4.3 "${INSTALL_DIR}/lib/edres-wrapper/libopencv_video.so.2.4"
  fi

  # Remove extracted library
  cdff_makedistclean edres-wrapper
fi
}
