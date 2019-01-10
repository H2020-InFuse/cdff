#!/usr/bin/env bash

# clement.bazerque@magellium.fr
# This file is required by ../get-cdff-dependencies.sh

# ## Visp latest ========================================================
#
# Repository      https://github.com/lagadic/visp
#
# Visp is a library that allows developing applications using visual tracking 
# and visual servoing technics.
#
# It is used in the CDFF for model-based tracking.
#
# ### Dependencies ----------------------------------------------------------
#
# Ogre, Coin3d, Realsense2
#
# ### Dependants ------------------------------------------------------------
#
# CDFF::DFNs::StereoModelBasedTracking

function install4infuse_visp {
if [[ ! -d "${INSTALL_DIR}/include/visp" ]]; then

  # Download library, extract, and change to resulting directory
  cdff_wget visp 3.1.0.tar.gz https://github.com/lagadic/visp/archive/
  mkdir build
  cd build
  
  # Build
  cmake \
    -DUSE_CPP11=ON \
    -DBUILD_DEMOS=OFF \
    -DBUILD_TESTS=OFF \
    -DBUILD_TUTORIALS=OFF \
    -DBUILD_EXAMPLES=OFF \
    -DUSE_YARP=OFF \
    -DUSE_REALSENSE2=ON \
    -DUSE_PCL=ON \
    -DBUILD_MODULE_visp_ar=ON \
    -DBUILD_MODULE_visp_blob=OFF \
    -DBUILD_MODULE_visp_core=ON \
    -DBUILD_MODULE_visp_detection=ON \
    -DBUILD_MODULE_visp_gui=ON \
    -DBUILD_MODULE_visp_imgproc=OFF \
    -DBUILD_MODULE_visp_io=ON \
    -DBUILD_MODULE_visp_klt=ON \
    -DBUILD_MODULE_visp_mbt=ON \
    -DBUILD_MODULE_visp_me=ON \
    -DBUILD_MODULE_visp_robot=OFF \
    -DBUILD_MODULE_visp_sensor=OFF \
    -DBUILD_MODULE_visp_tt=OFF \
    -DBUILD_MODULE_visp_tt_mi=OFF \
    -DBUILD_MODULE_visp_vision=ON \
    -DBUILD_MODULE_visp_visual_features=ON \
    -DBUILD_MODULE_visp_vs=OFF \
    "${SOURCE_DIR}/visp"

  make --jobs=${CPUS}

  # Install		
  cdff_makeinstall visp 3.1.0

  # Remove extracted library
  cdff_makedistclean visp
fi
}
