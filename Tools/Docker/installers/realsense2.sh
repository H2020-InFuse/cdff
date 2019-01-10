#!/usr/bin/env bash

# clement.bazerque@magellium.fr
# This file is required by ../get-cdff-dependencies.sh

# ## Realsense2 latest ========================================================
#
# Repository      https://github.com/IntelRealSense/librealsense
#
# Realsense2 is a cross-platform library for Intel® RealSense™ depth cameras.
#
# It is used in the CDFF by Visp as a dependency.
#
# ### Dependencies ----------------------------------------------------------
#
# libssl-dev, libusb-1.0-0-dev, pkg-config, libgtk-3-dev, libglfw3-dev, 
# libgl1-mesa-dev, libglu1-mesa-dev
#
# ### Dependants ------------------------------------------------------------
#
# CDFF::DFNs::StereoModelBasedTracking

function install4infuse_realsense2 {
if [[ ! -d "${INSTALL_DIR}/include/realsense2" ]]; then

  # Download library, extract, and change to resulting directory
  wget -q https://github.com/IntelRealSense/librealsense/archive/v2.8.0.tar.gz
  tar xvf v2.8.0.tar.gz
  cd librealsense-2.8.0
  mkdir build && cd build
  
  # Build
  cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=false ..
  make --jobs=${CPUS}

  # Install		
  cdff_makeinstall realsense2

  # Remove extracted library
  cdff_makedistclean realsense2
fi
}
