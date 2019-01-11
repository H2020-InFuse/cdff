#!/usr/bin/env bash

# clement.bazerque@magellium.fr
# This file is required by ../get-cdff-dependencies.sh

# ## Coin3d latest ========================================================
#
# Repository      https://github.com/strasdat/Pangolin
#
# Coin 3D is a library used for creating 3D graphics applications. In Visp,
# Coin 3D brings support for CAD models import, simulation and augmented
# reality capabilities.
#
# It is used in the CDFF by Visp as a dependency.
# It has to be installed from sources as the package available in the ubuntu 
# repository is based on a different boost version that the one used in the CDFF.
#
# ### Dependencies ----------------------------------------------------------
#
# libx11-6, libbz2-1.0, libfreetype6, libsimage-dev, zlib1g, libgl1-mesa-dev, 
# libxext-dev, libx11-dev, libxi-dev, libxmu-dev, libxpm-dev, libxt-dev
#
# ### Dependants ------------------------------------------------------------
#
# CDFF::DFNs::StereoModelBasedTracking

function install4infuse_coin3d {
if [[ ! -d "${INSTALL_DIR}/include/coin3d" ]]; then

  # Download library, extract, and change to resulting directory
  wget -q https://bitbucket.org/Coin3D/coin/get/default.tar.gz -O coin3d.tar.gz
  mkdir coin3d
  tar xf coin3d.tar.gz -C coin3d --strip-components=1
  mkdir build
  cd build
  
  # Build
  ../coin3d/configure
  make --jobs=${CPUS}

  # Install		
  cdff_makeinstall coin3d

  # Remove extracted library
  cdff_makedistclean coin3d
fi
}
