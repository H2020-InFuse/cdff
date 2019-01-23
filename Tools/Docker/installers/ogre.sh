#!/usr/bin/env bash

# clement.bazerque@magellium.fr
# This file is required by ../get-cdff-dependencies.sh

# ## ogre latest ========================================================
#
# Repository      https://github.com/OGRECave/ogre
#
# Ogre is a scene-oriented, flexible 3D engine written in C++.
#
# It is used in the CDFF by Visp as a dependency.
#
# ### Dependencies ----------------------------------------------------------
#
# libfreetype6-dev, libglew, libfreeimage-dev, libzzip-dev, libxrandr-dev, libxaw7-dev, freeglut3-dev, libgl1-mesa-dev, libglu1-mesa-dev, nvidia-cg-toolkit, libois-dev
#
# ### Dependants ------------------------------------------------------------
#
# CDFF::DFNs::StereoModelBasedTracking

function install4infuse_ogre {
if [[ ! -d "${INSTALL_DIR}/include/ogre" ]]; then

  # Download library, extract, and change to resulting directory
  wget -q https://github.com/OGRECave/ogre/archive/v1.9.1.tar.gz
  tar xf v1.9.1.tar.gz
  cd ogre-1.9.1
  mkdir build
  cd build
  
  # Build
  cmake -DCMAKE_BUILD_TYPE=Release -DOGRE_BUILD_SAMPLES=OFF -DOGRE_BUILD_TESTS=OFF "${SOURCE_DIR}/ogre-1.9.1"

  make --jobs=${CPUS}

  # Install		
  cdff_makeinstall ogre 1.9.1

  # Remove extracted library
  cdff_makedistclean ogre
fi
}
