#!/usr/bin/env bash

# vincent.bissonnette@magellium.fr
# This file is required by ../get-cdff-dependencies.sh

# ## Pangolin latest ========================================================
#
# Repository      https://github.com/strasdat/Pangolin
#
# Pangolin is a lightweight portable rapid development library for managing 
# OpenGL display / interaction and abstracting video input. 
#
# It is used in the CDFF for display components of the Visual SLAM DFNs, which
# allow the user to see and manipulate the map, control execution and change
# parameters. 
#
# ### Dependencies ----------------------------------------------------------
#
# C++11, libglew
#
# ### Dependants ------------------------------------------------------------
#
# CDFF::DFNs::StereoSlam

function install4infuse_pangolin {
if [[ ! -d "${INSTALL_DIR}/include/pangolin" ]]; then

  # Download library, extract, and change to resulting directory
  cdff_gitclone pangolin master git://github.com/strasdat/Pangolin.git 66df6eab2b456592884a53e7fc2805e8c2491fea
  cd "${SOURCE_DIR}/pangolin"
  mkdir build
  cd build
  
  # Build
  cmake \
    -D CMAKE_BUILD_TYPE=Release \
    "${SOURCE_DIR}/pangolin"

  make --jobs=${CPUS}

  # Install		
  cdff_makeinstall libpangolin

  # Remove extracted library
  cdff_makedistclean pangolin
fi
}
