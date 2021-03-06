#!/usr/bin/env bash

# romain.michalec@strath.ac.uk
# This file is required by ../get-cdff-dependencies.sh

# ## DLRTracker-core 1.0.0 ====================================================
#
# Contact   DLR's Institute of Robotics and Mechatronics
#
# DLRTracker is a proprietary C++ library developped by DLR. It is not publicly
# available, and is provided directly by DLR to the concerned parties.
#
# Assuming the library is available to you as an archive called DLRtrackerCore-
# 1.0.0.zip and containing the following files:
#
# DLRtracker_core
# ├── DLRtracker_coreConfig.cmake
# ├── include
# │   └── .h header files
# └── lib
#     ├── libDLRtracker_core.so -> libDLRtracker_core.so.1
#     ├── libDLRtracker_core.so.1 -> libDLRtracker_core.so.1.0.0
#     └── libDLRtracker_core.so.1.0.0
#
# you can put the archive in "${SOURCE_DIR}", and use this installation script
# to install the library in "${INSTALL_DIR}" as follow:
#
# ${INSTALL_DIR}
# ├── share
# │   └── DLRtracker_core
# │       └── DLRtracker_coreConfig.cmake
# ├── include
# │   └── DLRtracker_core
# │       └── .h header files
# └── lib
#     └── DLRtracker_core
#         ├── libDLRtracker_core.so -> libDLRtracker_core.so.1
#         ├── libDLRtracker_core.so.1 -> libDLRtracker_core.so.1.0.0
#         └── libDLRtracker_core.so.1.0.0
#
# ### Dependencies ------------------------------------------------------------
#
# * Required:
#   + C standard library
#   + C math library
#   + C++ standard library
#
# * Required if the library was compiled with OpenCV support:
#   + The following OpenCV libraries: core, imgcodecs, imgproc, highgui
#
# ### Dependants --------------------------------------------------------------
#
# CDFF::DFPC::ModelBasedVisualTracking

function install4infuse_dlrtracker-core {
if [[ ! -d "${INSTALL_DIR}/include/DLRtracker_core" ]]; then

  # Extract library and change to resulting directory
  unzip -d "${SOURCE_DIR}/DLRTracker-core" "${SOURCE_DIR}/DLRtrackerCore-1.0.0.zip"
  rm "${SOURCE_DIR}/DLRtrackerCore-1.0.0.zip"
  cd "${SOURCE_DIR}/DLRTracker-core/DLRtracker_core"

  # Install
  if [[ ${INSTALL_AS_ROOT} == yes ]]; then
    sudo install -m 0644 -D -t "${INSTALL_DIR}/share/DLRtracker_core/" DLRtracker_coreConfig.cmake
    sudo install -m 0644 -D -t "${INSTALL_DIR}/include/DLRtracker_core/" include/*.h
    sudo install -m 0644 -D -t "${INSTALL_DIR}/lib/DLRtracker_core/" lib/libDLRtracker_core.so.1.0.0
    sudo ln -s libDLRtracker_core.so.1.0.0 "${INSTALL_DIR}/lib/DLRtracker_core/libDLRtracker_core.so.1"
    sudo ln -s libDLRtracker_core.so.1 "${INSTALL_DIR}/lib/DLRtracker_core/libDLRtracker_core.so"
  else
    install -m 0664 -D -t "${INSTALL_DIR}/share/DLRtracker_core/" DLRtracker_coreConfig.cmake
    install -m 0664 -D -t "${INSTALL_DIR}/include/DLRtracker_core/" include/*.h
    install -m 0664 -D -t "${INSTALL_DIR}/lib/DLRtracker_core/" lib/libDLRtracker_core.so.1.0.0
    ln -s libDLRtracker_core.so.1.0.0 "${INSTALL_DIR}/lib/DLRtracker_core/libDLRtracker_core.so.1"
    ln -s libDLRtracker_core.so.1 "${INSTALL_DIR}/lib/DLRtracker_core/libDLRtracker_core.so"
  fi

  # Remove extracted library
  cdff_makedistclean DLRTracker-core
fi
}
