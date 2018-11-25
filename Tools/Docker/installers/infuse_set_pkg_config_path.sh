#!/usr/bin/env bash

# This script is meant to be sourced by the main installer

# Add a couple pkg-config directories to PKG_CONFIG_PATH, if they aren't present
# in PKG_CONFIG_PATH already, and export PKG_CONFIG_PATH to the environment
AddToPkg="${INSTALL_DIR}/lib/pkgconfig:${INSTALL_DIR}/share/pkgconfig"
if [[ ":${PKG_CONFIG_PATH}:" != *":${AddToPkg}:"* ]]; then
  export PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:${AddToPkg}"
fi

export LD_LIBRARY_PATH="${INSTALL_DIR}/lib"
export Rock_DIR="${INSTALL_DIR}/share/rock"
export console_bridge_DIR="${INSTALL_DIR}/share/console_bridge"
