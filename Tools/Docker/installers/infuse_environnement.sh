#!/usr/bin/env bash

  echo "Sourcing INFUSE environnement"
# add package config path if not present
[[ ":$PKG_CONFIG_PATH:" != *":$INSTALL_DIR/lib/pkgconfig:"* ]] && PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:$INSTALL_DIR/lib/pkgconfig"
# add bin install dir to path if not present
[[ ":$PATH:" != *"$INSTALL_DIR/bin"* ]] && PATH="${PATH}:$INSTALL_DIR/bin"

export $PKG_CONFIG_PATH
export $PATH
