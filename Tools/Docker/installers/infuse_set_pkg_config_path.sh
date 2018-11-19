#!/usr/bin/env bash


#Get working directory and script containing directory
SOURCE_AUX="${BASH_SOURCE[0]}"
while [ -h "$SOURCE_AUX" ]; do # resolve $SOURCE_AUX until the file is no longer a symlink
  DIR_AUX="$( cd -P "$( dirname "$SOURCE_AUX" )" && pwd )"
  SOURCE_AUX="$(readlink "$SOURCE_AUX")"
  [[ $SOURCE_AUX != /* ]] && SOURCE_AUX="$DIR_AUX/$SOURCE_AUX" # if $SOURCE_AUX was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR_AUX="$( cd -P "$( dirname "$SOURCE_AUX" )" && pwd )"
INSTALL_DIR_AUX="$(readlink -m $DIR_AUX"/../../../External/install")"

AddToPkg=$INSTALL_DIR_AUX/lib/pkgconfig:$INSTALL_DIR_AUX/share/pkgconfig

# add package config path if not present
if [[ ":$PKG_CONFIG_PATH:" != *":$AddToPkg:"* ]];then
 PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:$AddToPkg"
 #add to current shell
 export PKG_CONFIG_PATH
  #add to system 

fi

export LD_LIBRARY_PATH=$INSTALL_DIR_AUX/lib
export Rock_DIR=$INSTALL_DIR_AUX/share/rock
export console_bridge_DIR=$INSTALL_DIR_AUX/share/console_bridge
