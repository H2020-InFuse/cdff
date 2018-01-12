#!/bin/bash
#xma@spaceapplications.com
#This file fetches the dependencies in /Externals, builds and installs them.
# Version 1.0

#exit immediately if a simple command exits with a nonzero exit value.
#set -e

#Get working directory and script containing directory
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

# directory where the files will be build and installed
BUILD_DIR=$DIR"/build"
INSTALL_DIR=$DIR"/install"
PKG_DIR=$DIR"/package"


# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

function show_help {
	echo "Usage -b <build directory> -i <install directory> -p <package directory> -a [build all] -s <build only specified package>"
}


function show_defaults {
	echo "build directory    = ${BUILD_DIR}"
	echo "install directory  = ${INSTALL_DIR}"
	echo "Packages directory = ${PKG_DIR}"
}

function show_build {
echo $function
}

while getopts "ab:i:p:s:" opt; do
    case "$opt" in
    h|\?)
        show_help
        exit 0
        ;;
    a) 
	echo "-a was triggered, Parameter: $OPTARG" >&2
	;;
    b)  
	BUILD_DIR=$OPTARG
        ;;
    i)  
	INSTALL_DIR=$OPTARG
        ;;
    p)  
	PKG_DIR=$OPTARG
        ;;
    s)  
	function=$OPTARG
	show_build
        ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
    esac
done
shift $((OPTIND-1))
[ "$1" = "--" ] && shift
show_defaults





