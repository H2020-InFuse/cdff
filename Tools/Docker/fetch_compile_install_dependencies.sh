#!/bin/bash
#xma@spaceapplications.com
#This file fetches the dependencies in /Externals, builds and installs them.
# Version 1.3

#exit immediately if a simple command exits with a nonzero exit value.
set -e

#Get working directory and script containing directory
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

# directory where the files will be build and installed
SOURCE_DIR="$(readlink -m $DIR"/../../External/source")"
BUILD_DIR="$(readlink -m $DIR"/../../External/build")"
INSTALL_DIR="$(readlink -m $DIR"/../../External/install")"
PKG_DIR="$(readlink -m $DIR"/../../External/package")"

# How many processors?
if [ -f /proc/cpuinfo ]; then
  CPUS=$(grep --count --regexp=^processor /proc/cpuinfo)
else
  CPUS=1
fi

function show_help {

  cat <<EOF
Usage: $0 [OPTION]...

Defaults for the options are specified in brackets.

Configuration:
  -h, ?               	    Display this help and exit
  -s LIB                    Only compile LIB
			    can be used multiple times (-s LIB1 -s LIB2)
                            [LIB: cmake boost eigen flann qhull tinyxml2
			    yamlccp vtk opencv pcl]

Installation directories:
  -b DIR            	    Build all libraries in DIR
                            [$BUILD_DIR]
  -i DIR            	    Install all libraries in DIR
                            [$INSTALL_DIR]
  -p DIR            	    Directory where packages are stored
                            [$PKG_DIR]
  -c                        Print current Configuration only

EOF
}

function show_configuration {
  echo -n "Dependencies that will be BUILT : "
  for i in "${InstallersToRUN[@]}"
  do
    if [[ ${infuse_dependencies_map[$i]} ]] ;  then
      echo -n "$i ";
    fi
  done

  echo ""
  echo "source directory   = ${SOURCE_DIR}"
  echo "build directory    = ${BUILD_DIR}"
  echo "install directory  = ${INSTALL_DIR}"
  echo "Packages directory = ${PKG_DIR}"
}

# imports all functions present in all scripts in "/installers" folder
InstallersToRUN=()
INFUSE_INSTALLER_PREFIX="install4infuse_"
declare -A infuse_dependencies_map
function find_installers {
  if [ ! -d $DIR/installers ]; then
    echo "$DIR/installers directory missing"
    exit
  fi

  for i in `find $DIR/installers/ -name "*.sh"`
  do
     source $i
  done

  PreviousIFS=$IFS
  IFS=$'\n'
  for f in $(declare -F); do
    fct_name="${f:11}"
     if [[ $fct_name == $INFUSE_INSTALLER_PREFIX* ]]; then
      dep=${fct_name#$INFUSE_INSTALLER_PREFIX}
      infuse_dependencies_map[$dep]=$fct_name
    fi
  done

 echo "Found infuse installers for :" ${!infuse_dependencies_map[@]}
 IFS=$PreviousIFS
}

function set_environnement {
  envfile=$DIR/installers/infuse_environnement.env
  if [ ! -f "$envfile" ]; then
    echo "$envfile missing, cannot set INFUSE ENV."
    exit
  fi
  source $envfile
}

function run_installers {
#  set_environnement

  mkdir -p $BUILD_DIR
  mkdir -p $INSTALL_DIR
  mkdir -p $PKG_DIR
  mkdir -p $SOURCE_DIR

  cd $BUILD_DIR
  for i in "${InstallersToRUN[@]}"
  do
    if [[ ${infuse_dependencies_map[$i]} ]] ;  then
      # RUN the actual function
      echo ""
      echo "#"
      echo "# Running INFUSE $i installer"
      echo "#"
      echo ""
      eval ${infuse_dependencies_map[$i]}
      echo ""
      echo "INFUSE $i installer Done."
    fi
  done
}

function install_function {
if (command -v checkinstall); then
   checkinstall -y --pakdir $PKG_DIR --nodoc --pkgname="$1" --pkgversion="$2"
else
   make --jobs=${CPUS} install
fi
}

function fetchsource_function {
  echo "Downloading ${1}"
  mkdir -p "${SOURCE_DIR}/${1}"
  cd "${SOURCE_DIR}/${1}"
  wget "${3}${2}"
  if [[ "${2: -7}" == ".tar.gz" ]]; then
    tar xf "${2}"
    rm -f "${2}"
    cd "${2%.tar.gz}"
  fi
  echo "Downloading ${1}: done."
}

function fetchgit_function {
	echo "Checking out $1"
	git -C $SOURCE_DIR clone --depth 1 --single-branch --recursive -b $2 $3 $1
	mkdir -p $BUILD_DIR/$1
	cd $BUILD_DIR/$1
  echo "Done. $1 Checked out."
}

function clean_function {
  echo "Removing ${1} source and build directories"
  cd "${SOURCE_DIR}"
  rm -rf "${SOURCE_DIR}/${1}"
  rm -rf "${BUILD_DIR}/${1}"
  echo "Removing ${1} source and build directories: done."
}

function build_all_function {
 InstallersToRUN+=("boost")
 InstallersToRUN+=("eigen")
 InstallersToRUN+=("ceres")
 InstallersToRUN+=("flann")
 InstallersToRUN+=("qhull")
 InstallersToRUN+=("yaml-cpp")
 InstallersToRUN+=("opencv")
 InstallersToRUN+=("vtk")
 InstallersToRUN+=("pcl")
  #for i in "${!infuse_dependencies_map[@]}"
  #do
  #  InstallersToRUN+=($i)
  #done
}

###### MAIN PROGRAM

# Attempt to cleanup leftover source folders if we exited early due to errors.
function on_exit {
  for i in "${InstallersToRUN[@]}"; do
    if [[ -d  "${SOURCE_DIR}/$i" ]] ; then
          echo "Removing left over source folder: ${SOURCE_DIR}/$i"
      rm -rf "${SOURCE_DIR}/$i"
    fi
  done
}
trap on_exit EXIT

find_installers

# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.
# get options
while getopts ":b:i:p:s:c" opt; do
    case "$opt" in
    h|\?)
        show_help
        exit 0
        ;;
    c)
        show_configuration
        exit 0
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
  InstallersToRUN+=($OPTARG)
  	    ;;
    :)
      echo "Option -$OPTARG requires an argument." >&2
      exit 1
      ;;
    esac
done
if [ $OPTIND -eq 1 ]; then
  build_all_function
fi
shift $((OPTIND-1))
[ "$1" = "--" ] && shift

show_configuration
run_installers
