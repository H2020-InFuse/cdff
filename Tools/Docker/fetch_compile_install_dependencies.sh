#!/bin/bash
#xma@spaceapplications.com
#This file fetches the dependencies in /Externals, builds and installs them.
# Version 1.0

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
INFUSE_INSTALLER_PREFIX="install4infuse_"
declare -A infuse_dependencies_map

BUILD_DIR=$DIR"/build"
INSTALL_DIR=$DIR"/install"
PKG_DIR=$DIR"/package"


# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

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
  -c                      Print current Configuration only

EOF
}

function show_configuration {

  echo "Dependencies that will be BUILT : "
  for i in "${InstallersToRUN[@]}"
  do
    if [[ ${infuse_dependencies_map[$i]} ]] ;  then
    echo "$i";
  fi
  done

	echo "build directory    = ${BUILD_DIR}"
	echo "install directory  = ${INSTALL_DIR}"
	echo "Packages directory = ${PKG_DIR}"
}

# imports all functions present in all scripts in "/dependencies" folder
function find_installers {
  if [ ! -d installers ]; then
    echo "/installers directory missing"
    exit
  fi

  for i in `find installers/ -name "*.sh"`
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
  envfile=installers/infuse_environnement.sh
  if [ ! -f "$envfile" ]; then
    echo "installers/infuse_environnement.sh missing, cannot set INFUSE ENV."
    exit
  fi
  source installers/infuse_environnement.sh
}

function run_installers {
  set_environnement

  mkdir -p $BUILD_DIR
  cd $BUILD_DIR
  for i in "${InstallersToRUN[@]}"
  do
    if [[ ${infuse_dependencies_map[$i]} ]] ;  then
      # RUN the actual function
      scripttorun=${infuse_dependencies_map[$i]}
      echo "Running INFUSE $i installer"
    #  scripttorun
      echo "INFUSE $i installer Done."
  fi
  done
}

function install_function {
if (command -v checkinstall); then
   checkinstall -y --pakdir $PKG_DIR --nodoc --pkgversion="$1"
else
   make install
fi
}

function fetchsource_function {
	echo "Installing $1"
	git -C $DIR clone --depth 1 --single-branch -b $2 $3
	mkdir -p $BUILD_DIR/$1
	cd $BUILD_DIR/$1
}

function clean_function {
	#rm -rf $DIR/$1
	#rm -rf $BUILD_DIR/$1
	echo "$1 installation Done."
}

###### MAIN PROGRAMM

InstallersToRUN=()
find_installers
# get options
while getopts ":a:b:i:p:s:c" opt; do
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
shift $((OPTIND-1))
[ "$1" = "--" ] && shift

show_configuration
run_installers
