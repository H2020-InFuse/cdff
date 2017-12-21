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
BUILD_DIR=$DIR"/build"
INSTALL_DIR=$DIR"/install"
PKG_DIR=$DIR"/package"

if [ $# -eq 0 ]
  then
    echo "No arguments supplied, 
    using 
BUILD directory :$BUILD_DIR 
INSTALL directory :$INSTALL_DIR 	
"
  else
	if [ "$#" -ne 2 ]; then	
	    echo "Usage: script.sh [BUILD_DIR] [INSTALL_DIR]"
	else
	    BUILD_DIR=$1
    	    INSTALL_DIR=$2
	fi
fi

#update submodules
#git submodule update --remote --recursive --depth 1

# if exist remove previous packages

mkdir -p $BUILD_DIR
cd $BUILD_DIR

function install_function {
if (command -v checkinstall); then
   sudo checkinstall -y --pakdir $PKG_DIR --nodoc
else
   make install
fi
}  

function install_cmake {
if [ ! -d "$INSTALL_DIR" ]; then # should test for 3.10 version > installed
	echo "Installing Cmake"
	wget https://cmake.org/files/v3.10/cmake-3.10.1.tar.gz
	tar xf cmake-3.10.1.tar.gz
	cd cmake-3.10.1
	./bootstrap --prefix=$INSTALL_DIR 
	make
	install_function
	echo "Done."
fi
}

function install_boost {
if [[ ! -d "$INSTALL_DIR/include/boost" ]]; then
	mkdir -p $BUILD_DIR/boost
	cd $DIR/boost 
	./bootstrap.sh --prefix=$INSTALL_DIR 
	./b2 --build-dir=$BUILD_DIR/boost 
	./b2 install 
	./b2 --clean-all	
fi
}

function install_eigen {
if [[ ! -n $(find $DIR/package/ -name 'eigen*') ]]; then
	mkdir -p $BUILD_DIR/eigen
	cd $BUILD_DIR/eigen 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/eigen 
	install_function
fi
}

function install_flann {
if [[ ! -n $(find $DIR/package/ -name 'flann*') ]]; then
	mkdir -p $BUILD_DIR/flann
	cd $BUILD_DIR/flann 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/flann -D BUILD_TESTS=OFF -D BUILD_EXAMPLES=OFF 
	make
	install_function
fi
}

function install_opencv {
if [[ ! -n $(find $DIR/package/ -name 'opencv*') ]]; then
	mkdir -p $BUILD_DIR/opencv3
	cd $BUILD_DIR/opencv3
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_FFMPEG=OFF -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/opencv -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON
	make
	install_function
fi
}

function install_pcl {
if [[ ! -n $(find $DIR/package/ -name 'pcl*') ]]; then
	mkdir -p $BUILD_DIR/pcl
	cd $BUILD_DIR/pcl 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/pcl 
	make
	install_function
fi
}

function install_qhull {
if [[ ! -n $(find $DIR/package/ -name 'qhull*') ]]; then
	mkdir -p $BUILD_DIR/qhull
	cd $BUILD_DIR/qhull 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/qhull 
	make
	install_function
fi
}

function install_tinyxml2 {
if [[ ! -n $(find $DIR/package/ -name 'tinyxml2*') ]]; then
	mkdir -p $BUILD_DIR/tinyxml2
	cd $BUILD_DIR/tinyxml2 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/tinyxml2 -D BUILD_TESTING=OFF -D BUILD_TESTS=OFF
	install_function
fi
}

function install_vtk {
if [[ ! -n $(find $DIR/package/ -name 'vtk*') ]]; then
	mkdir -p $BUILD_DIR/vtk
	cd $BUILD_DIR/vtk 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/vtk -D BUILD_TESTING=OFF
	make
	install_function
fi
}

function install_yamlcpp {
if [[ ! -n $(find $DIR/package/ -name 'yamlcpp*') ]]; then
	mkdir -p $BUILD_DIR/yamlcpp
	cd $BUILD_DIR/yamlcpp 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/yamlcpp 
	make
	install_function
fi
}

# execute in this order to fix dependencies
 install_cmake
 install_boost
 install_eigen
 install_flann
 install_qhull
 install_tinyxml2
 install_yamlcpp
 install_vtk
 install_opencv
 install_pcl

# delete but install directory 











