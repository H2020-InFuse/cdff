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

mkdir -p $BUILD_DIR
cd $BUILD_DIR

function install_function {
if (command -v checkinstall); then
   checkinstall -y
else
   make install
}  

#install cmake 
wget https://cmake.org/files/v3.10/cmake-3.10.1.tar.gz
tar xf cmake-3.10.1.tar.gz
cd cmake-3.10.1
./configure -DCMAKE_INSTALL_PREFIX:PATH=$INSTALL_DIR
make
install_function

#install boost, mostly headers
mkdir -p $BUILD_DIR/boost
cd $BUILD_DIR/boost 
$BUILD_DIR/../boost/bootstrap.sh --prefix=$INSTALL_DIR

#install eigen, mostly headers
mkdir -p $BUILD_DIR/eigen
cd $BUILD_DIR/eigen 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/eigen 
install_function

#install flann
mkdir -p $BUILD_DIR/flann
cd $BUILD_DIR/flann 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/flann 
make
install_function

#install openCV
mkdir -p $BUILD_DIR/opencv3
cd $BUILD_DIR/opencv3 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_FFMPEG=OFF -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/opencv3 
make
install_function

#install pcl
mkdir -p $BUILD_DIR/pcl
cd $BUILD_DIR/pcl 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/pcl 
make
install_function

#install qhull
mkdir -p $BUILD_DIR/qhull
cd $BUILD_DIR/qhull 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/qhull 
make
install_function

#install tinyxml2
mkdir -p $BUILD_DIR/tinyxml2
cd $BUILD_DIR/tinyxml2 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/tinyxml2 
install_function

#install vtk
mkdir -p $BUILD_DIR/vtk
cd $BUILD_DIR/vtk 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/vtk 
make
install_function

#install yamlcpp
mkdir -p $BUILD_DIR/yamlcpp
cd $BUILD_DIR/yamlcpp 
cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/yamlcpp 
make
install_function











