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
cmake=false
boost=false
eigen=false
flann=false
qhull=false
tinyxml2=false
yamlcpp=false
vtk=false
opencv=false
pcl=false

# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.

function show_help {

  cat <<EOF
\`./bootstrap.sh' prepares Boost for building on a few kinds of systems.

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

EOF
}

function show_configuration {
if ($BUILD_ALL) ;then
	cmake=true
	boost=true
	eigen=true
	flann=true
	qhull=true
	tinyxml2=true
	yamlcpp=true
	vtk=true
	opencv=true
	pcl=true
fi
	echo "build directory    = ${BUILD_DIR}"
	echo "install directory  = ${INSTALL_DIR}"
	echo "Packages directory = ${PKG_DIR}"
	echo cmake=$cmake
	echo boost=$boost
	echo eigen=$eigen
	echo flann=$flann
	echo qhull=$qhull
	echo tinyxml2=$tinyxml2
	echo yamlcpp=$yamlcpp
	echo vtk=$vtk
	echo opencv=$opencv
	echo pcl=$pcl
}

function build {
mkdir -p $BUILD_DIR
cd $BUILD_DIR

# add package config path if not present
[[ ":$PKG_CONFIG_PATH:" != *":$INSTALL_DIR/lib/pkgconfig:"* ]] && PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:$INSTALL_DIR/lib/pkgconfig"
# add bin install dir to path if not present
[[ ":$PATH:" != *"$INSTALL_DIR/bin"* ]] && PATH="${PATH}:$INSTALL_DIR/bin"
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH
export PATH=$PATH

if ( $cmake ); then
	install_cmake
fi
if ( $boost ); then
	install_boost
fi
if ( $eigen ); then
	install_eigen
fi
if ( $flann ); then
	install_flann
fi
if ( $qhull ); then
	install_qhull
fi
if ( $tinyxml2 ); then
	install_tinyxml2
fi
if ( $yamlcpp ); then
	install_yaml-cpp
fi
if ( $opencv ); then
	install_opencv
fi
if ( $vtk ); then
	install_vtk
fi
if ( $pcl ); then
	install_pcl
fi
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

function install_cmake {
if [ ! -d "$INSTALL_DIR" ]; then # should test for 3.10 version > installed
	echo "Installing CMake"
	mkdir -p $BUILD_DIR/cmake
	cd $BUILD_DIR/cmake
	wget https://cmake.org/files/v3.10/cmake-3.10.1.tar.gz
	tar xf cmake-3.10.1.tar.gz
	rm cmake-3.10.1.tar.gz
	cd cmake-3.10.1
	./bootstrap --prefix=$INSTALL_DIR 
	make
	install_function 3.10.1
	clean_function cmake
	echo "CMake installation Done."
fi
}

function install_boost {
if [[ ! -d "$INSTALL_DIR/include/boost" ]]; then
	echo "Installing boost"   
	mkdir -p $BUILD_DIR/boost
	cd $BUILD_DIR/boost
	git clone --depth 1 --recursive --single-branch -b boost-1.61.0 https://github.com/boostorg/boost.git
	cd boost 
	echo "boost boostrap"
	./bootstrap.sh --prefix=$INSTALL_DIR  --show-libraries
	./b2 install --prefix=$INSTALL_DIR --NO_COMPRESSION --without-python --stagedir=$INSTALL_DIR 
	./b2 headers
	clean_function boost
	echo "boost installation Done."
fi
}

function install_eigen {
if [[ ! -n $(find $DIR/package/ -name 'eigen*') ]]; then
	fetchsource_function eigen 3.3.4 https://github.com/RLovelett/eigen.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/eigen 
	install_function 3.3.4
	clean_function eigen
fi
}

function install_flann {
if [[ ! -n $(find $DIR/package/ -name 'flann*') ]]; then
	fetchsource_function flann 1.9.1 https://github.com/mariusmuja/flann.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_TESTS=OFF -D BUILD_EXAMPLES=OFF $DIR/flann
	make
	install_function 1.9.1
	clean_function flann
fi
}

function install_qhull {
if [[ ! -n $(find $DIR/package/ -name 'qhull*') ]]; then
	fetchsource_function qhull master https://github.com/qhull/qhull.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/qhull $DIR/qhull
	make
	install_function 1.0
	clean_function qhull
fi
}

function install_tinyxml2 {
if [[ ! -n $(find $DIR/package/ -name 'tinyxml2*') ]]; then
	fetchsource_function tinyxml2 6.0.0 https://github.com/leethomason/tinyxml2.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_TESTING=OFF -D BUILD_TESTS=OFF $DIR/tinyxml2
	install_function 6.0.0
	clean_function tinyxml2
fi
}

function install_yaml-cpp {
if [[ ! -n $(find $DIR/package/ -name 'yaml-cpp*') ]]; then
	fetchsource_function yaml-cpp release-0.5.3 https://github.com/jbeder/yaml-cpp.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D BOOST_ROOT=$DIR/install -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/yaml-cpp
	make
	install_function 0.5.3
	clean_function yaml-cpp
fi
}

function install_vtk {
if [[ ! -n $(find $DIR/package/ -name 'vtk*') ]]; then
	fetchsource_function vtk v8.1.0 https://github.com/Kitware/VTK.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_TESTING=OFF -D BUILD_EXAMPLES=OFF -D BUILD_DOCUMENTATION=OFF $DIR/vtk
	make
	install_function 8.1.0
	clean_function vtk
fi
}

function install_opencv {
if [[ ! -n $(find $DIR/package/ -name 'opencv*') ]]; then
	fetchsource_function opencv 3.4.0 https://github.com/opencv/opencv.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_FFMPEG=OFF -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON -D WITH_IPP=OFF $DIR/opencv
	make
	install_function 3.4.0
	clean_function opencv
fi
}

function install_pcl {
if [[ ! -n $(find $DIR/package/ -name 'pcl*') ]]; then
	fetchsource_function opencv pcl-1.8.1 https://github.com/PointCloudLibrary/pcl.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_visualization=OFF -D BOOST_ROOT=$DIR/install -D VTK_DIR=$DIR/install -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/pcl
	make
	install_function 1.8.1
	clean_function pcl
fi
}

BUILD_ALL=true

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
	BUILD_ALL=false
	if [ "cmake" == "$OPTARG" ]; then
		cmake=true
	fi
	if [ "boost" == "$OPTARG" ]; then
		boost=true
	fi
	if [ "eigen" == "$OPTARG" ]; then
		eigen=true
	fi
	if [ "flann" == "$OPTARG" ]; then
		flann=true
	fi
	if [ "qhull" == "$OPTARG" ]; then
		qhull=true
	fi
	if [ "tinyxml2" == "$OPTARG" ]; then
		tinyxml2=true
	fi
	if [ "yamlcpp" == "$OPTARG" ]; then
		yamlcpp=true
	fi
	if [ "vtk" == "$OPTARG" ]; then
		vtk=true
	fi
	if [ "opencv" == "$OPTARG" ]; then
		opencv=true
	fi
	if [ "pcl" == "$OPTARG" ]; then
		pcl=true
	fi
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
build




