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
	install_yamlcpp
fi
if ( $vtk ); then
	install_vtk
fi
if ( $opencv ); then
	install_opencv
fi
if ( $pcl ); then
	install_pcl
fi
}

#update submodules
#git submodule update --remote --recursive --depth 1

# if exist remove previous packages

function install_function {
if (command -v checkinstall); then
   checkinstall -y --pakdir $PKG_DIR --nodoc
else
   make install
fi
}  

function install_cmake {
if [ ! -d "$INSTALL_DIR" ]; then # should test for 3.10 version > installed
	echo "Installing CMake"
	wget https://cmake.org/files/v3.10/cmake-3.10.1.tar.gz
	tar xf cmake-3.10.1.tar.gz
	cd cmake-3.10.1
	./bootstrap --prefix=$INSTALL_DIR 
	make
	install_function
	echo "CMake installation Done."
fi
}

function install_boost {
if [[ ! -d "$INSTALL_DIR/include/boost" ]]; then
	echo "Installing boost"        
	mkdir -p $BUILD_DIR/boost
	cd $DIR/boost 
	echo "boost boostrap"
	./bootstrap.sh --prefix=$INSTALL_DIR 
	echo "b2 install"
	./b2 --build-dir=$BUILD_DIR/boost --prefix=$INSTALL_DIR --NO_COMPRESSION --without-python install
	echo "clean-all"
	./b2 --clean-all
	echo "boost installation Done."
fi
}

function install_eigen {
if [[ ! -n $(find $DIR/package/ -name 'eigen*') ]]; then
	echo "Installing eigen"
	mkdir -p $BUILD_DIR/eigen
	cd $BUILD_DIR/eigen 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/eigen 
	install_function
	echo "eigen installation Done."
fi
}

function install_flann {
if [[ ! -n $(find $DIR/package/ -name 'flann*') ]]; then
	echo "Installing flann"
	mkdir -p $BUILD_DIR/flann
	cd $BUILD_DIR/flann 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_TESTS=OFF -D BUILD_EXAMPLES=OFF $DIR/flann
	make
	install_function
	echo "flann installation Done."
fi
}

function install_opencv {
if [[ ! -n $(find $DIR/package/ -name 'opencv*') ]]; then
	echo "Installing opencv"
	mkdir -p $BUILD_DIR/opencv3
	cd $BUILD_DIR/opencv3
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_FFMPEG=OFF -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON $DIR/opencv3
	make
	install_function
	echo "opencv installation Done."
fi
}

function install_pcl {
if [[ ! -n $(find $DIR/package/ -name 'pcl*') ]]; then
	echo "Installing pcl"
	mkdir -p $BUILD_DIR/pcl
	cd $BUILD_DIR/pcl 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/pcl 
	make
	install_function
	echo "pcl installation Done."
fi
}

function install_qhull {
if [[ ! -n $(find $DIR/package/ -name 'qhull*') ]]; then
	echo "Installing qhull"
	mkdir -p $BUILD_DIR/qhull
	cd $BUILD_DIR/qhull 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/qhull 
	make
	install_function
	echo "qhull installation Done."
fi
}

function install_tinyxml2 {
if [[ ! -n $(find $DIR/package/ -name 'tinyxml2*') ]]; then
	echo "Installing tinyxml2"
	mkdir -p $BUILD_DIR/tinyxml2
	cd $BUILD_DIR/tinyxml2 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_TESTING=OFF -D BUILD_TESTS=OFF $DIR/tinyxml2
	install_function
	echo "tinyxml2 installation Done."
fi
}

function install_vtk {
if [[ ! -n $(find $DIR/package/ -name 'vtk*') ]]; then
	echo "Installing vtk"
	mkdir -p $BUILD_DIR/vtk
	cd $BUILD_DIR/vtk 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_TESTING=OFF -D BUILD_EXAMPLES=OFF -D BUILD_DOCUMENTATION=OFF -D VTK_Group_Rendering=OFF $DIR/vtk
	make
	install_function
	echo "vtk installation Done."
fi
}

function install_yamlcpp {
if [[ ! -n $(find $DIR/package/ -name 'yamlcpp*') ]]; then
	echo "Installing yamlcpp"
	mkdir -p $BUILD_DIR/yamlcpp
	cd $BUILD_DIR/yamlcpp 
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/yamlcpp 
	make
	install_function
	echo "yamlcpp installation Done."
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




