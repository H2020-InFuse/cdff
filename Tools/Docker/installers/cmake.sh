#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_cmake=boost;

function install4infuse_cmake {
if [ ! -f "$INSTALL_DIR/bin/cmake" ]; then # should test for 3.10 version > installed
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
