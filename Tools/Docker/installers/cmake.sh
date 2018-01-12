#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_cmake=boost;

function install4infuse_cmake {
if [ ! -f "$INSTALL_DIR/bin/cmake" ]; then # should test for 3.10 version > installed
	fetchsource_function cmake cmake-3.10.1.tar.gz https://cmake.org/files/v3.10/
	./bootstrap --prefix=$INSTALL_DIR
	make
	install_function 3.10.1
	clean_function cmake
fi
}
