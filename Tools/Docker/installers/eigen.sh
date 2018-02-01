#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_eigen=cmake;

function install4infuse_eigen {
if [[ ! -n $(find $DIR/package/ -name 'eigen*') ]]; then
	fetchgit_function eigen 3.3.4 https://github.com/RLovelett/eigen.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/eigen
	install_function 3.3.4
	clean_function eigen
fi
}