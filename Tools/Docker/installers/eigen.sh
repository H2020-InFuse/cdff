#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_eigen=cmake;

function install4infuse_eigen {
if [[ ! -n $(find $PKG_DIR -name 'eigen*') ]]; then
	fetchgit_function eigen 3.3.4 https://github.com/eigenteam/eigen-git-mirror.git
	mv $SOURCE_DIR/eigen-git-mirror $SOURCE_DIR/eigen
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $SOURCE_DIR/eigen
	install_function 3.3.4
	clean_function eigen
fi
}
