#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

function install4infuse_qhull {
if [[ ! -n $(find $PKG_DIR -name 'qhull*') ]]; then
	fetchgit_function qhull master https://github.com/qhull/qhull.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/qhull $DIR/qhull
	make
	install_function 1.0
	clean_function qhull
fi
}
