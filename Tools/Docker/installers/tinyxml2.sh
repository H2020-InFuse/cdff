#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

function install4infuse_tinyxml2 {
if [[ ! -n $(find $PKG_DIR -name 'tinyxml2*') ]]; then
	fetchgit_function tinyxml2 6.0.0 https://github.com/leethomason/tinyxml2.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_TESTING=OFF -D BUILD_TESTS=OFF $SOURCE_DIR/tinyxml2
	install_function 6.0.0
	clean_function tinyxml2
fi
}
