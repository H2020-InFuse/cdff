#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

function install4infuse_yaml-cpp {
if [[ ! -n $(find $PKG_DIR -name 'yaml-cpp*') ]]; then
	fetchgit_function yaml-cpp release-0.5.3 https://github.com/jbeder/yaml-cpp.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D BOOST_ROOT=$INSTALL_DIR -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $SOURCE_DIR/yaml-cpp
	make
	install_function 0.5.3
	clean_function yaml-cpp
fi
}
