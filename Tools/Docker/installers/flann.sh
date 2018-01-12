#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_flann=eigen;

function install4infuse_flann {
if [[ ! -n $(find $DIR/package/ -name 'flann*') ]]; then
	fetchsource_function flann 1.9.1 https://github.com/mariusmuja/flann.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_TESTS=OFF -D BUILD_EXAMPLES=OFF $DIR/flann
	make
	install_function 1.9.1
	clean_function flann
fi
}
