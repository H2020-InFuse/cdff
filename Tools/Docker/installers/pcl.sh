#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

function install4infuse_pcl {
if [[ ! -n $(find $PKG_DIR -name 'pcl*') ]]; then
	fetchgit_function pcl pcl-1.8.1 https://github.com/PointCloudLibrary/pcl.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D BOOST_ROOT=$DIR/install -D VTK_DIR=$DIR/install -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $SOURCE_DIR/pcl
	make --jobs=${CPUS}
	install_function 1.8.1
	clean_function pcl
fi
}
