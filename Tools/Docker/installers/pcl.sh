#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

function install4infuse_pcl {
if [[ ! -n $(find $DIR/package/ -name 'pcl*') ]]; then
	fetchsource_function opencv pcl-1.8.1 https://github.com/PointCloudLibrary/pcl.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D BUILD_visualization=OFF -D BOOST_ROOT=$DIR/install -D VTK_DIR=$DIR/install -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR $DIR/pcl
	make
	install_function 1.8.1
	clean_function pcl
fi
}
