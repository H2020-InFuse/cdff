#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_opencv=flann;

function install4infuse_opencv {
if [[ ! -n $(find $DIR/package/ -name 'opencv*') ]]; then
	fetchsource_function opencv 3.4.0 https://github.com/opencv/opencv.git
	cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_FFMPEG=OFF -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON -D WITH_IPP=OFF $DIR/opencv
	make
	install_function 3.4.0
	clean_function opencv
fi
}
