#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_opencv=flann;

function install4infuse_opencv {
if [[ ! -n $(find $PKG_DIR -name 'opencv*') ]]; then
	fetchgit_function opencv 3.4.0 https://github.com/opencv/opencv.git
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF -D CPU_BASELINE=SSE3 -D CPU_DISPATCH=SSE4_1,SSE4_2 \
	    $SOURCE_DIR/opencv

	make --jobs=${CPUS}
	install_function opencv 3.4.0
	clean_function opencv
fi
}
