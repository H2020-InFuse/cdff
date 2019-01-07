#!/usr/bin/env bash
#xma@spaceapplications.com
#This file is needed by ../get-cdff-dependencies.sh
# Version 1.0

function install4infuse_opencv {
if [[ ! -n $(find $PKG_DIR -name 'opencv*') ]]; then
	cdff_gitclone opencv 3.4.0 https://github.com/opencv/opencv.git
	mkdir build
	cd build
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
  	    -D BUILD_LIST=calib3d,core,dnn,features2d,flann,highgui,imgcodecs,imgproc,ml,objdetect,photo,python2,python3,python_bindings_generator,shape,stitching,superres,ts,video,videoio,videostab,viz \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF -D CPU_BASELINE=SSE3 -D CPU_DISPATCH=SSE4_1,SSE4_2 \
	    $SOURCE_DIR/opencv

	make --jobs=${CPUS}
	cdff_makeinstall opencv 3.4.0
	cdff_makedistclean opencv
fi
}
