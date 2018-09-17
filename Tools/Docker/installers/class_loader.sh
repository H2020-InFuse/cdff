depends_class_loader=console_bridge;

function install4infuse_class_loader {
if [[ ! -n $(find $PKG_DIR -name 'class_loader*') ]]; then
	fetchgit_function class_loader 0.3.7 https://github.com/ros/class_loader
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF \
	    $SOURCE_DIR/class_loader

	make --jobs=${CPUS}
	install_function class_loader 0.3.7
	clean_function class_loader
fi
}
