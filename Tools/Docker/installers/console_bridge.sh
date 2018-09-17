
function install4infuse_console_bridge {
if [[ ! -n $(find $PKG_DIR -name 'console_bridge*') ]]; then
	fetchgit_function console_bridge master https://github.com:/rock-core/base-console_bridge
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF \
	    $SOURCE_DIR/console_bridge

	make --jobs=${CPUS}
	install_function console_bridge master
	clean_function console_bridge
fi
}
