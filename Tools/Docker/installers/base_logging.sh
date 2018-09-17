depends_tools_plugin_manager=base_cmake;

function install4infuse_base_logging {
if [[ ! -n $(find $PKG_DIR -name 'base_logging*') ]]; then
	fetchgit_function base_logging master https://github.com/rock-core/base-logging
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF \
	    $SOURCE_DIR/base_logging

	make --jobs=${CPUS}
	install_function base_logging master
	clean_function base_logging
fi
}
