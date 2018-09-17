function install4infuse_base_cmake {
if [[ ! -n $(find $PKG_DIR -name 'base_cmake*') ]]; then
	fetchgit_function base_cmake master https://github.com/rock-core/base-cmake
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF \
	    $SOURCE_DIR/base_cmake

	make --jobs=${CPUS}
	install_function base_cmake master
	clean_function base_cmake
fi
}
