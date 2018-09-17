function install4infuse_base_types {
if [[ ! -n $(find $PKG_DIR -name 'base_types*') ]]; then
	fetchgit_function base_types master https://github.com/rock-core/base-types.git

	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF -D CMAKE_CXX_FLAGS:STRING=-fPIC -D BUILD_SHARED_LIBS=ON \
	    -D BINDINGS_RUBY=OFF \
	    $SOURCE_DIR/base_types

	make --jobs=${CPUS}
	install_function base_types master
	clean_function base_types
fi
}
