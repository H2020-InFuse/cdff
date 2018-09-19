echo PKG_CONFIG_PATH is $PKG_CONFIG_PATH
if [[ ! -n $(find $PKG_DIR -name 'base_boost_serialization*') ]]; then
	fetchgit_function base_boost_serialization master https://github.com/envire/base-boost_serialization.git

	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D CMAKE_CXX_FLAGS:STRING=-std=c++11 \
	    $SOURCE_DIR/base_boost_serialization

	make --jobs=${CPUS}
	install_function base_boost_serialization master
	clean_function base_boost_serialization
fi
}
