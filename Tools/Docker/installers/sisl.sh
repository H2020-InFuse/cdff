function install4infuse_sisl {
if [[ ! -n $(find $PKG_DIR -name 'sisl*') ]]; then
	fetchgit_function sisl master https://github.com/SINTEF-Geometry/SISL.git
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF -D CMAKE_CXX_FLAGS:STRING=-fPIC -D BUILD_SHARED_LIBS=ON \
	    $SOURCE_DIR/sisl

	make --jobs=${CPUS}
	install_function sisl master
	clean_function sisl
fi
}
