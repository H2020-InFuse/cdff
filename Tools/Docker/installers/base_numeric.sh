function install4infuse_base_numeric {
if [[ ! -n $(find $PKG_DIR -name 'base_numeric*') ]]; then
	fetchgit_function base_numeric master https://github.com/envire/base-numeric.git


	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    $SOURCE_DIR/base_numeric

	make --jobs=${CPUS}
	install_function base_numeric master
	clean_function base_numeric
fi
}
