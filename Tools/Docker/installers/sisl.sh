function install4infuse_sisl {
if [[ ! -n $(find $PKG_DIR -name 'sisl*') ]]; then
	fetchgit_function sisl master https://github.com/SINTEF-Geometry/SISL.git fc8e334ed0ee1c881c87745534d083254dcc63e8
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D CMAKE_CXX_FLAGS:STRING=-fPIC -D BUILD_SHARED_LIBS=ON \
	    $SOURCE_DIR/sisl

	make --jobs=${CPUS}
	install_function sisl master
	clean_function sisl
fi
}
