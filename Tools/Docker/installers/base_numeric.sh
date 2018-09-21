function install4infuse_base_numeric {
if [[ ! -n $(find $PKG_DIR -name 'base_numeric*') ]]; then
	fetchgit_function base_numeric master https://github.com/envire/base-numeric.git


	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/base_numeric

	make --jobs=${CPUS}
	install_function base_numeric master
	clean_function base_numeric
fi
}
