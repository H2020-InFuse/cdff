function install4infuse_base_types {

if [[ ! -n $(find $PKG_DIR -name 'base_types*') ]]; then
	fetchgit_function base_types master https://github.com/rock-core/base-types.git 70b7b2d78cdcae591866469f2a935d5e3779d302

	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules"
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D BINDINGS_RUBY=OFF \
	    $SOURCE_DIR/base_types

	make --jobs=${CPUS}
	install_function base_types master
	clean_function base_types
fi
}
