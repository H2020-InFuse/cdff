# Dependency: poco

function install4infuse_poco_vendor {
if [[ ! -n $(find $PKG_DIR -name 'poco_vendor*') ]]; then
	fetchgit_function poco_vendor 1.1.1 https://github.com/ros2/poco_vendor
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/poco_vendor

	make --jobs=${CPUS}
	install_function poco_vendor 1.1.1
	clean_function poco_vendor
fi
}
