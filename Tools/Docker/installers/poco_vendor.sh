#!/usr/bin/env bash

# Dependency: poco

function install4infuse_poco_vendor {
if [[ ! -n $(find $PKG_DIR -name 'poco_vendor*') ]]; then
	cdff_gitclone poco_vendor 1.1.1 https://github.com/ros2/poco_vendor
	mkdir build
	cd build
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/poco_vendor

	make --jobs=${CPUS}
	cdff_makeinstall poco_vendor 1.1.1
	cdff_makedistclean poco_vendor
fi
}
