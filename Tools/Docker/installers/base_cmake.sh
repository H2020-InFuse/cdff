#!/usr/bin/env bash

function install4infuse_base_cmake {
if [[ ! -n $(find $PKG_DIR -name 'base_cmake*') ]]; then
	cdff_gitclone base_cmake master https://github.com/rock-core/base-cmake a1703a0b30dcc0380a5be147ea2ee1ca89fa25b3
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/base_cmake

	make --jobs=${CPUS}
	cdff_makeinstall base_cmake master
	cdff_makedistclean base_cmake
fi
}
