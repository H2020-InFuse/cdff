#!/usr/bin/env bash

function install4infuse_poco {
if [[ ! -n $(find $PKG_DIR -name 'poco*') ]]; then
	fetchgit_function poco poco-1.9.0-release https://github.com/pocoproject/poco
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/poco

	make --jobs=${CPUS}
	install_function poco poco-1.9.0-release
	clean_function poco
fi
}
