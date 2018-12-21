#!/usr/bin/env bash

function install4infuse_sisl {
if [[ ! -n $(find $PKG_DIR -name 'sisl*') ]]; then
	cdff_gitclone sisl master https://github.com/SINTEF-Geometry/SISL.git fc8e334ed0ee1c881c87745534d083254dcc63e8
	mkdir build
	cd build
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D CMAKE_CXX_FLAGS:STRING=-fPIC -D BUILD_SHARED_LIBS=ON \
	    $SOURCE_DIR/sisl

	make --jobs=${CPUS}
	cdff_makeinstall sisl master
	cdff_makedistclean sisl
fi
}
