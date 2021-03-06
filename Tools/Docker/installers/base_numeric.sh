#!/usr/bin/env bash

function install4infuse_base_numeric {
if [[ ! -n $(find $PKG_DIR -name 'base_numeric*') ]]; then
	cdff_gitclone base_numeric master https://github.com/envire/base-numeric.git f4e12bf3b9e10b3f5bb56ac58f7478bd2590b990
	mkdir build
	cd build
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/base_numeric

	make --jobs=${CPUS}
	cdff_makeinstall base_numeric master
	cdff_makedistclean base_numeric
fi
}
