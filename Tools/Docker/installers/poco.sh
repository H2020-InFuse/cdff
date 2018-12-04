#!/usr/bin/env bash

function install4infuse_poco {
if [[ ! -n $(find $PKG_DIR -name 'poco*') ]]; then
	cdff_gitclone poco poco-1.9.0-release https://github.com/pocoproject/poco
	mkdir BUILD # build in lower-case is an existing directory in Poco's source
	cd BUILD
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/poco

	make --jobs=${CPUS}
	cdff_makeinstall poco poco-1.9.0-release
	cdff_makedistclean poco
fi
}
