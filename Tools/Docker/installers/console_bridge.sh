#!/usr/bin/env bash

function install4infuse_console_bridge {
if [[ ! -n $(find $PKG_DIR -name 'console_bridge*') ]]; then
	cdff_gitclone console_bridge master https://github.com:/rock-core/base-console_bridge c947d8b8b8d755a8ca7161909950cbb1c74b5095
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/console_bridge

	make --jobs=${CPUS}
	cdff_makeinstall console_bridge master
	cdff_makedistclean console_bridge
fi
}
