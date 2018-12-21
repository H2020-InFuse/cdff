#!/usr/bin/env bash

# Dependency: console_bridge

function install4infuse_tools_plugin_manager {
if [[ ! -n $(find $PKG_DIR -name 'tools_plugin_manager*') ]]; then
	cdff_gitclone tools_plugin_manager master https://github.com/envire/tools-plugin_manager.git e31336046e22e8279b527a89a34290fa9cd16a6a
	mkdir build
	cd build
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules" \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/tools_plugin_manager

	make --jobs=${CPUS}
	cdff_makeinstall tools_plugin_manager master
	cdff_makedistclean tools_plugin_manager
fi
}
