#!/usr/bin/env bash

# Dependency: console_bridge

function install4infuse_tools_plugin_manager {
if [[ ! -n $(find $PKG_DIR -name 'tools_plugin_manager*') ]]; then
	fetchgit_function tools_plugin_manager master https://github.com/envire/tools-plugin_manager.git e31336046e22e8279b527a89a34290fa9cd16a6a
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules" \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/tools_plugin_manager

	make --jobs=${CPUS}
	install_function tools_plugin_manager master
	clean_function tools_plugin_manager
fi
}
