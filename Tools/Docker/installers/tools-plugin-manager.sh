depends_tools_plugin_manager=console_bridge;

function install4infuse_tools_plugin_manager {
if [[ ! -n $(find $PKG_DIR -name 'tools_plugin_manager*') ]]; then
	fetchgit_function tools_plugin_manager fix_cmake_console_bridge https://github.com/H2020-InFuse/tools-plugin_manager.git
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/tools_plugin_manager

	make --jobs=${CPUS}
	install_function tools_plugin_manager fix_cmake_console_bridge
	clean_function tools_plugin_manager
fi
}
