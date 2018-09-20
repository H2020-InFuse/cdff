# Depends on class_loader
#depends_tools_plugin_manager=class_loader;
#depends_tools_plugin_manager=base_logging;
depends_tools_plugin_manager=console_bridge;

function install4infuse_tools_plugin_manager {
if [[ ! -n $(find $PKG_DIR -name 'tools_plugin_manager*') ]]; then
	fetchgit_function tools_plugin_manager fix_cmake_console_bridge https://github.com/H2020-InFuse/tools-plugin_manager.git
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D WITH_FFMPEG=OFF -D BUILD_DOCS=OFF -D BUILD_EXAMPLES=OFF \
	    -D BUILD_TESTS=OFF -D ENABLE_CXX11=ON -D ENABLE_FAST_MATH=ON \
	    -D WITH_IPP=OFF \
	    $SOURCE_DIR/tools_plugin_manager

	make --jobs=${CPUS}
	install_function tools_plugin_manager fix_cmake_console_bridge
	clean_function tools_plugin_manager
fi
}
