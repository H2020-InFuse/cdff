function install4infuse_envire_envire_core {
if [[ ! -n $(find $PKG_DIR -name 'envire_envire_core*') ]]; then
	fetchgit_function envire_envire_core fix_cmake_console_bridge https://github.com/H2020-InFuse/envire-envire_core.git 

	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/envire_envire_core

	make --jobs=${CPUS}
	install_function envire_envire_core fix_cmake_console_bridge 
	clean_function envire_envire_core
fi
}