function install4infuse_envire_envire_core {
if [[ ! -n $(find $PKG_DIR -name 'envire_envire_core*') ]]; then
	fetchgit_function envire_envire_core master https://github.com/envire/envire-envire_core.git 24c1b03355e962cf3ec29ee9adb03942e2284a02

	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules" \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D ROCK_VIZ_ENABLED=OFF\
	    $SOURCE_DIR/envire_envire_core



	make --jobs=${CPUS}
	install_function envire_envire_core master
	clean_function envire_envire_core
fi
}
