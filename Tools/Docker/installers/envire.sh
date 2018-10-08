function install4infuse_envire_envire_core {
if [[ ! -n $(find $PKG_DIR -name 'envire_envire_core*') ]]; then
	fetchgit_function envire_envire_core qt4_optional https://github.com/H2020-InFuse/envire-envire_core.git 1fa7d253e2624dce2e058b5b0c0f95e07c6c9fa4

	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D AVOID_QT=True\
	    $SOURCE_DIR/envire_envire_core



	make --jobs=${CPUS}
	install_function envire_envire_core master
	clean_function envire_envire_core
fi
}
