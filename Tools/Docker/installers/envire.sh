function install4infuse_envire_envire_core {
if [[ ! -n $(find $PKG_DIR -name 'envire_envire_core*') ]]; then
	fetchgit_function envire_envire_core master https://github.com/envire/envire-envire_core.git 6e5f5210c31be37386bbaeff68cf16378c909334

	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    $SOURCE_DIR/envire_envire_core

	make --jobs=${CPUS}
	install_function envire_envire_core master
	clean_function envire_envire_core
fi
}
