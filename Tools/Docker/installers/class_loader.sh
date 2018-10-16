depends_class_loader=console_bridge;

function install4infuse_class_loader {
if [[ ! -n $(find $PKG_DIR -name 'class_loader*') ]]; then
	fetchgit_function class_loader 0.3.7 https://github.com/ros/class_loader
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules" \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D CMAKE_CXX_FLAGS:STRING=-fPIC -D BUILD_SHARED_LIBS=ON \
	    $SOURCE_DIR/class_loader

	make --jobs=${CPUS}
	install_function class_loader 0.3.7
	clean_function class_loader
fi
}
