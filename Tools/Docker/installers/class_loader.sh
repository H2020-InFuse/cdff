#!/usr/bin/env bash

# Dependency: console_bridge

function install4infuse_class_loader {
if [[ ! -n $(find $PKG_DIR -name 'class_loader*') ]]; then
	cdff_gitclone class_loader 0.3.7 https://github.com/ros/class_loader
	mkdir build
	cd build
	cmake \
	    -D CMAKE_BUILD_TYPE=RELEASE \
	    -D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules" \
	    -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
	    -D CMAKE_CXX_FLAGS:STRING=-fPIC -D BUILD_SHARED_LIBS=ON \
	    $SOURCE_DIR/class_loader

	make --jobs=${CPUS}
	cdff_makeinstall class_loader 0.3.7
	cdff_makedistclean class_loader
fi
}
