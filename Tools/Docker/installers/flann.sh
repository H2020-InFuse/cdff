#!/usr/bin/env bash
#xma@spaceapplications.com
#This file is needed by ../get-cdff-dependencies.sh
# Version 1.0

# Dependency: Eigen

function install4infuse_flann {
if [[ ! -n $(find $PKG_DIR -name 'flann*') ]]; then
	cdff_gitclone flann 1.9.1 https://github.com/mariusmuja/flann.git
	mkdir build
	cd build
	cmake \
		-D CMAKE_BUILD_TYPE=Release \
		-D BUILD_PYTHON_BINDINGS=OFF -D BUILD_MATLAB_BINDINGS=OFF \
		-D BUILD_TESTS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_DOC=OFF \
		-D CMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
		$SOURCE_DIR/flann

	make --jobs=${CPUS}
	cdff_makeinstall flann 1.9.1
	cdff_makedistclean flann
fi
}
