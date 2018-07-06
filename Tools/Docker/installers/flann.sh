#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_flann=eigen;

function install4infuse_flann {
if [[ ! -n $(find $PKG_DIR -name 'flann*') ]]; then
	fetchgit_function flann 1.9.1 https://github.com/mariusmuja/flann.git
	cmake \
		-D CMAKE_BUILD_TYPE=Release \
		-D BUILD_PYTHON_BINDINGS=OFF -D BUILD_MATLAB_BINDINGS=OFF \
		-D BUILD_TESTS=OFF -D BUILD_EXAMPLES=OFF -D BUILD_DOC=OFF \
		-D CMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
		$SOURCE_DIR/flann

	make --jobs=${CPUS}
	install_function flann 1.9.1
	clean_function flann
fi
}
