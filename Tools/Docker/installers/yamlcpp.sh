#!/usr/bin/env bash
#xma@spaceapplications.com
#This file is needed by ../get-cdff-dependencies.sh
# Version 1.1

function install4infuse_yaml-cpp {
if [[ ! -n $(find $PKG_DIR -name 'yaml-cpp*') ]]; then

	cdff_gitclone yaml-cpp release-0.5.3 https://github.com/jbeder/yaml-cpp.git

	cmake \
		-D CMAKE_BUILD_TYPE=Release \
		-D BUILD_SHARED_LIBS=ON \
		-D CMAKE_MODULE_PATH="${INSTALL_DIR}/share/cmake-3.11.4/Modules" \
		-D CMAKE_PREFIX_PATH=../../install \
		-D CMAKE_INSTALL_PREFIX="${INSTALL_DIR}" \
		$SOURCE_DIR/yaml-cpp
	make --jobs=${CPUS}

	cdff_makeinstall yamlcpp 0.5.3

	cdff_makedistclean yaml-cpp
fi
}
