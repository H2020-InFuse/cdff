#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_boost=;

function install4infuse_boost {
if [[ ! -d "$INSTALL_DIR/include/boost" ]]; then
	# fetchgit_function boost boost-1.61.0 https://github.com/boostorg/boost.git
        fetchsource_function boost boost_1_66_0.tar.gz https://dl.bintray.com/boostorg/release/1.66.0/source/
	./bootstrap.sh
	./b2 install --prefix=$INSTALL_DIR
	clean_function boost
fi
}
