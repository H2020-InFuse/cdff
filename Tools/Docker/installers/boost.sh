#!/bin/bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

depends_boost=;

function install4infuse_boost {
if [[ ! -d "$INSTALL_DIR/include/boost" ]]; then
	fetchgit_function boost boost-1.61.0 https://github.com/boostorg/boost.git
  cd $DIR/boost
	echo "boost boostrap"
	./bootstrap.sh --prefix=$INSTALL_DIR  --show-libraries
	./b2 install --prefix=$INSTALL_DIR --NO_COMPRESSION --without-python --stagedir=$INSTALL_DIR
	./b2 headers
	clean_function boost
fi
}
