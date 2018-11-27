#!/usr/bin/env bash
#xma@spaceapplications.com
#This file is needed by ../fetch_compile_install_dependencies.sh
# Version 1.0

function install4infuse_eigen {
if [[ ! -n $(find $PKG_DIR -name 'eigen*') ]]; then
    cdff_gitclone eigen 3.3.4 https://github.com/eigenteam/eigen-git-mirror
    cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
        $SOURCE_DIR/eigen

    cdff_makeinstall eigen 3.3.4
    cdff_makedistclean eigen
fi
}
