#!/usr/bin/env bash
#xma@spaceapplications.com
#This file is needed by ../get-cdff-dependencies.sh
# Version 1.0

function install4infuse_qhull {
if [[ ! -n $(find $PKG_DIR -name 'qhull*') ]]; then
    cdff_gitclone qhull master https://github.com/qhull/qhull.git
    cmake \
        -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
        $SOURCE_DIR/qhull

    make --jobs=${CPUS}
    cdff_makeinstall qhull 0.0.0
    cdff_makedistclean qhull
fi
}
