#!usr/bin/env bash

function install4infuse_gdalwrap {
if [[ ! -n $(find $PKG_DIR -name 'gdalwrap*') ]]; then
  cdff_gitclone gdalwrap master https://github.com/pierriko/gdalwrap.git 
  cmake \
      -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=$INSTALL_DIR \
      $SOURCE_DIR/gdalwrap \

  make --jobs=${CPUS}

  cdff_makeinstall gdalwrap master
  cdff_makedistclean gdalwrap
fi
}
