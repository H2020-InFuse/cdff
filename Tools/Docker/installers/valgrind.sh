#!/usr/bin/env bash

# Dependency: console_bridge

function install4infuse_valgrind {
if [[ ! -n $(find $PKG_DIR -name 'valgrind*') ]]; then
	cdff_gitclone valgrind master git://sourceware.org/git/valgrind.git
	./autogen.sh
	./configure
	make --jobs=${CPUS}
	cdff_makeinstall valgrind master
	cdff_makedistclean valgrind
fi
}