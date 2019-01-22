#!/bin/bash

# install autoconf (needed for Valgrind)
sudo apt-get install autoconf

# install Valgrind from source (the apt version has a bug in it on Ubuntu 16.04)
git clone git://sourceware.org/git/valgrind.git
cd valgrind/
mkdir install
./autogen.sh
./configure --prefix=./install/
make
make install

# check Valgrind install
./install/valgrind --version

# in case Valgrind is not working as it should, install the apt version alongside it, to have the right settings / dependencies, but use the compiled version path ;-)

# go to the CDFF dir
cd ../CDFF

# from the CDFF dir, create the dir that will contain the built/compiled code (assuming no build folder)
mkdir build
cd build

# if there was a build folder, reset the CMakeCache, just to make sure nothing is preconfigured
rm CMakeCache.txt

# maybe re-use the ASN1 types from a different job?
# create the unit tests with debug symbols (-gdb3 for high level and better feedback; not portable! Else use -g2, which should work on all platforms)
cmake -D USE_BUNDLED_DEPENDENCIES=ON -D COMPILE_ASN1=OFF -D CMAKE_INSTALL_PREFIX=../install/ -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_C_FLAGS_DEBUG:STRING=-ggdb3 -DCMAKE_CXX_FLAGS_DEBUG:STRING=-ggdb3 ..

# cd to where the UnitTests need to be executed
cd ./Tests/UnitTests/

# create directory to hold the the log/xml output from Valgrind (and to create the artifacts from)
mkdir leak_check_result

# run Valgrind
# with these parameters, a full check will be performed, outputting the leaks / errors to an xml file that can be used with Valkyrie to parse them in a user-friendly way
# --track-origins=yes also traces back to where an uninitialised variable is first defined, to make fixing the "[UNC] Conditional jump or move depends on uninitialised value(s)" errors easier to resolve
# --error-exitcode=1 finished with this exit code on error at the end (in case of any errors), instead of the result of the called program exit code (which is tested for in a different job anyway!)
# --child-silent-after-fork=yes to not mangle the xml output
# -q to be less verbose and only output valgrind errors (not program errors/leaks) in the valgrind.log file. The file should therefore usually be completely empty.
# --fullpath-after=InFuse/ supposedly cuts the path reference for source files from this reference frame. Though with xml output this does not seem to do anything.
../../../../valgrind/install/bin/valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --error-exitcode=1 --child-silent-after-fork=yes -q --xml=yes --xml-file=leak_check_result/error.xml --log-file=leak_check_result/valgrind.log --fullpath-after=InFuse/ ./cdff-unit-tests

# print summary for a nice view in gitlab
cd leak_check_result
awk '/== HEAP SUMMARY/,/== $/' ./valgrind.log; awk '/== LEAK SUMMARY/,/== $/' ./valgrind.log; awk '/== ERROR SUMMARY/,/== $/' ./valgrind.log
