#!/bin/bash

# assuming we are running on an Image with valgrind.
# assuming source contains already ASN types in c.

FULL=false
VALDIR=valgrind_release
for var in "$@"
do
   if [[ $var == "--full"  ]]
    then
    FULL=true
    VALDIR=valgrind_debug
   fi
done

DIR=$PWD
echo building in ${VALDIR}

mkdir -p ${VALDIR} && cd ${VALDIR}

# create the unit tests with debug symbols (-gdb3 for high level and better feedback; not portable! Else use -g2, which should work on all platforms)
CMAKE_FLAGS=""
if [[ "$FULL" == true ]]
    then
        CMAKE_FLAGS+="-DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_C_FLAGS_DEBUG:STRING=-ggdb3 -DCMAKE_CXX_FLAGS_DEBUG:STRING=-ggdb3"
    else
        CMAKE_FLAGS+="-DCMAKE_BUILD_TYPE:STRING=Release"
fi

cmake ${CMAKE_FLAGS} ..

make
mkdir -p results

# run Valgrind
# with these parameters, a full check will be performed, outputting the leaks / errors to an xml file that can be used with Valkyrie to parse them in a user-friendly way
# --track-origins=yes also traces back to where an uninitialised variable is first defined, to make fixing the "[UNC] Conditional jump or move depends on uninitialised value(s)" errors easier to resolve
# --error-exitcode=1 finished with this exit code on error at the end (in case of any errors), instead of the result of the called program exit code (which is tested for in a different job anyway!)
# --child-silent-after-fork=yes to not mangle the xml output
# -q to be less verbose and only output valgrind errors (not program errors/leaks) in the valgrind.log file. The file should therefore usually be completely empty.
# --fullpath-after=InFuse/ supposedly cuts the path reference for source files from this reference frame. Though with xml output this does not seem to do anything.


cd Tests/UnitTests/
FLAGS="--leak-check=full --show-leak-kinds=all --error-exitcode=1 --child-silent-after-fork=yes --xml=yes --xml-file=${DIR}/${VALDIR}/results/error.xml --log-file=${DIR}/${VALDIR}/results/valgrind.log --fullpath-after=CDFF/ --suppressions=${DIR}/Tools/Valgrind/suppression.txt"

if [[ "$FULL" == true ]]
    then
        FLAGS+=" --track-origins=yes"
    else
        FLAGS+=" --track-origins=no --undef-value-errors=no"
fi

echo RUNNING valgrind: ${FLAGS}
valgrind ${FLAGS} ./cdff-unit-tests

# print summary for a nice view in gitlab
awk '/== HEAP SUMMARY/,/== $/' ${DIR}/${VALDIR}/results/valgrind.log;
awk '/== LEAK SUMMARY/,/== $/' ${DIR}/${VALDIR}/results/valgrind.log;
awk '/== ERROR SUMMARY/,/== $/' ${DIR}/${VALDIR}/results/valgrind.log;