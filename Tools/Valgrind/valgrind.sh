#!/bin/bash

# assuming we are running on an Image with valgrind.
# assuming source contains already ASN types in c.

FULL=false
for var in "$@"
do
   if [[ $var == "--full"  ]]
    then
    FULL=true
   fi
done


mkdir -p valgrind && cd valgrind

# create the unit tests with debug symbols (-gdb3 for high level and better feedback; not portable! Else use -g2, which should work on all platforms)
cmake -DCMAKE_BUILD_TYPE:STRING=Debug -DCMAKE_C_FLAGS_DEBUG:STRING=-ggdb3 -DCMAKE_CXX_FLAGS_DEBUG:STRING=-ggdb3 ..

make

mkdir results

# run Valgrind
# with these parameters, a full check will be performed, outputting the leaks / errors to an xml file that can be used with Valkyrie to parse them in a user-friendly way
# --track-origins=yes also traces back to where an uninitialised variable is first defined, to make fixing the "[UNC] Conditional jump or move depends on uninitialised value(s)" errors easier to resolve
# --error-exitcode=1 finished with this exit code on error at the end (in case of any errors), instead of the result of the called program exit code (which is tested for in a different job anyway!)
# --child-silent-after-fork=yes to not mangle the xml output
# -q to be less verbose and only output valgrind errors (not program errors/leaks) in the valgrind.log file. The file should therefore usually be completely empty.
# --fullpath-after=InFuse/ supposedly cuts the path reference for source files from this reference frame. Though with xml output this does not seem to do anything.

if [[ "$FULL" == true ]]
    then
        valgrind --leak-check=full --show-leak-kinds=all --track-origins=yes --error-exitcode=1 --child-silent-after-fork=yes -q --xml=yes --xml-file=results/error.xml --log-file=results/valgrind.log --fullpath-after=CDFF/ ./Tests/UnitTests/cdff-unit-tests
    else
        valgrind --leak-check=full --show-leak-kinds=all --track-origins=no --undef-value-errors=no --error-exitcode=1 --child-silent-after-fork=yes -q --xml=yes --xml-file=results/error.xml --log-file=results/valgrind.log --fullpath-after=CDFF/ ./Tests/UnitTests/cdff-unit-tests
fi

# print summary for a nice view in gitlab

awk '/== HEAP SUMMARY/,/== $/' results/valgrind.log;
awk '/== LEAK SUMMARY/,/== $/' results/valgrind.log;
awk '/== ERROR SUMMARY/,/== $/' results/valgrind.log;
