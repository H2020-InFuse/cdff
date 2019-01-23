#!/bin/bash

if [ $# -eq 0 ]; then
    echo "No compilation database provided.
    run  cmake with -D CMAKE_EXPORT_COMPILE_COMMANDS=ON to create a database file named compile_commands.json.
    Then : #analysis-cppcheck.sh compile_commands.json"
    exit 1
fi

# Canonical path to the directory containing this script
# Uses GNU readlink from GNU coreutils
DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cppcheck --version

# How many processors?
if [ -f /proc/cpuinfo ]; then
  CPUS=$(grep --count --regexp=^processor /proc/cpuinfo)
else
  CPUS=1
fi
JOBS=$((CPUS/2))
JOBS=$((JOBS > 0 ? JOBS : 1))

mkdir -p cppcheck

compile_database="$1"

cppcheck --language=c++ -j $JOBS --suppressions-list=${DIR}/cppcheck_ignores.txt --force --enable=all --std=c++11 --project=$compile_database --xml --xml-version=2 2> cppcheck/cppcheck.xml
cppcheck-htmlreport --file=cppcheck/cppcheck.xml --report-dir=cppcheck/public

#count errors and put them in a separate file.
sed -n -e '/severity="error"/,/[</]error[>]/ p' cppcheck/cppcheck.xml > cppcheck/cppcheck_error.log

COUNT=$(wc -l < cppcheck/cppcheck_error.log )

if [ $COUNT -gt 0 ]; then
	echo "Error count is $(($COUNT/3))! cppcheck run failed :-(.";
	echo ""
	echo "Errors list:"
	cat cppcheck/cppcheck_error.log
	exit 1;
else
	echo "There is no error :-)."
fi

exit 0;
