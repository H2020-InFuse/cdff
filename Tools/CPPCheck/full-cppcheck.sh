#!/bin/bash

#set -eax

# Maintainers: xma@spaceapplications.com

# Canonical path to the directory containing this script
# Uses GNU readlink from GNU coreutils
DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
IGNORE_FILE=${3:-"${DIR}/cppcheck_ignores.txt"}
OUTPUT_FOLDER=${2:-"${DIR}/report"}
DATABASE_FILE=${1:-"${DIR}/build/compile_commands.json"}

cppcheck --version
# How many processors?
if [ -f /proc/cpuinfo ]; then
  CPUS=$(grep --count --regexp=^processor /proc/cpuinfo)
else
  CPUS=1
fi
JOBS=$((CPUS/2))
JOBS=$((JOBS > 0 ? JOBS : 1))

mkdir -p ${OUTPUT_FOLDER}

cppcheck --language=c++ -j $JOBS --suppressions-list=${IGNORE_FILE} --force --enable=all --std=c++11 --project=${DATABASE_FILE} --xml --xml-version=2 2> ${OUTPUT_FOLDER}/cppcheck.xml

#count errors and put them in a separate file.
sed -n -e '/severity="error"/,/[</]error[>]/ p' ${OUTPUT_FOLDER}/cppcheck.xml > ${OUTPUT_FOLDER}/cppcheck_error.log
COUNT=$(wc -l < ${OUTPUT_FOLDER}/cppcheck_error.log )

if [ $COUNT -gt 0 ]; then
	echo "Error count is $(($COUNT/3))! cppcheck run failed :-(.";
	echo ""
	echo "Errors list:"
	cat ${OUTPUT_FOLDER}/cppcheck_error.log
	exit 1;
else
	echo "There is no error :-)."
fi
