#!/bin/bash

# find changed files between this branch and master
ChangedFiles=$(git diff --name-only master . )
SourceFiles=()
for f in $ChangedFiles; do
  if [[ $f =~ \.(c|cpp|h|cc)$ ]]; then
   SourceFiles+=" $f" 
  fi
done
echo $SourceFiles

# Canonical path to the directory containing this script
# Uses GNU readlink from GNU coreutils
DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

cppcheck --version

mkdir -p cppcheck

cppcheck --language=c++ --suppressions-list=${DIR}/cppcheck_ignores.txt --force --enable=all --std=c++11 $SourceFiles 2> cppcheck/cppcheck_partial.log


echo Branch is $(git branch | grep \* | cut -d ' ' -f2)
#replace '/' with '-'
branch=$(git branch | grep \* | cut -d ' ' -f2 | sed -e 's/\//-/g')

cat cppcheck/cppcheck_partial.log | grep "(error)"          > cppcheck/cppcheck_error.log
cat cppcheck/cppcheck_partial.log | grep "(warning)"        > cppcheck/cppcheck_warning.log
cat cppcheck/cppcheck_partial.log | grep "(performance)"    > cppcheck/cppcheck_performance.log
cat cppcheck/cppcheck_partial.log | grep "(style)"          > cppcheck/cppcheck_style.log
cat cppcheck/cppcheck_partial.log | grep "(portability)"    > cppcheck/cppcheck_portability.log
cat cppcheck/cppcheck_partial.log | grep "(information)"    > cppcheck/cppcheck_information.log
cat cppcheck/cppcheck_partial.log | grep "(unusedFunction)" > cppcheck/cppcheck_unusedFunction.log
cat cppcheck/cppcheck_partial.log | grep "(missingInclude)" > cppcheck/cppcheck_missingInclude.log

#count errors and put them in a separate file.
COUNT=$(wc -l < cppcheck/cppcheck_error.log )

if [ $COUNT -gt 0 ]; then
        echo "Error count is $COUNT ! cppcheck run failed :-(.";
        echo ""
        echo "Errors list:"
        cat cppcheck/cppcheck_error.log
        exit 1;
else
        echo "There is no error :-)."
fi

COUNTW=$(wc -l < cppcheck/cppcheck_warning.log )
if [ $COUNTW -gt 0 ]; then
        echo "Warning count is $COUNTW.";
        echo ""
        echo "Warning list:"
        cat cppcheck/cppcheck_warning.log
else
        echo "There is no warnings :-)."
fi

exit 0;

