#!/bin/bash
set -x

cppcheck --version

mkdir cppcheck

cppcheck --language=c++ --suppressions-list=CppCheckSuppressions.txt --force --enable=all --std=c++11 --project=compile_commands.json --xml --xml-version=2 2> cppcheck/cppcheck.xml
cppcheck --language=c++ --suppressions-list=CppCheckSuppressions.txt --force --enable=all --std=c++11 --project=compile_commands.json 2> cppcheck/cppcheck_all.log

cat cppcheck/cppcheck_all.log | grep "(error)"          > cppcheck/cppcheck_error.log
cat cppcheck/cppcheck_all.log | grep "(warning)"        > cppcheck/cppcheck_warning.log
cat cppcheck/cppcheck_all.log | grep "(performance)"    > cppcheck/cppcheck_performance.log
cat cppcheck/cppcheck_all.log | grep "(style)"          > cppcheck/cppcheck_style.log
cat cppcheck/cppcheck_all.log | grep "(portability)"    > cppcheck/cppcheck_portability.log
cat cppcheck/cppcheck_all.log | grep "(information)"    > cppcheck/cppcheck_information.log
cat cppcheck/cppcheck_all.log | grep "(unusedFunction)" > cppcheck/cppcheck_unusedFunction.log
cat cppcheck/cppcheck_all.log | grep "(missingInclude)" > cppcheck/cppcheck_missingInclude.log

COUNT=$(wc -l < cppcheck/cppcheck_error.log )

if [ $COUNT -gt 0 ]; then
	echo "Error count is $COUNT! cppcheck run failed :-(.";
	echo ""
	echo "Errors list:"
	cat cppcheck/cppcheck_error.log
	exit 1;
else
	echo "There is no error :-)."
fi

exit 0;
