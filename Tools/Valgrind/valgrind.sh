#!/bin/bash

# assuming we are running on an Image with valgrind.
# assuming source contains already ASN types in c.

FULL=false

DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

EXECUTABLE=${1:-"Tests/UnitTests/cdff-unit-tests"}
OUTPUT_FOLDER=${2:-"/memcheck"}
IGNORE_FILE=${3:-"${DIR}/Tools/Valgrind/suppression.txt"}
FULL=${4:-true}

mkdir -p ${OUTPUT_FOLDER} && cd ${OUTPUT_FOLDER}

FLAGS="--leak-check=full --show-leak-kinds=all --error-exitcode=1 --child-silent-after-fork=yes --xml=yes --xml-file=${OUTPUT_FOLDER}/valgrind.xml --fullpath-after=CDFF/ --suppressions=${IGNORE_FILE}"
if [[ "$FULL" == true ]]
    then
        FLAGS+=" --track-origins=yes"
    else
        FLAGS+=" --track-origins=no --undef-value-errors=no"
fi

echo RUNNING valgrind: ${FLAGS}
valgrind ${FLAGS} ${EXECUTABLE}

ERROR_VALGRIND=$?
ERROR_COUNT=$(grep -c "<error>" ${OUTPUT_FOLDER}/results/valgrind.xml)

if [ $ERROR_VALGRIND -gt 0 ]
    then
        echo "=================== END OF LEAK CHECK - FAILURE ==================="
        echo "There are some problems with the code: $(($ERROR_COUNT)) errors detected!"
        exit 1
else
    echo "=================== END OF LEAK CHECK - SUCCESS ==================="
    echo "There was no memory leak detected in the code that was run :-)"
fi

exit 0;