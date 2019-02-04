#!/usr/bin/env bash

DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
INTPUT_FILE=${1:-"${DIR}/report/cppcheck.xml"}
OUTPUT_FOLDER=${2:-"${DIR}/report"}

mkdir -p ${OUTPUT_FOLDER}
cppcheck-htmlreport --file=${INTPUT_FILE} --report-dir=${OUTPUT_FOLDER}
