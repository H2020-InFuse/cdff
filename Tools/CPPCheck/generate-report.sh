#!/usr/bin/env bash

INTPUT_FILE=${1:-"/cppcheck/cppcheck.xml"}
OUTPUT_FOLDER=${2:-"/cppcheck/report"}

mkdir -p ${OUTPUT_FOLDER}
cppcheck-htmlreport --file=${INTPUT_FILE} --report-dir=${OUTPUT_FOLDER}