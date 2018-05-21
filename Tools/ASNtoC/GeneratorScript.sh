#!/usr/bin/env bash

# This script runs the ASN1SCC compiler on the ASN.1 data types in (by default)
# Common/Types/ASN.1 to compile them to C files in Common/Types/C. Beware that
# all existing files and directories in Common/Types/C will be deleted.
#
# The ASN1SCC compiler must be run every time an ASN.1 data type is added or
# modified.
#
# Maintainers: xma@spaceapplications.com, romain.michalec@strath.ac.uk

#exit immediately if a simple command exits with a nonzero exit value.
set -e

#Get working directory and script containing directory
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" && pwd )"

# Input (ASN.1) and output (C) directories
if [[ ${#} -eq 1 ]]; then
  # Undocumented because not robust (invalid input allowed)
  ASN1_DIR="${1}/ASN.1"
  ASN1_OUT_DIR="${1}/C"
else
  # Default paths
  ASN1_DIR="${DIR}/../../Common/Types/ASN.1"
  ASN1_OUT_DIR="${DIR}/../../Common/Types/C"
fi

rm -rf "${ASN1_OUT_DIR}"
mkdir -p "${ASN1_OUT_DIR}"

echo "Input directory (ASN.1): ${ASN1_DIR}"
echo "Output directory (C):    ${ASN1_OUT_DIR}"

# The ASN1SCC compiler can be built from its sources cloned from https://github.
# com/ttsiodras/asn1scc or downloaded in precompiled form from https://download.
# tuxfamily.org/taste/ASN1SCC
ASN1_COMPILER=asn1scc/asn1.exe
if [[ -f "${ASN1_COMPILER}" ]]; then
  echo "ASN1SCC compiler: ${ASN1_COMPILER}"
else
  echo "ASN1SCC compiler not found in current directory"
  echo "Downloading ASN1SCC compiler"
  wget --quiet https://download.tuxfamily.org/taste/ASN1SCC/ASN1SCC-latest.tgz
  echo "Downloading ASN1SCC compiler: done"
  echo "Extracting ASN1SCC compiler in current directory"
  tar xf ASN1SCC-latest.tgz
  rm -f ASN1SCC-latest.tgz
  echo "Extracting ASN1SCC compiler in current directory: done"
fi

# Compile ASN.1 files to C files
ASN1_PREFIX=asn1Scc
echo "Compiling ASN.1 data types to C"
find "${ASN1_DIR}" -name \*.asn -print0 |
  xargs -0 mono "${ASN1_COMPILER}" \
    -c -typePrefix ${ASN1_PREFIX} -uPER -wordSize 8 -ACN -o "${ASN1_OUT_DIR}" \
    "${ASN1_FILES}"
echo "Compiling ASN.1 data types to C: done"
