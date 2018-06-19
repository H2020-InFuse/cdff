#!/bin/sh

# This script is intended for use as the entrypoint script of a Docker
# container. See Dockerfile.user.
#
# POSIX and Bourne-compatible. Requires: POSIX-compatible grep, head, cut (e.g.
# GNU Grep and GNU Core Utilities), sudo, the Python 3 package manager.
#
# Maintainer: Romain Michalec <romain.michalec@strath.ac.uk>

# Find out if and where CDFF and CDFF-dev have been mounted
CDFF=$(grep    -F -i -e "cdff "         /proc/mounts | head -n 1 | cut -d " " -f 2)
CDFFDEV=$(grep -E -i -e "cdff[_-]?dev " /proc/mounts | head -n 1 | cut -d " " -f 2)

if [ "${CDFF}" ]; then
  echo "Found CDFF-core and CDFF-support: ${CDFF}/"

  if [ -d "${CDFF}/Common/Types/C" ]; then
    echo "Found compiled ASN.1 data types:  ${CDFF}/Common/Types/C/"
  else
    echo "Compiled ASN.1 data types not found. Run CMake in the CDFF directory."
  fi
fi

if [ "${CDFFDEV}" ]; then
  echo "Found CDFF-dev: ${CDFFDEV}/"
fi

# Build and install CDFF-dev at container creation time from its sources
# mounted by the user if and only if:
# - The user has provided all components of the CDFF
# - The ASN.1 data types have been compiled to C files
# - CDFF-dev is not already installed (it is when restarting a container)
if [ "${CDFF}" -a "${CDFFDEV}" ]; then

  echo "Setting up CDFF-dev... "

  # Build and install
  if [ -d "${CDFF}/Common/Types/C" -a -z "$(pip3 show --no-cache-dir cdff-dev)" ]; then
    sudo -H /tmp/CDFF/Tools/Docker/cdff-dev.sh "${CDFF}" "${CDFFDEV}"
  fi

  # Set /path/to/CDFF in the environment
  # This will be inherited by the process started at the end of this script
  CDFFPATH="${CDFF}"
  export CDFFPATH

  echo "Setting up CDFF-dev: done"
fi

# Replace this script by whatever command is given on the "docker run" command
# line or in the image's CMD instruction, in this order of precedence, without
# creating a new process. The script terminates and the command becomes the
# container's primary process.
exec "${@}"
