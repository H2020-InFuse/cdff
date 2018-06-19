#!/bin/sh

# This script is intended to be run as root in a Docker container as part of
# that container's startup. It performs the installation of CDFF-dev as root.
# See entrypoint script.
#
# arg1  /path/to/CDFF
# arg2  /path/to/CDFF-dev
#
# Maintainer: Romain Michalec <romain.michalec@strath.ac.uk>

CDFFPATH="${1}" pip3 install --editable "${2}"

rm /etc/sudoers.d/cdff-dev
