#!/usr/bin/env bash

# Maintainers: xma@spaceapplications.com, romain.michalec@strath.ac.uk

# Exit the shell immediately if a command exits with a non-zero status
set -e

# Canonical path to the directory containing this script
# Uses GNU readlink from GNU coreutils
DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

# Canonical paths to the source and installation directories (-m because
# External/ doesn't exist yet when using this script to build a Docker image)
SOURCE_DIR="$(readlink -m "${DIR}/../../External/source")"
INSTALL_DIR="$(readlink -m "${DIR}/../../External/install")"
PKG_DIR="$(readlink -m "${DIR}/../../External/package")"

## Functions used by the main installer (this script) #########################

# Usage
function print_help {
  cat <<EOF
Usage: [bash [--]] get-cdff-dependencies [OPTIONS]

Download, build, and install the direct dependencies of the CDFF. This program
doesn't install the dependencies of those dependencies, you are responsible for
making sure that they are available on your system.

If CheckInstall is available, it is used for installation, otherwise a regular
make install is used. The program assumes that you have write access to the
chosen installation prefix, and will fail if you haven't.

Options:
  -h, --help             Display this help and exit
  -d, --dependency LIB   Install LIB
                         Repeat as needed: -d LIB1 -d LIB2 ...
                         Default: boost yaml-cpp eigen cloudcompare-core ceres
                           flann nabo pointmatcher qhull opencv vtk pcl
                           edres-wrapper
  -e, --envire           Install EnviRe and its dependencies
                         Required by CDFF::CentralDPM
                         Default: no
  -s, --sources DIR      Where to download the sources of the requested LIBs
                         Default: ${SOURCE_DIR}
  -i, --install DIR      Installation prefix
                         Default: ${INSTALL_DIR}
  -p, --package DIR      Output directory for packages made by CheckInstall
                         Default: ${PKG_DIR}
EOF
}

# Cleanup leftover source directories in case of early termination on error
function on_exit {
  for dependency in "${dependencies[@]}"; do
    if [[ -d "${SOURCE_DIR}/${dependency}" ]]; then
      echo "Removing leftover source directory: ${SOURCE_DIR}/${dependency}" >&2
      rm -rf "${SOURCE_DIR:?}/${dependency}"
    fi
  done
}
trap on_exit EXIT

# Import all functions present in all scripts in the installers/ subdirectory
function find_installers {
  if [[ ! -d "${DIR}/installers" ]]; then
    echo "${DIR}/installers: no such directory" >&2
    exit 1
  fi

  for file in "${DIR}"/installers/*.sh; do
    source "${file}"
  done

  declare -g -A installers
  installer_prefix=install4infuse_
  while read -r -d " " fct_name; do
    dependency=${fct_name#${installer_prefix}}
    installers[${dependency}]=${fct_name}
  done <<< $(declare -F | grep -o "${installer_prefix}.*$")
}

# Run all requested installers who have an install function
function run_installers {
  mkdir -p "${SOURCE_DIR}"
  mkdir -p "${INSTALL_DIR}"
  mkdir -p "${PKG_DIR}"

  cd "${SOURCE_DIR}"
  for dependency in "${dependencies[@]}"; do
    printf "#\n# Running installer for %s\n#\n" ${dependency}
    eval ${installers[${dependency}]}
    printf "#\n# Running installer for %s: done\n#\n" ${dependency}
  done
}

## Functions used by the installers in installers/ ############################

# How many processors?
if [ -f /proc/cpuinfo ]; then
  CPUS=$(grep --count --regexp=^processor /proc/cpuinfo)
else
  CPUS=1
fi

# A wrapper around "wget"
function cdff_wget {
  echo "Downloading ${1}"
  mkdir -p "${SOURCE_DIR}/${1}"
  cd "${SOURCE_DIR}/${1}"
  wget "${3}${2}"
  if [[ "${2: -7}" == ".tar.gz" ]]; then
    tar x --file="${2}"
    rm -f "${2}"
    cd "${2%.tar.gz}"
  fi
  echo "Downloading ${1}: done"
}

# A wrapper around "git clone"
function cdff_gitclone {
  echo "Cloning ${1}'s code repository"
  # Uncomment the lines prefixed with #+# to install from local sources already
  # available in ${SOURCE_DIR}/${1} instead of first cloning sources in there;
  # this can be useful, for instance, for debugging purposes
  #+# if [ ! -d "${SOURCE_DIR}/${1}" ]; then
    if [ -z ${4} ]; then
      git -C "${SOURCE_DIR}" clone --recursive --depth 1 --single-branch --branch "${2}" "${3}" "${1}"
    else
      git -C "${SOURCE_DIR}" clone --recursive --branch "${2}" "${3}" "${1}"
      git -C "${SOURCE_DIR}/${1}" checkout -f ${4}
    fi
  #+# else
  #+#   echo "Directory ${SOURCE_DIR}/${1} already exists, we will work with that one."
  #+# fi
  cd "${SOURCE_DIR}/${1}"
  echo "Cloning ${1}'s code repository: done"
}

# A wrapper around "make install", or "checkinstall" if installed
function cdff_makeinstall {
  if (command -v checkinstall); then
   sudo checkinstall -y --pakdir "${PKG_DIR}" --nodoc --pkgname="${1}" --pkgversion="${2}"
  else
   make --jobs=${CPUS} install
  fi
}

# A wrapper around "rm -rf"
function cdff_makedistclean {
  echo "Removing ${1} source directory and build subdirectory"
  rm -rf "${SOURCE_DIR:?}/${1}"
  echo "Removing ${1} source directory and build subdirectory: done"
}

## Main installer #############################################################

# Parse command options (adapted from /usr/share/doc/util-linux/examples/
# getopt-parse.bash)
# Uses GNU getopt from util-linux, not the shell builtin getopts, nor the
# original getopt utility from the 1980s
SHORT=h,d:,e,i:,p:
LONG=help,dependency:,envire,install:,package:
PARSED=$(getopt --options ${SHORT} --longoptions ${LONG} --name "${0}" -- "${@}")
if [[ ${?} != 0 ]]; then
  echo "${0}: returning getopt error code" >&2
  exit ${?}
fi

eval set -- "${PARSED}"
while true; do
  case "${1}" in

    -h | --help) print_help; exit 0 ;;

    -d | --dependency) dependencies+=("${2}"); shift 2 ;;
    -e | --envire) envire=yes; shift ;;

    -s | --sources) SOURCE_DIR="$(readlink -f "${2}")"; shift 2 ;;
    -i | --install) INSTALL_DIR="$(readlink -f "${2}")"; shift 2 ;;
    -p | --package) PKG_DIR="$(readlink -f "${2}")"; shift 2 ;;

    --) shift; break ;;
    *)  echo "${0}: internal error!" >&2; exit 1 ;;

  esac
done

# If no -d option was provided, mark everything but EnviRe for installation
if [[ -z "${dependencies[*]}" ]]; then
  dependencies=(boost yaml-cpp eigen cloudcompare-core ceres nabo \
    pointmatcher flann qhull opencv vtk pcl edres-wrapper)
fi

# If the -e option was provided, mark EnviRe for installation
if [[ ${envire} = yes ]]; then
  dependencies+=(base_cmake base_logging sisl base_types base_numeric \
    base_boost_serialization console_bridge poco poco_vendor class_loader \
    tools_plugin_manager envire_envire_core)

  # Add pkg-config directories to PKG_CONFIG_PATH, if they aren't in
  # PKG_CONFIG_PATH already, and export PKG_CONFIG_PATH to the environment
  AddToPkg="${INSTALL_DIR}/lib/pkgconfig:${INSTALL_DIR}/share/pkgconfig"
  if [[ ":${PKG_CONFIG_PATH}:" != *":${AddToPkg}:"* ]]; then
    export PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:${AddToPkg}"
  fi

  # More environment variables used for installing EnviRe
  export LD_LIBRARY_PATH="${INSTALL_DIR}/lib"
  export Rock_DIR="${INSTALL_DIR}/share/rock"
  export console_bridge_DIR="${INSTALL_DIR}/share/console_bridge"
fi

# Parse the installers/ subdirectory
find_installers

# Unmark dependencies which don't have an installer
for i in "${!dependencies[@]}"; do
  if [[ -z ${installers[${dependencies[i]}]} ]]; then
    unset dependencies[i]
  fi
done

# Print what will be installed, and where
echo "Found installers for: " "${!installers[@]}"
echo "Dependencies selected for installation: " "${dependencies[@]}"
echo "Where sources will be downloaded: ${SOURCE_DIR}"
echo "Installation prefix:              ${INSTALL_DIR}"
echo "Output directory for packages:    ${PKG_DIR}"

# Install
if [[ "${dependencies[*]}" ]]; then

  # TODO: check if we have write access to ${INSTALL_DIR}: if not ask user
  # password and cache it for sudo, if yes run next test

  # Check if `checkinstall` is installed. If it is installed we need superuser
  # privileges to complete the build & install for each package even if we're
  # not installing the packages globally.
  if type checkinstall 2>/dev/null; then
    echo "Caching your sudoers credential for running checkinstall unattended"
    sudo --validate
  fi

  # Run the installers for the selected dependencies
  run_installers
fi

exit 0
