#!/usr/bin/env bash

# xma@spaceapplications.com, romain.michalec@strath.ac.uk
# This script downloads, builds, and installs the direct dependencies of the
# CDFF. It doesn't take care of the recurse dependencies of the CDFF.

# Exit the shell immediately if a command exits with a non-zero status
set -e

# Canonical path to the directory containing this script
DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

# Canonical paths to the build and installation directories
# (-m because External/ doesn't exist yet when building a Docker image)
SOURCE_DIR="$(readlink -m "${DIR}/../../External/source")"
BUILD_DIR="$(readlink -m "${DIR}/../../External/build")"
INSTALL_DIR="$(readlink -m "${DIR}/../../External/install")"
PKG_DIR="$(readlink -m "${DIR}/../../External/package")"

# How many processors?
if [ -f /proc/cpuinfo ]; then
  CPUS=$(grep --count --regexp=^processor /proc/cpuinfo)
else
  CPUS=1
fi

# Print usage
function print_help {
  cat <<EOF
Usage: ${BASH_SOURCE[0]} [OPTION]...

Configuration:
  -h, -?   Display this help and quit
  -s LIB   Build and install LIB
           Can be repeated (-s LIB1 -s LIB2 ...)
           Default:
             Build and install the following LIBs: boost yaml-cpp eigen
             cloudcompare-core ceres flann nabo pointmatcher qhull opencv
             vtk pcl edres-wrapper
  -e       Build and install EnviRe and its dependencies
           Required by CDFF::CentralDPM
           Default:
             Disabled
  -c       Print the selected configuration

Directories:
  -b DIR   Prefix for build directories
           Default:
             ${BUILD_DIR}
  -i DIR   Installation prefix
           Default:
             ${INSTALL_DIR}
  -p DIR   Output directory for packages made by CheckInstall, if available
           Default:
             ${PKG_DIR}
EOF
}

# Print selected configuration
function print_config {
  printf "Dependencies that will be built and installed:"
  for dependency in "${dependencies[@]}"; do
    if [[ ${installers[${dependency}]} ]] ;  then
      printf " %s" ${dependency}
    fi
  done
  printf "\n"
  echo "Prefix for source directories: ${SOURCE_DIR}"
  echo "Prefix for build directories : ${BUILD_DIR}"
  echo "Installation prefix:           ${INSTALL_DIR}"
  echo "Output directory for packages: ${PKG_DIR}"
}

# Import all functions present in all scripts in the installers/ subdirectory
declare -A installers
function find_installers {
  if [[ ! -d "${DIR}/installers" ]]; then
    echo "${DIR}/installers directory missing"
    exit 1
  fi

  for file in "${DIR}"/installers/*.sh
  do
     source "${file}"
  done

  PreviousIFS=$IFS
  IFS=$'\n'
  installer_prefix=install4infuse_
  for f in $(declare -F); do
    fct_name="${f:11}"
    if [[ ${fct_name} == ${installer_prefix}* ]]; then
      dependency=${fct_name#${installer_prefix}}
      installers[${dependency}]=${fct_name}
    fi
  done
  echo "Found installers for:" "${!installers[@]}"
  IFS=$PreviousIFS
}

# Run all requested installers who have an install function
function run_installers {
  mkdir -p "${SOURCE_DIR}"
  mkdir -p "${BUILD_DIR}"
  mkdir -p "${INSTALL_DIR}"
  mkdir -p "${PKG_DIR}"

  cd "${BUILD_DIR}"
  for dependency in "${dependencies[@]}"; do
    if [[ ${installers[${dependency}]} ]]; then
      echo "#"
      echo "# Running installer for ${dependency}"
      echo "#"
      eval ${installers[${dependency}]}
      echo "#"
      echo "# Running installer for ${dependency}: done"
      echo "#"
    fi
  done
}

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
  mkdir -p "${BUILD_DIR}/${1}"
  cd "${BUILD_DIR}/${1}"
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
  echo "Removing ${1} source and build directories"
  cd "${SOURCE_DIR}"
  rm -rf "${SOURCE_DIR:?}/${1}"
  rm -rf "${BUILD_DIR:?}/${1}"
  echo "Removing ${1} source and build directories: done"
}

# Select all the dependencies for installation by run_installers
function select_all_dependencies {
  dependencies=(boost yaml-cpp eigen cloudcompare-core ceres nabo \
    pointmatcher flann qhull opencv vtk pcl edres-wrapper)
  if [[ "${ENVIRE_FULL}" = true ]]; then
    dependencies+=(base_cmake base_logging sisl base_types base_numeric  \
      base_boost_serialization console_bridge poco poco_vendor class_loader \
      tools_plugin_manager envire_envire_core)
  #else
    #dependencies+=(envire-min)
  fi
}

# Cleanup leftover source directories in case of early termination on error
function on_exit {
  for dependency in "${dependencies[@]}"; do
    if [[ -d  "${SOURCE_DIR}/${dependency}" ]]; then
      echo "Removing leftover source directory: ${SOURCE_DIR}/${dependency}"
      rm -rf "${SOURCE_DIR:?}/${dependency}"
    fi
  done
}
trap on_exit EXIT

# If ENVIRE_FULL: add pkg-config directories to PKG_CONFIG_PATH, if they aren't
# in PKG_CONFIG_PATH already, and export PKG_CONFIG_PATH to the environment
function set_pkgconfig_path {
  AddToPkg="${INSTALL_DIR}/lib/pkgconfig:${INSTALL_DIR}/share/pkgconfig"
  if [[ ":${PKG_CONFIG_PATH}:" != *":${AddToPkg}:"* ]]; then
    export PKG_CONFIG_PATH="${PKG_CONFIG_PATH}:${AddToPkg}"
  fi

  export LD_LIBRARY_PATH="${INSTALL_DIR}/lib"
  export Rock_DIR="${INSTALL_DIR}/share/rock"
  export console_bridge_DIR="${INSTALL_DIR}/share/console_bridge"
}

###### MAIN PROGRAM

# Parse the installers/ subdirectory
find_installers

# Parse the arguments provided by the user
dependencies=()
# A POSIX variable
OPTIND=1         # Reset in case getopts has been used previously in the shell.
# get options
while getopts ":b:i:p:s:c:e" opt; do
    case "$opt" in
    h|\?)
        print_help
        exit 0
        ;;
    c)
        print_config
        exit 0
        ;;
    b)
        BUILD_DIR=$OPTARG
        ;;
    i)
        INSTALL_DIR=$OPTARG
        ;;
    p)
        PKG_DIR=$OPTARG
        ;;
    s)
        dependencies+=($OPTARG)
        ;;
    e)
        ENVIRE_FULL=true
        echo "A complete version of EnviRe will be installed, along with EnviRe's dependencies"
        set_pkgconfig_path
        ;;
    :)
        echo "Option -$OPTARG requires an argument." >&2
        exit 1
        ;;
    esac
done

if [[ ($OPTIND -eq 1) || ( ($OPTIND -eq 2) && (${ENVIRE_FULL} = true) ) ]]; then
  # Check if `checkinstall` is installed. If it is installed we need superuser
  # privileges to complete the build & install for each package even if we're
  # not installing the packages globally.
  if type checkinstall 2>/dev/null; then
    echo "Caching your sudo password for install scripts ... "
    sudo true
  else
    echo "Checkinstall is not installed. No distribution packages will be generated."
  fi

  # Default when no -s argument is provided: build everything
  select_all_dependencies
fi
shift $((OPTIND-1))
[ "$1" = "--" ] && shift

# Print selected configuration and run the installers for the selected dependencies
print_config
run_installers
