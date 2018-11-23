cmake_minimum_required(VERSION 3.5.1 FATAL_ERROR)
project(CDFF VERSION 0.1 LANGUAGES C CXX)

# Maintainer: Romain Michalec <romain.michalec@strath.ac.uk>

## User interface --------------------------------------------------------------
##
## You can set these cache entries on the cmake command line: -D <var>=<value>.
## The setting will take priority over the project's default values (defined
## below) and update the entries present in the CMake cache (CMakeCache.txt).

# Choose between building the CDFF as static libraries or shared libraries

option(BUILD_SHARED_LIBS
  "Build shared libraries instead of static libraries" OFF)

# To use the libraries provided in the source tree, set USE_BUNDLED_DEPENDENCIES
# to ON: CMake will look for headers, compiled libraries, and executable files
# under BUNDLED_DEPENDENCIES_PREFIX before looking anywhere else, in particular
# in your system directories.
#
# When building inside a container spanned from a CDFF Docker image, leave at
# OFF: the appropriate libraries are in /usr/local.
#
# Switching from ON to OFF without deleting the CMake cache will not result in
# the desired effect if bundled libraries were previously found, because
# information about them will persist in the CMake cache. Delete the cache.

option(USE_BUNDLED_DEPENDENCIES
  "Preferably link the CDFF against the libraries provided in the source tree" OFF)

set(BUNDLED_DEPENDENCIES_PREFIX External/install CACHE PATH
  "Location of the libraries in the source tree")

# Define preprocessor definition TESTING
# Use it in #ifdef and #ifndef preprocessor conditionnals in your source code
#
# TODO: this preprocessor definition should probably be defined at a more inner
# scope than the top-level CMakeLists file, perhaps with target_compile_
# definitions() or set_source_files_properties()?

add_definitions(-DTESTING)

# Generate tests for CTest. Each test defined with the CMake command add_test()
# will be registered as a CTest test. Run these tests with ctest(1). The enable_
# testing() command must be in the top-level source directory.

enable_testing()

# Flags passed to the compiler

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

## Print basic information -----------------------------------------------------

message(STATUS "Generating makefiles for project: ${PROJECT_NAME}")
message(STATUS "Top-level source directory: ${CMAKE_SOURCE_DIR}")
message(STATUS "Top-level build directory:  ${CMAKE_BINARY_DIR}")

## Work in progress ------------------------------------------------------------

# TODO: remove this temporary internal variable when all find_package() commands
# load everything they are supposed to load (probably just requires fixing inner
# CMakeLists files). For now this variable selects between looking for VTK, PCL,
# OpenCV in /usr/local (default installation path selected in the Docker image)
# or in the source tree, and it also helps with finding FindBoost.cmake.

set(_DEPENDENCIES_PREFIX /usr/local)

if(USE_BUNDLED_DEPENDENCIES)
  set(_DEPENDENCIES_PREFIX "${BUNDLED_DEPENDENCIES_PREFIX}")
endif()

## Customize search paths ------------------------------------------------------

# When used in "module" mode, find_package() looks for a find module called
# Find<package>.cmake in the directories specified in CMAKE_MODULE_PATH
# (default: empty) before checking the default modules that come with CMake.
#
# include() looks for modules called <module>.cmake in the exact same way.
#
# Find modules loaded by find_package() in "module" mode will typically look
# for the package's components, whether compiled libraries, header files, and/or
# binaries, by using the other find_*() commands, so look further down for the
# CMAKE_PREFIX_PATH variable. Additionally, they will typically give higher
# priority to package-specific variables (e.g. BOOST_ROOT), if set.

if(USE_BUNDLED_DEPENDENCIES)

  set(CMAKE_MODULE_PATH
    "${CMAKE_SOURCE_DIR}/${BUNDLED_DEPENDENCIES_PREFIX}/share/cmake-3.11.4/Modules")

else()

  # TODO: check if CMake 3.5.1 doesn't automatically look for find modules in
  # /usr/local/share/cmake*/Modules (it does in /usr/share/cmake-3.5/Modules)
  set(CMAKE_MODULE_PATH
    "${_DEPENDENCIES_PREFIX}/share/cmake-3.11.4/Modules")

endif()

if(USE_BUNDLED_DEPENDENCIES)

  # When used in "config" mode, find_package() looks for a package configuration
  # file called <package>Config.cmake or <lower-case-package>-config.cmake in
  # the directories specified by certain cache/environment variables, including
  # directories deduced from PATH, as well as in a variety of standard locations
  # under these directories.
  #
  # find_library(), find_file(), find_path(), and find_program() look for a
  # compiled library, a file, a directory containing the given file, and an
  # executable file in the directories specified by similar cache/environment
  # variables, including directories deduced from PATH, as well as in their
  # lib/, include/, and [s]bin/ subdirectories, respectively.
  #
  # The complete search procedure is described in the documentation of those
  # functions.
  #
  # In all cases, the first search paths tried are those under the directories
  # specified by the following variable (default: empty):

  set(CMAKE_PREFIX_PATH "${BUNDLED_DEPENDENCIES_PREFIX}")

endif()

## Info/debug function ---------------------------------------------------------

# Print whether a previously executed find_package() command ran in "module"
# mode or in "config" mode

macro(find_package_mode pkg)

  if(NOT ${pkg}_CONFIG)
    message(STATUS "Found CMake find module for ${pkg}")
  else()
    message(STATUS "Found package configuration file for ${pkg}: ${${pkg}_CONFIG}")
  endif()

endmacro()

## Look for Boost --------------------------------------------------------------

# Most Boost libraries are header-only. Please write the compiled libraries that
# you want to link against after the COMPONENTS keyword, so that CMake looks for
# them: for instance date_time for Boost.Date_Time aka libboost_date_time.so.
#
# CMake will look for the headers and the requested compiled libraries, as well
# as the compiled libraries they depend on.

find_package(Boost 1.66.0 REQUIRED)

if(Boost_FOUND)

  find_package_mode(Boost)

  # To use header-only libraries:
  #
  # * In your source:
  #     #include <boost/...>
  #
  # * In your CMakeLists file:
  #     include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
  #     add_library(mylib src1.cpp src2.cpp ...)
  #
  # To use compiled libraries:
  #
  # * In your source:
  #     #include <boost/...>
  #
  # * In your CMakeLists file:
  #     include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
  #     add_executable(mybin src1.cpp src2.cpp ...)
  #     target_link_libraries(mybin Boost::date_time Boost::serialization ...)
  #       # to link against the specified libraries and their dependencies
  #
  # TODO: target_include_directories() should be preferred to include_
  # directories() but I don't understand how it works

  message(STATUS "Found Boost headers in: ${Boost_INCLUDE_DIRS}")
  message(STATUS "Found Boost compiled libraries in: ${Boost_LIBRARY_DIRS}")

  # TODO: better reporting of imported targets
  if(TARGET Boost::boost)
    message(STATUS "Succesfully imported targets: Boost::boost")
  endif()

  message(STATUS "All requested Boost compiled libraries, "
    "and their dependencies: ${Boost_LIBRARIES}")

  # TODO: kept for compatibility with the existing CDFF code base: to be removed
  # when all CMakeLists that define targets that use Boost libraries feature the
  # appropriate [target_]include_directories()
  include_directories(SYSTEM ${Boost_INCLUDE_DIRS})

else()

  # Abort if the headers or a requested compiled library is not found
  return()

endif()

## Look for Eigen --------------------------------------------------------------

# TODO: doc

find_package(Eigen3 3.3.4 REQUIRED)

if(Eigen3_FOUND)

  find_package_mode(Eigen3)

  # TODO: better reporting of imported targets
  if(TARGET Eigen3::Eigen)
    message(STATUS "Succesfully imported target: Eigen3::Eigen")
    message(STATUS "Pass it to your target_link_libraries() commands")
  endif()

else()

  # Abort if the headers are not found
  return()

endif()

# TODO: remove, use imported target Eigen3::Eigen instead
include_directories(
  SYSTEM "${_DEPENDENCIES_PREFIX}/include/eigen3"
)

## Look for Ceres --------------------------------------------------------------

# TODO: doc

find_package(Ceres 1.14.0 REQUIRED COMPONENTS SparseLinearAlgebraLibrary)

if(Ceres_FOUND)

  find_package_mode(Ceres)

  # To use Ceres:
  #
  # * In your source:
  #     #include <ceres/...>
  #
  # * In your CMakeLists file:
  #     include_directories(SYSTEM ${CERES_INCLUDE_DIRS})
  #     add_executable(mybin src1.cpp src2.cpp ...)
  #     target_link_libraries(mybin ${CERES_LIBRARIES})
  #       # to link against Ceres and its dependencies
  #
  # TODO: target_include_directories() should be preferred to include_
  # directories() but I don't understand how it works

  message(STATUS "Found Ceres headers, and the headers of its dependencies, in: ${CERES_INCLUDE_DIRS}")

  # TODO: CERES_LIBRARIES appears to actually be an imported target instead of a list of
  # libraries, cf. http://ceres-solver.org/installation.html#understanding-the-cmake-package-system
  message(STATUS "Requested Ceres library, "
    "and its dependencies: ${CERES_LIBRARIES}")

  # TODO: kept for compatibility with the existing CDFF code base: to be removed
  # when all CMakeLists that define targets feature the appropriate [target_]
  # include_directories()
  # TODO: also, ${CERES_INCLUDE_DIRS} features /usr/include if some dependencies
  # of Ceres have been installed as osdeps, so /usr/include will be before all
  # other include dirs subsequently appended to the list, which can cause trouble
  # when looking for a library which is in two different version in /usr/include
  # and ${_DEPENDENCIES_PREFIX}/include -> really necessary to restrict the
  # include dirs to specific targets
  include_directories(SYSTEM ${CERES_INCLUDE_DIRS})

else()

  # Abort if Ceres is not found
  return()

endif()

## Don't look for FLANN, QHull, yaml-cpp, CloudCompare-core --------------------

# There aren't any CMake find modules nor any package configuration files for
# those libraries, so just add the main include directory and use the libraries
# as e.g. #include <flann/stuff.hpp>
#
# TODO: there are .pc files though, that might be of use in conjunction with
# CMake's pkg-config module

include_directories(
  SYSTEM "${_DEPENDENCIES_PREFIX}/include"
)

link_directories(
  "${_DEPENDENCIES_PREFIX}/lib"
)

## Look for VTK, PCL, OpenCV ---------------------------------------------------

# TODO: rework this section

find_package(VTK 8.1.0 REQUIRED NAMES VTK VTK-8.1 CONFIGS VTKConfig.cmake)
find_package_mode(VTK)

find_package(PCL 1.8.1 REQUIRED NAMES PCL PCL-1.8 CONFIGS PCLConfig.cmake)
find_package_mode(PCL)
add_definitions(${PCL_DEFINITIONS})
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

message(STATUS "Preprocessor definitions and compiler flags added by the PCL: ${PCL_DEFINITIONS}")
message(STATUS "Found PCL headers, and the headers of its dependencies, in: ${PCL_INCLUDE_DIRS}")
message(STATUS "Found PCL libraries in: ${PCL_LIBRARY_DIRS}")
message(STATUS "All requested PCL libraries, and their dependencies: ${PCL_LIBRARIES}")

find_package(OpenCV 3.4.0 REQUIRED)
find_package_mode(OpenCV)

## Look for libpointmatcher ---------------------------------------------------

# TODO: rework this section

find_package(libpointmatcher 1.3.0 REQUIRED)


## Look for Edres-Wrapper (optional dependency) --------------------------------

find_package(Edres-Wrapper QUIET)

if(Edres-Wrapper_FOUND)

  find_package_mode(Edres-Wrapper)
  message(STATUS "Found Edres-Wrapper headers in: ${Edres-Wrapper_INCLUDE_DIRS}")
  message(STATUS "Found Edres-Wrapper libraries, and their dependencies: ${Edres-Wrapper_LIBRARIES}")

  option(WITH_EDRES "Build Edres-dependent parts" ON)
  include_directories(SYSTEM ${Edres-Wrapper_INCLUDE_DIRS})

else()

  message(STATUS "Edres-Wrapper not found: Edres-dependent parts will not be built")
  option(WITH_EDRES "Build Edres-dependent parts" OFF)

endif()

## Look for DLRTracker-core (optional dependency) ------------------------------

find_package(DLRtracker_core QUIET)

if(DLRTRACKER-CORE_FOUND)

  find_package_mode(DLRtracker_core)
  message(STATUS "Found DLRTracker-core headers in: ${DLRTRACKER-CORE_INCLUDE_DIRS}")
  message(STATUS "Found DLRTracker-core libraries, and their dependencies: ${DLRTRACKER-CORE_LIBRARIES}")

  option(WITH_DLRTRACKER "Build DLRTracker-dependent parts" ON)
  include_directories(SYSTEM ${DLRTRACKER-CORE_INCLUDE_DIRS})

else()

  message(STATUS "DLRTracker-core not found: DLRTracker-dependent parts will not be built")
  option(WITH_DLRTRACKER "Build DLRTracker-dependent parts" OFF)

endif()

## Setup CDFF installation -----------------------------------------------------

# Standard installation directories, as defined by the GNU Coding Standards:
#
# * CMAKE_INSTALL_INCLUDEDIR is include
# * CMAKE_INSTALL_LIBDIR is lib
# * CMAKE_INSTALL_BINDIR is bin
# * Full list at https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
#
# Use them in your install() commands, see example below. CMAKE_INSTALL_PREFIX,
# default /usr/local, will be prepended to those directories.

include(GNUInstallDirs)

# Install all the header files

file(GLOB_RECURSE headers
  RELATIVE "${CMAKE_SOURCE_DIR}"
  "Common/*.h" "Common/*.hpp" "DFNs/*.hpp" "DFPCs/*.hpp" "Tests/*.hpp")
list(SORT headers)

foreach(header ${headers})
  get_filename_component(dir "${header}" DIRECTORY)
  install(FILES "${header}"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}/${dir}")
endforeach()

## Setup CDFF uninstallation ---------------------------------------------------

# Add an uninstall target, see documentation in cmake_uninstall.cmake[.in]

configure_file(
  "${CMAKE_SOURCE_DIR}/Tools/CMake/cmake_uninstall.cmake.in"
  "${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
  @ONLY)

add_custom_target(uninstall
  COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_BINARY_DIR}/cmake_uninstall.cmake"
  VERBATIM)

# Add a purge target, see documentation in cmake_purge.cmake[.in]

configure_file(
  "${CMAKE_SOURCE_DIR}/Tools/CMake/cmake_purge.cmake.in"
  "${CMAKE_BINARY_DIR}/cmake_purge.cmake"
  @ONLY)

add_custom_target(purge
  COMMAND "${CMAKE_COMMAND}" -P "${CMAKE_BINARY_DIR}/cmake_purge.cmake"
  VERBATIM)

## Compile the CDFF ------------------------------------------------------------

# Take into account the headers provided in the following directories
#
# To use these headers:
#
# * In your source:
#     #include "[subdir/.../]header.hpp"
#
# TODO: as with the previous libraries, [target_]include_directories() commands
# written in inner CMakeLists file should be preferred to include_directories()
# written in the top-level CMakeLists file

include_directories(
  Common
  DFNs
  DFPCs
)

# Process all subdirectories containing a CMakeLists file

add_subdirectory(Common)
add_subdirectory(DFNs)
add_subdirectory(DFPCs)

# Optionally compile the tests (unit tests, interactive tests, performance
# tests, etc)

option(BUILD_TESTS "Build the CDFF's tests" ON)

if(BUILD_TESTS)
  include_directories(Tests)
  add_subdirectory(Tests)
endif()
