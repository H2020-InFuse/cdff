#===========================================================================================================================================
#
#                            LIBORBSLAM
#
#===========================================================================================================================================
#
# Author: Vincent Bissonnette
# Company: Magellium
#
# Language: CMake
#
# Description: Main CMakeLists.txt of this library
#===========================================================================================================================================
CMAKE_MINIMUM_REQUIRED(VERSION 3.0)

# Declare the library name in UPPERCASE
# The library will have the same name in lowercase
PROJECT(LIBORBSLAM)

# Some usefule CMake macros to build the library
INCLUDE(cmake/tools.cmake)
SET(VERSION 1.0.0)

# Set a common output directory for all modules, it facilitates the execution.
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Add the subfolder containing any FindXXXX.cmake for this library
SET(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})

#================================================================================
# 1 - Initialization
#================================================================================
# Initialize an iteration. Allows for recursive cmake submodules.
# INPUT  : None
# OUTPUT : defintion of ${PROJECT_NAME}_ISMAIN (TRUE if this is the top-level CMake)
INIT_MODULE()

#================================================================================
# 2 - Some customizing to be done first, if needed
#================================================================================

SET(CMAKE_CXX_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11  -O3 ")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#================================================================================
# 3 - Add submodules present in the "modules" directory, if any
#================================================================================
ADD_SUBMODULES()

#================================================================================
# 4 - Find all dependencies
#     The following variables must be set to propagate dependecy information:
#     INCLUDE_DIRS : necessary includes for users of this library
#     LIBRARIES    : necessary libraries for the users of this library
#================================================================================
FIND_PACKAGE(Eigen3 REQUIRED)
LIST(APPEND INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS})
LIST(APPEND LIBRARIES    ${EIGEN3_LIBRARIES})

FIND_PACKAGE(DBoW2 REQUIRED)
LIST(APPEND INCLUDE_DIRS ${DBoW2_INCLUDE_DIRS})
LIST(APPEND LIBRARIES    ${DBoW2_LIBRARIES})

FIND_PACKAGE(g2o REQUIRED)
LIST(APPEND INCLUDE_DIRS ${g2o_INCLUDE_DIRS})
LIST(APPEND LIBRARIES    ${g2o_LIBRARIES})

FIND_PACKAGE(Pangolin REQUIRED)
LIST(APPEND INCLUDE_DIRS ${Pangolin_INCLUDE_DIRS})
LIST(APPEND LIBRARIES    ${Pangolin_LIBRARIES})

FIND_PACKAGE(OpenCV 3 REQUIRED)
LIST(APPEND INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
LIST(APPEND LIBRARIES    ${OpenCV_LIBRARIES})

FIND_PACKAGE(Boost COMPONENTS serialization REQUIRED)
LIST(APPEND INCLUDE_DIRS ${Boost_INCLUDE_DIRS})
LIST(APPEND LIBRARIES    ${Boost_LIBRARIES})

#================================================================================
# 5 - Default include
#================================================================================
LIST(APPEND INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include)

INCLUDE_DIRECTORIES(${INCLUDE_DIRS})

#================================================================================
# 6 - Find all library sources and headers, following the convention
#================================================================================
FILE(GLOB SRCFILES "src/*.cc" "src/*.cpp")

FILE(GLOB_RECURSE HDRFILES "include/*.h")

#================================================================================
# 7 - Declare the library name
#================================================================================
ADD_LIBRARY_AUTO()

#================================================================================
# 8 - Declare all executables, following the convention
#================================================================================
FILE(GLOB_RECURSE APPFILES "app/*.cc" "app/*.cpp")
FOREACH(APPFILE ${APPFILES})
    GET_FILENAME_COMPONENT(APPNAME ${APPFILE} NAME_WE)
    ADD_EXECUTABLE(${APPNAME} ${APPFILE})
    TARGET_LINK_LIBRARIES(${APPNAME} ${LIBRARIES} -lpthread)
ENDFOREACH()

#================================================================================
# 9 - Create an export 
#================================================================================
SET(USE_${PROJECT_NAME} TRUE)
EXPORT_MODULE()

#================================================================================
# 10 - Build tests, if any
#================================================================================
#ADD_SUBDIRECTORY(tests)

#================================================================================
# 11 - Generate documentation
# When the current CMakeLists is the top-level one, the Doxyfile is generated from 
# the Doxyfile.in template, and the main page of the documentation becomes the one
# of the project. Further CMakeLists in submodules are added recursively. 
#================================================================================
IF( ${PROJECT_NAME}_ISMAIN )
    FIND_PACKAGE(Doxygen)
    IF(DOXYGEN_FOUND)
	SET(ROOT_PROJECT "${PROJECT_NAME}_ISMAIN")
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile.in ${CMAKE_CURRENT_SOURCE_DIR}/Doxyfile @ONLY)
            ADD_CUSTOM_TARGET(doc ${DOXYGEN_EXECUTABLE}
            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
            COMMENT "Generating API documentation with Doxygen" VERBATIM)
    ENDIF(DOXYGEN_FOUND)
ENDIF()
