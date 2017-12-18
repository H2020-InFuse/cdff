project(infuse_dev)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)
find_package(
    OpenCV 3.0.0 REQUIRED
    PATHS ${CMAKE_SOURCE_DIR}/External/opencv3/share/OpenCV)

enable_testing()

#This compiles in testing mode
add_definitions(-DTESTING)

#This policy disable warning for the use of relative libraries path
cmake_policy(SET CMP0015 NEW)

include_directories(
    DFNs Common Tests Tools
    Common/Types/C/ASNCommon
    Common/Types/C Common/Converters
    External/opencv3/include
    External/tinyxml2/include
)
link_directories(
	 External/opencv3/lib
	 External/tinyxml2/lib
)

add_subdirectory(Common)
add_subdirectory(DFNs)
add_subdirectory(Tests)
