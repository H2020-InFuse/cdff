project(infuse_dev)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)
#find_package(OpenCV 3.0.0 REQUIRED)

enable_testing()

#This compiles in testing mode
add_definitions(-DTESTING)

#C11
if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

#This policy disable warning for the use of relative libraries path
cmake_policy(SET CMP0015 NEW)

include_directories(
    DFNs Common Tests Tools
    Common/Types/C/ASNCommon
    Common/Types/C Common/Converters
    Common/Types/Cpp
    External/opencv3/include
    External/eigen/include/eigen3
    External/yamlcpp/include
    External/pcl/include/pcl-1.8
    External/vtk/include/vtk-8.0
    External/boost/include
    External/flann/include
    External/qhull/include
)
link_directories(
	 External/opencv3/lib
	 External/yamlcpp/lib
	 External/pcl/lib
	 External/vtk/lib
	 External/boost/lib
	 External/flann/lib
)

add_subdirectory(Common)
add_subdirectory(DFNs)
add_subdirectory(Tests)
