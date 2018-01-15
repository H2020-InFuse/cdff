project(infuse_dev)
cmake_minimum_required(VERSION 2.6)

#include(FindPkgConfig) #uncomment if you want to use your system libs instead
#find_package( Boost 1.61 REQUIRED )
#find_package( Eigen3 3.3.4 REQUIRED )
#find_package( flann 1.9.1 REQUIRED )
#find_package( OpenCV 3.4.0 REQUIRED )
#find_package( pcl 1.8.1 REQUIRED )
#find_package( QHull REQUIRED )
#find_package( vtk 8.1.0 REQUIRED )
#find_package( yaml-cpp 0.5.3 REQUIRED )
#find_package( Eigen3 3.3.4 REQUIRED )

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
    External/install/include
    External/install/include/boost
    External/install/include/eigen3
    External/install/include/flann
    External/install/include/libqhull
    External/install/include/libqhullcpp
    External/install/include/libqhull_r
    External/install/include/opencv
    External/install/include/opencv2
    External/install/include/pcl-1.8
    External/install/include/vtk-8.1
    External/install/include/yaml-cpp
)
link_directories(
	 /CDFF/External/install/lib
)

add_subdirectory(Common)
add_subdirectory(DFNs)
add_subdirectory(Tests)
