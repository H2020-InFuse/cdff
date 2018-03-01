project(infuse_dev)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)
find_package( Boost 1.61 REQUIRED )
find_package( Eigen3 3.3.4 REQUIRED )
find_package( OpenCV 3.4.0 REQUIRED )
find_package( VTK 8.1.0 REQUIRED )
find_package( PCL 1.8.1 REQUIRED )
#find_package( Flann 1.9.1 REQUIRED )
#find_package( QHull REQUIRED )
#find_package( yaml-cpp 0.5.3 REQUIRED )

enable_testing()

#This compiles in testing mode
add_definitions(-DTESTING)

#C11
if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native")

#This policy disable warning for the use of relative libraries path
cmake_policy(SET CMP0015 NEW)

include_directories(
    DFNs Common Tests Tools DFPCs
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
	 External/install/lib
)

add_subdirectory(Common)
add_subdirectory(DFNs)
add_subdirectory(Tests)
add_subdirectory(DFPCs)
