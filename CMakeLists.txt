project(infuse_dev)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)

#This compiles in testing mode
add_definitions(-DTESTING)

#This policy disable warning for the use of relative libraries path
cmake_policy(SET CMP0015 NEW)

#Reference directories
include_directories(
    DFNs Common Tests Tools Common/Types/C/ASNCommon Common/Types/C Common/Converters External/opencv3/include External/tinyxml2/include
)
link_directories(
	 External/opencv3/lib External/tinyxml2/lib
)


add_subdirectory(Common)
add_subdirectory(DFNs)
add_subdirectory(Tests)

add_library(FeaturesExtraction2D DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Common/XmlHelper/XmlHelper.cpp)
target_link_libraries(
	FeaturesExtraction2D cdff_types tinyxml2
)

#Moving configurations
FILE (COPY Tests/ConfigurationFiles/ DESTINATION ./tests/ConfigurationFiles/)
FILE (COPY Tests/Data/ DESTINATION ./tests/Data/)

