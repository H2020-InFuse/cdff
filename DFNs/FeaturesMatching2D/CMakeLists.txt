set(FEATUES_MATCHING_2D_SOURCES "FeaturesMatching2DInterface.cpp")
set(FEATUES_MATCHING_2D_INCLUDE_DIRS "")
set(FEATUES_MATCHING_2D_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(FEATUES_MATCHING_2D_SOURCES ${FEATUES_MATCHING_2D_SOURCES} "FlannMatcher.cpp")
	set(FEATUES_MATCHING_2D_INCLUDE_DIRS ${FEATUES_MATCHING_2D_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
	set(FEATUES_MATCHING_2D_DEPENDENCIES ${FEATUES_MATCHING_2D_DEPENDENCIES} ${OpenCV_LIBRARIES})
endif()

add_library(
    cdff_dfn_features_matching_2d
    ${FEATUES_MATCHING_2D_SOURCES}
)
target_include_directories(
    cdff_dfn_features_matching_2d
    SYSTEM PRIVATE ${FEATUES_MATCHING_2D_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_features_matching_2d
    ${FEATUES_MATCHING_2D_DEPENDENCIES}
)
