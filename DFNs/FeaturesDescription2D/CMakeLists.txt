set(FEATURES_DESCRIPTION_2D_SOURCES "FeaturesDescription2DInterface.cpp")
set(FEATURES_DESCRIPTION_2D_INCLUDE_DIRS "")
set(FEATURES_DESCRIPTION_2D_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(FEATURES_DESCRIPTION_2D_SOURCES ${FEATURES_DESCRIPTION_2D_SOURCES} "OrbDescriptor.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(FEATURES_DESCRIPTION_2D_DEPENDENCIES ${FEATURES_DESCRIPTION_2D_DEPENDENCIES} opencv_core opencv_features2d)
endif()

add_library(
    cdff_dfn_features_description_2d
    ${FEATURES_DESCRIPTION_2D_SOURCES}
)
target_include_directories(
    cdff_dfn_features_description_2d
    SYSTEM PRIVATE ${FEATURES_DESCRIPTION_2D_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_features_description_2d
    ${FEATURES_DESCRIPTION_2D_DEPENDENCIES}
)
