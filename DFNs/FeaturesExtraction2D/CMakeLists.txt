set(FEATURES_EXTRACTION_2D_SOURCES "FeaturesExtraction2DInterface.cpp")
set(FEATURES_EXTRACTION_2D_INCLUDE_DIRS "")
set(FEATURES_EXTRACTION_2D_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(FEATURES_EXTRACTION_2D_SOURCES ${FEATURES_EXTRACTION_2D_SOURCES} "HarrisDetector2D.cpp" "OrbDetectorDescriptor.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(FEATURES_EXTRACTION_2D_DEPENDENCIES ${FEATURES_EXTRACTION_2D_DEPENDENCIES} ${OpenCV_LIBS})
endif()

add_library(
    cdff_dfn_features_extraction_2d
    ${FEATURES_EXTRACTION_2D_SOURCES}
)
target_include_directories(
    cdff_dfn_features_extraction_2d
    SYSTEM PRIVATE ${FEATURES_EXTRACTION_2D_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_features_extraction_2d
    ${FEATURES_EXTRACTION_2D_DEPENDENCIES}
)
