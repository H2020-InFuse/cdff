set(IMAGE_FILTERING_SOURCES "ImageFilteringInterface.cpp")
set(IMAGE_FILTERING_INCLUDE_DIRS "")
set(IMAGE_FILTERING_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(IMAGE_FILTERING_SOURCES ${IMAGE_FILTERING_SOURCES} "ImageUndistortion.cpp" "ImageUndistortionRectification.cpp" "DerivativeEdgeDetection.cpp" "CannyEdgeDetection.cpp"
		"BackgroundExtraction.cpp" "NormalVectorExtraction.cpp" "KMeansClustering.cpp" "BackgroundSubtractorMOG2.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(IMAGE_FILTERING_DEPENDENCIES ${IMAGE_FILTERING_DEPENDENCIES} ${OpenCV_LIBS} cdff_validators)
endif()

add_library(
    cdff_dfn_image_filtering
    ${IMAGE_FILTERING_SOURCES}
)
target_include_directories(
    cdff_dfn_image_filtering
    SYSTEM PRIVATE ${IMAGE_FILTERING_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_image_filtering
    ${IMAGE_FILTERING_DEPENDENCIES}
)
