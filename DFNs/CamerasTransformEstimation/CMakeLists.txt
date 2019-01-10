set(CAMERAS_TRANSFORM_ESTIMATION_SOURCES "CamerasTransformEstimationInterface.cpp")
set(CAMERAS_TRANSFORM_ESTIMATION_INCLUDE_DIRS "")
set(CAMERAS_TRANSFORM_ESTIMATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(CAMERAS_TRANSFORM_ESTIMATION_SOURCES ${CAMERAS_TRANSFORM_ESTIMATION_SOURCES} "EssentialMatrixDecomposition.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(CAMERAS_TRANSFORM_ESTIMATION_DEPENDENCIES ${CAMERAS_TRANSFORM_ESTIMATION_DEPENDENCIES} opencv_core)
endif()

add_library(
    cdff_dfn_cameras_transform_estimation
    ${CAMERAS_TRANSFORM_ESTIMATION_SOURCES}
)
target_include_directories(
    cdff_dfn_cameras_transform_estimation
    SYSTEM PRIVATE ${CAMERAS_TRANSFORM_ESTIMATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_cameras_transform_estimation
    ${CAMERAS_TRANSFORM_ESTIMATION_DEPENDENCIES}
)
