set(TRANSFORM_3D_ESTIMATION_SOURCES "Transform3DEstimationInterface.cpp")
set(TRANSFORM_3D_ESTIMATION_INCLUDE_DIRS "")
set(TRANSFORM_3D_ESTIMATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(TRANSFORM_3D_ESTIMATION_SOURCES ${TRANSFORM_3D_ESTIMATION_SOURCES} "LeastSquaresMinimization.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(TRANSFORM_3D_ESTIMATION_DEPENDENCIES ${TRANSFORM_3D_ESTIMATION_DEPENDENCIES} opencv_core)
endif()

if(CERES_FOUND)
	set(TRANSFORM_3D_ESTIMATION_SOURCES ${TRANSFORM_3D_ESTIMATION_SOURCES} "CeresEstimation.cpp")
	set(TRANSFORM_3D_ESTIMATION_INCLUDE_DIRS ${TRANSFORM_3D_ESTIMATION_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
	set(TRANSFORM_3D_ESTIMATION_DEPENDENCIES ${TRANSFORM_3D_ESTIMATION_DEPENDENCIES} ${CERES_LIBRARIES})
endif()

add_library(
    cdff_dfn_transform_3d_estimation
    ${TRANSFORM_3D_ESTIMATION_SOURCES}
)
target_include_directories(
    cdff_dfn_transform_3d_estimation
    SYSTEM PRIVATE ${TRANSFORM_3D_ESTIMATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_transform_3d_estimation
    ${TRANSFORM_3D_ESTIMATION_DEPENDENCIES}
)
