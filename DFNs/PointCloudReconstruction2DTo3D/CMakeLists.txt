set(POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_SOURCES "PointCloudReconstruction2DTo3DInterface.cpp")
set(POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_INCLUDE_DIRS "")
set(POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_SOURCES ${POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_SOURCES} "Triangulation.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_DEPENDENCIES ${POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_DEPENDENCIES} opencv_core opencv_calib3d)
endif()

add_library(
    cdff_dfn_point_cloud_reconstruction_2d_to_3d
    ${POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_SOURCES}
)
target_include_directories(
    cdff_dfn_point_cloud_reconstruction_2d_to_3d
    SYSTEM PRIVATE ${POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_point_cloud_reconstruction_2d_to_3d
    ${POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_DEPENDENCIES}
)
