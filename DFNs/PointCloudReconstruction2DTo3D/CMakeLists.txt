add_library(
    point_cloud_reconstruction_2d_to_3d
    PointCloudReconstruction2DTo3DInterface.cpp
    PointCloudReconstruction2DTo3DExecutor.cpp
)
set_target_properties(point_cloud_reconstruction_2d_to_3d PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    triangulation
    Triangulation.cpp
)
set_target_properties(triangulation PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    triangulation
    point_cloud_reconstruction_2d_to_3d cdff_types yaml-cpp opencv_calib3d cdff_helpers opencv_core converters_opencv
)
