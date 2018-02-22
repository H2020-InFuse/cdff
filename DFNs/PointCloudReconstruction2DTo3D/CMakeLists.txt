add_library(
    point_cloud_reconstruction_2d_to_3d
    PointCloudReconstruction2DTo3DInterface.cpp
)

add_library(
    triangulation
    Triangulation.cpp
)
target_link_libraries(
    triangulation
     point_cloud_reconstruction_2d_to_3d cdff_types yaml-cpp opencv_calib3d cdff_helpers
)
