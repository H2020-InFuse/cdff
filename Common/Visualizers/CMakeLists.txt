add_library(
    cdff_opencv_visualizer
    OpencvVisualizer.cpp
)
target_link_libraries(
    cdff_opencv_visualizer
    cdff_logger cdff_types opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui converters_opencv opencv_imgcodecs opencv_features2d
)
install(
    TARGETS cdff_opencv_visualizer
    DESTINATION lib
)

add_library(
    cdff_pcl_visualizer
    PclVisualizer.cpp
)
target_link_libraries(
    cdff_pcl_visualizer
    converters_opencv cdff_logger cdff_types pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo boost_system ${VTK_LIBRARIES}
)
install(
    TARGETS cdff_pcl_visualizer
    DESTINATION lib
)
