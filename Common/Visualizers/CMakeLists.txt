# libcdff_visualizers_opencv and libcdff_visualizers_pcl

add_library(cdff_opencv_visualizer
    OpencvVisualizer.cpp)

target_link_libraries(cdff_opencv_visualizer
    PUBLIC cdff_types opencv_core
    PRIVATE cdff_converters opencv_highgui opencv_imgproc opencv_features2d)

add_library(cdff_pcl_visualizer
    PclVisualizer.cpp)

target_compile_definitions(cdff_pcl_visualizer PRIVATE BOOST_ERROR_CODE_HEADER_ONLY)
target_link_libraries(cdff_pcl_visualizer
    PUBLIC cdff_types ${PCL_COMMON_LIBRARIES}
    PRIVATE cdff_converters ${PCL_IO_LIBRARIES} ${PCL_VISUALIZATION_LIBRARIES})

install(TARGETS cdff_opencv_visualizer cdff_pcl_visualizer
    DESTINATION "${CMAKE_INSTALL_LIBDIR}")
