add_executable(
    HarrisDetector2DGuiTest
    DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(HarrisDetector2DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector2DGuiTest
	mock cdff_logger harris_detector_2d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui
)

add_executable(
    HarrisDetector3DGuiTest
    DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp
    ParametersInterface.cpp
    ButtonsInterface.cpp
    MainInterface.cpp
    DFNs/DFNTestInterface.cpp
)
set_target_properties(HarrisDetector3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector3DGuiTest
	 harris_detector_3d mock cdff_logger harris_detector_2d converters_opencv opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo vtksys vtkCommon boost_system vtkIO vtkViews vtkCharts vtkalglib vtkParallel vtkGeovis vtkRendering vtkInfovis vtkImaging yaml-cpp vtkGraphics vtkGenericFiltering vtkVolumeRendering vtkftgl
)
