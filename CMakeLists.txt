project(infuse_dev)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)

#This compiles in testing mode
add_definitions(-DTESTING)

#C11
if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11")
endif()

#This policy disable warning for the use of relative libraries path
cmake_policy(SET CMP0015 NEW)

#Reference directories
include_directories(
    DFNs Common Tests Tools Common/Types/C/ASNCommon Common/Types/C Common/Converters External/opencv3/include External/pcl/include/pcl-1.8/ External/eigen3/include/eigen3/ External/vtk8/include/vtk-8.0/ External/yamlcpp/include/ External/boost/include External/qhull/include External/flann/include Common/Types/Cpp
)
link_directories(
	 External/opencv3/lib External/pcl/lib External/vtk8/lib External/yamlcpp/lib External/boost/lib External/flann/lib Extenal/boost/lib
)


#Main Libraries
add_library(Common Common/Loggers/Logger.cpp Common/Loggers/StandardOutputLogger.cpp Common/Loggers/LoggerFactory.cpp Common/Types/C/VisualPointFeatureVector2D.c Common/Converters/MatToVisualPointFeatureVector2DConverter.cpp Common/Types/C/asn_SEQUENCE_OF.c Common/Types/C/asn_SET_OF.c Common/Converters/VisualPointFeatureVector2DToMatConverter.cpp Common/Types/C/OCTET_STRING.c Common/Types/C/per_opentype.c Common/Types/C/asn_codecs_prim.c Common/Types/C/per_support.c Common/Types/C/NativeReal.c Common/Types/C/NativeInteger.c Common/Types/C/NativeEnumerated.c Common/Types/C/xer_decoder.c Common/Types/C/xer_decoder.c Common/Types/C/ber_tlv_tag.c Common/Types/C/ber_decoder.c Common/Types/C/ber_tlv_length.c Common/Types/C/constraints.c Common/Types/C/BIT_STRING.c Common/Types/C/der_encoder.c Common/Types/C/per_encoder.c Common/Types/C/per_decoder.c Common/Types/C/constr_SEQUENCE_OF.c Common/Types/C/constr_SET_OF.c Common/Types/C/constr_TYPE.c Common/Types/C/constr_SEQUENCE.c Common/Types/C/xer_support.c Common/Types/C/Geometry.c Common/Types/C/Point.c Common/Types/C/Eigen.c
Common/Converters/VisualPointFeatureVector3DToMatConverter.cpp Common/Converters/MatToVisualPointFeatureVector3DConverter.cpp Common/Converters/PclPointCloudToPointCloudConverter.cpp Common/Converters/PointCloudToPclPointCloudConverter.cpp Common/Types/C/Pointcloud.c Common/Types/C/VisualPointFeatureVector3D.c Common/Types/C/VisualPointFeature3D.c Common/Types/C/Point3D.c
Common/Types/C/Frame.c Common/Types/C/Time.c Common/Types/C/asn1crt.c Common/Types/C/taste-types.c Common/Types/C/taste-extended.c Common/Types/Cpp/Frame.cpp Common/Converters/FrameToMatConverter.cpp Common/Converters/MatToFrameConverter.cpp Common/Types/Cpp/VisualPointFeatureVector2D.cpp Common/Types/Cpp/PointCloud.cpp Common/Types/Cpp/VisualPointFeatureVector3D.cpp)
set_target_properties(Common PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ./lib)

add_library(FeaturesExtraction2D DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp)
set_target_properties(FeaturesExtraction2D PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ./lib)
target_link_libraries(
	FeaturesExtraction2D Common tinyxml2 yaml-cpp
)

add_library(FeaturesExtraction3D DFNs/FeaturesExtraction3D/FeaturesExtraction3DInterface.cpp DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp)
set_target_properties(FeaturesExtraction3D PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ./lib)
target_link_libraries(
	FeaturesExtraction3D Common tinyxml2 yaml-cpp
)


#Testing programs
add_library(StubFeaturesExtraction2D Tests/Stubs/DFNs/FeaturesExtraction2D/StubFeaturesExtraction2D.cpp DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp)
set_target_properties(StubFeaturesExtraction2D PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ./test_lib)
target_link_libraries(
	StubFeaturesExtraction2D Common
)

add_executable(HarrisDetector2DTest Tests/UnitTests/DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Tests/Mocks/Common/Converters/FrameToMatConverter.cpp Tests/Mocks/Mock.cpp Tests/Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.cpp DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp)
set_target_properties(HarrisDetector2DTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector2DTest Common opencv_core opencv_imgproc opencv_imgcodecs yaml-cpp
)

add_executable(HarrisDetector2DGuiTest Tests/GuiTests/DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Tests/Mocks/Common/Converters/FrameToMatConverter.cpp Tests/Mocks/Mock.cpp Tests/Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.cpp DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Tests/GuiTests/ParametersInterface.cpp Tests/GuiTests/ButtonsInterface.cpp Tests/GuiTests/MainInterface.cpp Tests/GuiTests/DFNs/DFNTestInterface.cpp)
set_target_properties(HarrisDetector2DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector2DGuiTest Common opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui yaml-cpp
)

add_executable(MatFrameConvertersTest Tests/UnitTests/Common/Converters/MatFrameConvertersTest.cpp )
set_target_properties(MatFrameConvertersTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	MatFrameConvertersTest Common opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(MatVisualPointFeatureVector2DConvertersTest Tests/UnitTests/Common/Converters/MatVisualPointFeatureVector2DConvertersTest.cpp )
set_target_properties(MatVisualPointFeatureVector2DConvertersTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	MatVisualPointFeatureVector2DConvertersTest Common opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(PointCloudPclPointCloudConvertersTest Tests/UnitTests/Common/Converters/PointCloudPclPointCloudConvertersTest.cpp )
set_target_properties(PointCloudPclPointCloudConvertersTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	PointCloudPclPointCloudConvertersTest Common opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(MatVisualPointFeatureVector3DConvertersTest Tests/UnitTests/Common/Converters/MatVisualPointFeatureVector3DConvertersTest.cpp )
set_target_properties(MatVisualPointFeatureVector3DConvertersTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	MatVisualPointFeatureVector3DConvertersTest Common opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(HarrisDetector3DTest Tests/UnitTests/DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp Tests/Mocks/Common/Converters/PointCloudToPclPointCloudConverter.cpp Tests/Mocks/Mock.cpp Tests/Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.cpp DFNs/FeaturesExtraction3D/FeaturesExtraction3DInterface.cpp DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp)
set_target_properties(HarrisDetector3DTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector3DTest Common opencv_core opencv_imgproc opencv_imgcodecs pcl_common pcl_features pcl_search pcl_filters yaml-cpp
)

#add_executable(HarrisDetector3DGuiTest Tests/GuiTests/DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp Tests/Mocks/Common/Converters/PointCloudToPclPointCloudConverter.cpp Tests/Mocks/Mock.cpp Tests/Mocks/Common/Converters/MatToVisualPointFeatureVector3DConverter.cpp DFNs/FeaturesExtraction3D/FeaturesExtraction3DInterface.cpp DFNs/FeaturesExtraction3D/HarrisDetector3D.cpp Tests/GuiTests/ParametersInterface.cpp Tests/GuiTests/ButtonsInterface.cpp Tests/GuiTests/MainInterface.cpp Tests/GuiTests/DFNs/DFNTestInterface.cpp)
#set_target_properties(HarrisDetector3DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
#target_link_libraries(
#	HarrisDetector3DGuiTest Common opencv_core opencv_imgproc opencv_imgcodecs opencv_highgui pcl_common pcl_features pcl_search pcl_filters pcl_visualization pcl_io pcl_ml pcl_io_ply pcl_octree #pcl_outofcore pcl_kdtree pcl_tracking pcl_stereo pcl_recognition pcl_registration pcl_people pcl_keypoints pcl_surface pcl_segmentation pcl_sample_consensus pcl_stereo vtksys vtkCommon boost_system vtkIO vtkViews vtkCharts vtkalglib vtkParallel vtkGeovis vtkRendering vtkInfovis vtkImaging yaml-cpp vtkGraphics vtkGenericFiltering vtkVolumeRendering vtkftgl
#)

#Moving configurations
FILE (COPY Tests/ConfigurationFiles/ DESTINATION ./tests/ConfigurationFiles/)
FILE (COPY Tests/Data/ DESTINATION ./tests/Data/)




#Installation scripts Examples
#install(
#    TARGETS Common FeaturesExtraction2D
#    DESTINATION lib
#)
#
#install(
#    FILES DFNs/FeaturesExtraction2D/HarrisDetector2D.hpp DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp DFNs/DFNCommonInterface.hpp
#    DESTINATION include/DFNs
#)

