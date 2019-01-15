# libcdff_converters

set(converters
    MatToFrameConverter.cpp
    FrameToMatConverter.cpp
    MatToVisualPointFeatureVector2DConverter.cpp
    VisualPointFeatureVector2DToMatConverter.cpp
    MatToVisualPointFeatureVector3DConverter.cpp
    VisualPointFeatureVector3DToMatConverter.cpp
    PclPointCloudToVisualPointFeatureVector3DConverter.cpp
    VisualPointFeatureVector3DToPclPointCloudConverter.cpp
    PclPointCloudToPointCloudConverter.cpp
    PointCloudToPclPointCloudConverter.cpp
    EigenTransformToTransform3DConverter.cpp
    Transform3DToEigenTransformConverter.cpp
    MatToTransform3DConverter.cpp
    Transform3DToMatConverter.cpp
    PclNormalsCloudToPointCloudConverter.cpp
    PointCloudToPclNormalsCloudConverter.cpp
    MatToCorrespondenceMaps2DSequenceConverter.cpp
    CorrespondenceMaps2DSequenceToMatConverter.cpp
    MatToCorrespondenceMaps3DSequenceConverter.cpp
    CorrespondenceMaps3DSequenceToMatConverter.cpp
    StdVectorOfStringsToStringSequenceConverter.cpp
    StringSequenceToStdVectorOfStringsConverter.cpp
    PclOctreeToOctreeConverter.cpp
    OctreeToPclOctreeConverter.cpp)

add_library(cdff_converters
    ${converters})

target_link_libraries(cdff_converters
    PUBLIC cdff_types Eigen3::Eigen opencv_core ${PCL_COMMON_LIBRARIES} ${PCL_OCTREE_LIBRARIES}
    PRIVATE cdff_logger Boost::boost)

install(TARGETS cdff_converters
    DESTINATION "${CMAKE_INSTALL_LIBDIR}")
