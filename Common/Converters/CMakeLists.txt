add_library(
    cdff_converters
    FrameToMatConverter.cpp
    MatToFrameConverter.cpp
    FrameToImageConverter.cpp
    MatToVisualPointFeatureVector2DConverter.cpp
    VisualPointFeatureVector2DToMatConverter.cpp
    MatToVisualPointFeatureVector3DConverter.cpp
    VisualPointFeatureVector3DToMatConverter.cpp
    VisualPointFeatureVector3DToPclPointCloudConverter.cpp
    PclPointCloudToVisualPointFeatureVector3DConverter.cpp
    PointCloudToPclPointCloudConverter.cpp
    PclPointCloudToPointCloudConverter.cpp
    Transform3DToEigenTransformConverter.cpp
    EigenTransformToTransform3DConverter.cpp
    Transform3DToMatConverter.cpp
    MatToTransform3DConverter.cpp
    PointCloudToPclNormalsCloudConverter.cpp
    PclNormalsCloudToPointCloudConverter.cpp  
    CorrespondenceMaps2DSequenceToMatConverter.cpp
    MatToCorrespondenceMaps2DSequenceConverter.cpp
    CorrespondenceMaps3DSequenceToMatConverter.cpp
    MatToCorrespondenceMaps3DSequenceConverter.cpp
    StringSequenceToStdVectorOfStringsConverter.cpp
    StdVectorOfStringsToStringSequenceConverter.cpp
    PclOctreeToOctreeConverter.cpp
    OctreeToPclOctreeConverter.cpp
)
target_link_libraries(cdff_converters opencv_core cdff_types ${PCL_OCTREE_LIBRARIES})

install(
    TARGETS cdff_converters
    DESTINATION lib
)
