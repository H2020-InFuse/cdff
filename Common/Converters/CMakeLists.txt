add_library(
    converters_opencv
    FrameToMatConverter.cpp
    MatToFrameConverter.cpp
    MatToVisualPointFeatureVector2DConverter.cpp
    VisualPointFeatureVector2DToMatConverter.cpp
    MatToVisualPointFeatureVector3DConverter.cpp
    VisualPointFeatureVector3DToMatConverter.cpp
    VisualPointFeatureVector3DToPclPointCloudConverter.cpp
    PclPointCloudToVisualPointFeatureVector3DConverter.cpp
    PointCloudToPclPointCloudConverter.cpp
    PclPointCloudToPointCloudConverter.cpp
)
target_link_libraries(converters_opencv opencv_core cdff_types)

install(
    TARGETS converters_opencv
    DESTINATION lib
)
