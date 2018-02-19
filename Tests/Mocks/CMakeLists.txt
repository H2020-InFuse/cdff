add_library(  # TODO split up in multiple mocks
    mock
    Mock.cpp
    Common/Converters/FrameToMatConverter.cpp
    Common/Converters/MatToFrameConverter.cpp
    Common/Converters/MatToVisualPointFeatureVector2DConverter.cpp
    Common/Converters/MatToVisualPointFeatureVector3DConverter.cpp
    Common/Converters/PointCloudToPclPointCloudConverter.cpp
    Common/Converters/VisualPointFeatureVector3DToPclPointCloudConverter.cpp
    Common/Converters/EigenTransformToTransform3DConverter.cpp
    Common/Converters/VisualPointFeatureVector2DToMatConverter.cpp
    Common/Converters/PointCloudToPclNormalsCloudConverter.cpp
)
