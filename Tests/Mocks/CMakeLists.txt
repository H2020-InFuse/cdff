add_library(  # TODO split up in multiple mocks
    mock
    Mock.cpp
    Common/Converters/FrameToMatConverter.cpp
    Common/Converters/MatToVisualPointFeatureVector2DConverter.cpp
    Common/Converters/MatToVisualPointFeatureVector3DConverter.cpp
    Common/Converters/PointCloudToPclPointCloudConverter.cpp
)
