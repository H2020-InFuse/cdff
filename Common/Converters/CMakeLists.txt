add_library(
    converters_opencv
    ImageTypeToMatConverter.cpp
    MatToImageTypeConverter.cpp
    MatToVisualPointFeatureVector2DConverter.cpp
    VisualPointFeatureVector2DToMatConverter.cpp
)
target_link_libraries(converters_opencv opencv_core)
install(
    TARGETS converters_opencv
    DESTINATION lib
)
