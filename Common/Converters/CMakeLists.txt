add_library(
    converters_opencv
    ImageTypeToMatConverter.cpp
    MatToImageTypeConverter.cpp
    MatToVisualPointFeatureVector2DConverter.cpp
    VisualPointFeatureVector2DToMatConverter.cpp
)
# TODO we have to link to some OpenCV lib
#target_link_libraries(opencv_converters TODO)
install(
    TARGETS converters_opencv
    DESTINATION lib
)
