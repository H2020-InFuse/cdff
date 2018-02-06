add_library(
    feature_matching_2d
    FeaturesMatching2DInterface.cpp
)

add_library(
    flann_matcher
    FlannMatcher.cpp
)
target_link_libraries(
    flann_matcher
    feature_matching_2d cdff_types cdff_helpers yaml-cpp opencv_features2d opencv_flann
)
