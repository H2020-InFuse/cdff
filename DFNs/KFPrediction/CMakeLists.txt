add_library(
    kf_prediction
    KFPredictionInterface.cpp
)

add_library(
    kalman_predictor
    KalmanPredictor.cpp
)
target_link_libraries(
   kalman_predictor
   kf_prediction cdff_types yaml-cpp cdff_helpers opencv_imgproc opencv_core opencv_imgcodecs opencv_video
)

