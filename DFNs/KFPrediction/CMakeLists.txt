add_library(
    kf_prediction
    KFPredictionInterface.cpp
)
set_target_properties(kf_prediction PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    kalman_predictor
    KalmanPredictor.cpp
)
set_target_properties(kalman_predictor PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
   kalman_predictor
   kf_prediction cdff_types yaml-cpp cdff_helpers opencv_core opencv_video
)

