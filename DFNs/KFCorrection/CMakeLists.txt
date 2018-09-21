add_library(
    kf_correction
    KFCorrectionInterface.cpp
)

add_library(
    kalman_corrector
    KalmanCorrector.cpp
)
target_link_libraries(
   kalman_corrector
   kf_correction cdff_types yaml-cpp cdff_helpers opencv_video
)
