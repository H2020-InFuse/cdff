add_library(
    kf_correction
    KFCorrectionInterface.cpp
)
set_target_properties(kf_correction PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    kalman_corrector
    KalmanCorrector.cpp
)
set_target_properties(kalman_corrector PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
   kalman_corrector
   kf_correction cdff_types yaml-cpp cdff_helpers opencv_video
)
