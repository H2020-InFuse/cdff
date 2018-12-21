set(KF_PREDICTION_SOURCES "KFPredictionInterface.cpp")
set(KF_PREDICTION_INCLUDE_DIRS "")
set(KF_PREDICTION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(KF_PREDICTION_SOURCES ${KF_PREDICTION_SOURCES} "KalmanPredictor.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(KF_PREDICTION_DEPENDENCIES ${KF_PREDICTION_DEPENDENCIES} ${OpenCV_LIBS})
endif()

add_library(
    cdff_dfn_kf_prediction
    ${KF_PREDICTION_SOURCES}
)
target_include_directories(
    cdff_dfn_kf_prediction
    SYSTEM PRIVATE ${KF_PREDICTION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_kf_prediction
    ${KF_PREDICTION_DEPENDENCIES}
)

