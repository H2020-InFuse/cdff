set(KF_CORRECTION_SOURCES "KFCorrectionInterface.cpp")
set(KF_CORRECTION_INCLUDE_DIRS "")
set(KF_CORRECTION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(KF_CORRECTION_SOURCES ${KF_CORRECTION_SOURCES} "KalmanCorrector.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(KF_CORRECTION_DEPENDENCIES ${KF_CORRECTION_DEPENDENCIES} opencv_core)
endif()

add_library(
    cdff_dfn_kf_correction
    ${KF_CORRECTION_SOURCES}
)
target_include_directories(
    cdff_dfn_kf_correction
    SYSTEM PRIVATE ${KF_CORRECTION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_kf_correction
    ${KF_CORRECTION_DEPENDENCIES}
)
