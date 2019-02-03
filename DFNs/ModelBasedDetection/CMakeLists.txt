set(MODEL_BASED_DETECTION_SOURCES "ModelBasedDetectionInterface.cpp")
set(MODEL_BASED_DETECTION_INCLUDE_DIRS "")
set(MODEL_BASED_DETECTION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(HAVE_OPENCV_CONTRIB)
    set(MODEL_BASED_DETECTION_SOURCES ${MODEL_BASED_DETECTION_SOURCES} "Linemod.cpp" "linemod-wrapper/LinemodImpl.cpp")
    #OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
    set(MODEL_BASED_DETECTION_DEPENDENCIES ${MODEL_BASED_DETECTION_DEPENDENCIES} opencv_rgbd opencv_core opencv_calib3d opencv_imgproc)
endif()

add_library(
    cdff_dfn_model_based_detection
    ${MODEL_BASED_DETECTION_SOURCES}
)
target_include_directories(
    cdff_dfn_model_based_detection
    SYSTEM PRIVATE ${MODEL_BASED_DETECTION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_model_based_detection
    ${MODEL_BASED_DETECTION_DEPENDENCIES}
)
