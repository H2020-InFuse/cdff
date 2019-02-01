set(MODELBASEDVISUALTRACKING_SOURCES ModelBasedVisualTrackingInterface.cpp)
set(MODELBASEDVISUALTRACKING_INCLUDE_DIRS "")
set(MODELBASEDVISUALTRACKING_DEPENDENCIES "")

if(WITH_DLRTRACKER AND OpenCV_FOUND)
    list(APPEND MODELBASEDVISUALTRACKING_SOURCES EdgeModelContourMatching.cpp)
    list(APPEND MODELBASEDVISUALTRACKING_INCLUDE_DIRS ${DLRTRACKER-CORE_INCLUDE_DIRS})
    list(APPEND MODELBASEDVISUALTRACKING_DEPENDENCIES opencv_imgproc opencv_core opencv_imgcodecs opencv_features2d ${DLRTRACKER-CORE_LIBRARIES})
endif()

add_library(cdff_dfpc_model_based_visual_tracking
    ${MODELBASEDVISUALTRACKING_SOURCES})

target_include_directories(cdff_dfpc_model_based_visual_tracking
    SYSTEM PUBLIC ${MODELBASEDVISUALTRACKING_INCLUDE_DIRS})

target_link_libraries(cdff_dfpc_model_based_visual_tracking
    PUBLIC ${MODELBASEDVISUALTRACKING_DEPENDENCIES})
