set(sources ModelBasedVisualTrackingInterface.cpp)
set(includedirectories_public "")
set(includedirectories_private "")
set(linklibraries_public cdff_types)
set(linklibraries_private "")

if(DLRTRACKER-CORE_FOUND AND OpenCV_FOUND)
    list(APPEND sources EdgeModelContourMatching.cpp)
    list(APPEND includedirectories_public ${DLRTRACKER-CORE_INCLUDE_DIRS})
    list(APPEND linklibraries_public cdff_types cdff_converters ${DLRTRACKER-CORE_LIBRARIES})
    list(APPEND linklibraries_private cdff_logger opencv_imgproc opencv_core opencv_imgcodecs opencv_highgui)
endif()

add_library(cdff_dfpc_model_based_visual_tracking
    ${sources})

target_include_directories(cdff_dfpc_model_based_visual_tracking
    SYSTEM PUBLIC ${includedirectories_public})

target_link_libraries(cdff_dfpc_model_based_visual_tracking
    PUBLIC ${linklibraries_public} PRIVATE ${linklibraries_private})
