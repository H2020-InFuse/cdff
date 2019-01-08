set(WEIGHTING_EXPERT_SOURCES "WeightingExpertInterface.cpp")
set(WEIGHTING_EXPERT_INCLUDE_DIRS "")
set(WEIGHTING_EXPERT_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
    set(WEIGHTING_EXPERT_SOURCES ${WEIGHTING_EXPERT_SOURCES} "PoseWeighting.cpp")
    #OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
    set(WEIGHTING_EXPERT_DEPENDENCIES ${WEIGHTING_EXPERT_DEPENDENCIES} opencv_highgui opencv_video)
endif()

add_library(
        cdff_dfn_weighting_expert
        ${WEIGHTING_EXPERT_SOURCES}
)
target_include_directories(
        cdff_dfn_weighting_expert
        SYSTEM PRIVATE ${WEIGHTING_EXPERT_INCLUDE_DIRS}
)
target_link_libraries(
        cdff_dfn_weighting_expert
        ${WEIGHTING_EXPERT_DEPENDENCIES}
)