set(LINEMOD_DETECT_SOURCES "LinemodDetectInterface.cpp")
set(LINEMOD_DETECT_INCLUDE_DIRS "")
set(LINEMOD_DETECT_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(WITH_OPENCV_CONTRIB AND VTK_FOUND)
    set(LINEMOD_DETECT_SOURCES ${LINEMOD_DETECT_SOURCES} "LinemodDetect.cpp" "linemod-wrapper/LinemodDetectImpl.cpp")
    #OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
    set(LINEMOD_DETECT_DEPENDENCIES ${LINEMOD_DETECT_DEPENDENCIES} ${VTK_LIBRARIES} opencv_rgbd opencv_core opencv_calib3d opencv_imgproc)
endif()

add_library(
    cdff_dfn_linemod_detect
    ${LINEMOD_DETECT_SOURCES}
)
target_include_directories(
    cdff_dfn_linemod_detect
    SYSTEM PRIVATE ${LINEMOD_DETECT_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_linemod_detect
    ${LINEMOD_DETECT_DEPENDENCIES}
)
