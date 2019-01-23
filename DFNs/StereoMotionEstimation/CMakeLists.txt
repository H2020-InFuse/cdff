set(STEREO_MOTION_ESTIMATION_SOURCES "StereoMotionEstimationInterface.cpp")
set(STEREO_MOTION_ESTIMATION_INCLUDE_DIRS "")
set(STEREO_MOTION_ESTIMATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(Edres-Wrapper_FOUND)
	set(STEREO_MOTION_ESTIMATION_SOURCES ${STEREO_MOTION_ESTIMATION_SOURCES} "StereoMotionEstimationEdres.cpp")
	set(STEREO_MOTION_ESTIMATION_INCLUDE_DIRS ${STEREO_MOTION_ESTIMATION_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
	set(STEREO_MOTION_ESTIMATION_DEPENDENCIES ${STEREO_MOTION_ESTIMATION_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
endif()

add_library(
    cdff_dfn_stereo_motion_estimation
    ${STEREO_MOTION_ESTIMATION_SOURCES}
)
target_include_directories(
    cdff_dfn_stereo_motion_estimation
    SYSTEM PRIVATE ${STEREO_MOTION_ESTIMATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_stereo_motion_estimation
    ${STEREO_MOTION_ESTIMATION_DEPENDENCIES}
)
