set(STEREO_RECTIFICATION_SOURCES "StereoRectificationInterface.cpp")
set(STEREO_RECTIFICATION_INCLUDE_DIRS "")
set(STEREO_RECTIFICATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(OpenCV_FOUND)
	set(STEREO_RECTIFICATION_SOURCES ${STEREO_RECTIFICATION_SOURCES} "StereoRectification.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
        set(STEREO_RECTIFICATION_DEPENDENCIES ${STEREO_RECTIFICATION_DEPENDENCIES} opencv_core opencv_imgproc opencv_calib3d)
endif()

if(Edres-Wrapper_FOUND)
	if(WITH_EDRES)
		set(STEREO_RECTIFICATION_SOURCES ${STEREO_RECTIFICATION_SOURCES} "StereoRectificationEdres.cpp")
		set(STEREO_RECTIFICATION_INCLUDE_DIRS ${STEREO_RECTIFICATION_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
		set(STEREO_RECTIFICATION_DEPENDENCIES ${STEREO_RECTIFICATION_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
	endif()
endif()

add_library(
    cdff_dfn_stereo_rectification
    ${STEREO_RECTIFICATION_SOURCES}
)
target_include_directories(
    cdff_dfn_stereo_rectification
    SYSTEM PRIVATE ${STEREO_RECTIFICATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_stereo_rectification
    ${STEREO_RECTIFICATION_DEPENDENCIES}
)
