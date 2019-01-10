set(STEREO_DEGRADATION_SOURCES "StereoDegradationInterface.cpp")
set(STEREO_DEGRADATION_INCLUDE_DIRS "")
set(STEREO_DEGRADATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(OpenCV_FOUND)
	set(STEREO_DEGRADATION_SOURCES ${STEREO_DEGRADATION_SOURCES} "StereoDegradation.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
        set(STEREO_DEGRADATION_DEPENDENCIES ${STEREO_DEGRADATION_DEPENDENCIES} opencv_core opencv_imgproc)
endif()

if(Edres-Wrapper_FOUND)
	if(WITH_EDRES)
		set(STEREO_DEGRADATION_SOURCES ${STEREO_DEGRADATION_SOURCES} "StereoDegradationEdres.cpp")
		set(STEREO_DEGRADATION_INCLUDE_DIRS ${STEREO_DEGRADATION_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
		set(STEREO_DEGRADATION_DEPENDENCIES ${STEREO_DEGRADATION_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
	endif()
endif()

add_library(
    cdff_dfn_stereo_degradation
    ${STEREO_DEGRADATION_SOURCES}
)
target_include_directories(
    cdff_dfn_stereo_degradation
    SYSTEM PRIVATE ${STEREO_DEGRADATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_stereo_degradation
    ${STEREO_DEGRADATION_DEPENDENCIES}
)
