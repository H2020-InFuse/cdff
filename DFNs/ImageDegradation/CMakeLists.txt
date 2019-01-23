set(IMAGE_DEGRADATION_SOURCES "ImageDegradationInterface.cpp")
set(IMAGE_DEGRADATION_INCLUDE_DIRS "")
set(IMAGE_DEGRADATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(OpenCV_FOUND)
	set(IMAGE_DEGRADATION_SOURCES ${IMAGE_DEGRADATION_SOURCES} "ImageDegradation.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
        set(IMAGE_DEGRADATION_DEPENDENCIES ${IMAGE_DEGRADATION_DEPENDENCIES} opencv_core opencv_imgproc)
endif()

if(Edres-Wrapper_FOUND)
	set(IMAGE_DEGRADATION_SOURCES ${IMAGE_DEGRADATION_SOURCES} "ImageDegradationEdres.cpp")
	set(IMAGE_DEGRADATION_INCLUDE_DIRS ${IMAGE_DEGRADATION_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
	set(IMAGE_DEGRADATION_DEPENDENCIES ${IMAGE_DEGRADATION_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
endif()

add_library(
    cdff_dfn_image_degradation
    ${IMAGE_DEGRADATION_SOURCES}
)
target_include_directories(
    cdff_dfn_image_degradation
    SYSTEM PRIVATE ${IMAGE_DEGRADATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_image_degradation
    ${IMAGE_DEGRADATION_DEPENDENCIES}
)
