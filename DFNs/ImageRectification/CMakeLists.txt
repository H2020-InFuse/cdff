set(IMAGE_RECTIFICATION_SOURCES "ImageRectificationInterface.cpp")
set(IMAGE_RECTIFICATION_INCLUDE_DIRS "")
set(IMAGE_RECTIFICATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(OpenCV_FOUND)
	set(IMAGE_RECTIFICATION_SOURCES ${IMAGE_RECTIFICATION_SOURCES} "ImageRectification.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
        set(IMAGE_RECTIFICATION_DEPENDENCIES ${IMAGE_RECTIFICATION_DEPENDENCIES} opencv_core opencv_imgproc opencv_calib3d )
endif()

if(Edres-Wrapper_FOUND)
	if(WITH_EDRES)
		set(IMAGE_RECTIFICATION_SOURCES ${IMAGE_RECTIFICATION_SOURCES} "ImageRectificationEdres.cpp")
		set(IMAGE_RECTIFICATION_INCLUDE_DIRS ${IMAGE_RECTIFICATION_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
		set(IMAGE_RECTIFICATION_DEPENDENCIES ${IMAGE_RECTIFICATION_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
	endif()
endif()

add_library(
    cdff_dfn_image_rectification
    ${IMAGE_RECTIFICATION_SOURCES}
)
target_include_directories(
    cdff_dfn_image_rectification
    SYSTEM PRIVATE ${IMAGE_RECTIFICATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_image_rectification
    ${IMAGE_RECTIFICATION_DEPENDENCIES}
)
