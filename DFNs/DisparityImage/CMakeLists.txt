set(DISPARITY_IMAGE_SOURCES "DisparityImageInterface.cpp")
set(DISPARITY_IMAGE_INCLUDE_DIRS "")
set(DISPARITY_IMAGE_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(DISPARITY_IMAGE_SOURCES ${DISPARITY_IMAGE_SOURCES} "DisparityImage.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(DISPARITY_IMAGE_DEPENDENCIES ${DISPARITY_IMAGE_DEPENDENCIES} ${OpenCV_LIBS})
endif()

if(Edres-Wrapper_FOUND)
	if(WITH_EDRES)
		set(DISPARITY_IMAGE_SOURCES ${DISPARITY_IMAGE_SOURCES} "DisparityImageEdres.cpp")
		set(DISPARITY_IMAGE_INCLUDE_DIRS ${DISPARITY_IMAGE_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
		set(DISPARITY_IMAGE_DEPENDENCIES ${DISPARITY_IMAGE_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
	endif()
endif()

add_library(
    cdff_dfn_disparity_image
    ${DISPARITY_IMAGE_SOURCES}
)
target_include_directories(
    cdff_dfn_disparity_image
    SYSTEM PRIVATE ${DISPARITY_IMAGE_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_disparity_image
    ${DISPARITY_IMAGE_DEPENDENCIES}
)
