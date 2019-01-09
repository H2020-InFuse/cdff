set(DISPARITY_IMAGE_SOURCES "DisparityImageInterface.cpp")
set(DISPARITY_IMAGE_INCLUDE_DIRS "")
set(DISPARITY_IMAGE_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(DISPARITY_IMAGE_SOURCES ${DISPARITY_IMAGE_SOURCES} "DisparityImage.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(DISPARITY_IMAGE_DEPENDENCIES ${DISPARITY_IMAGE_DEPENDENCIES} ${OpenCV_LIBS})

        set(OPENCV_XIMGPROC_FOUND FALSE CACHE INTERNAL "Can #include <opencv2/ximgproc/ximgproc.hpp>")
        include(CheckIncludeFileCXX)
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

if(OpenCV_FOUND)
    find_package(OpenCV)

    # Checking for contrib module ximgproc
    check_include_file_cxx("opencv2/ximgproc/ximgproc.hpp" OPENCV_XIMGPROC_FOUND)

    if(OPENCV_XIMGPROC_FOUND)
        target_compile_definitions(cdff_dfn_disparity_image PUBLIC WITH_XIMGPROC=${OPENCV_XIMGPROC_FOUND} PRIVATE BUILD_TESTS=1 )
    endif(OPENCV_XIMGPROC_FOUND)
endif()
