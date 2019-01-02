set(DISPARITY_TO_POINTCLOUD_SOURCES "DisparityToPointCloudWithIntensityInterface.cpp")
set(DISPARITY_TO_POINTCLOUD_INCLUDE_DIRS "")
set(DISPARITY_TO_POINTCLOUD_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(DISPARITY_TO_POINTCLOUD_SOURCES ${DISPARITY_TO_POINTCLOUD_SOURCES} "DisparityToPointCloudWithIntensity.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(DISPARITY_TO_POINTCLOUD_DEPENDENCIES ${DISPARITY_TO_POINTCLOUD_DEPENDENCIES} ${OpenCV_LIBS})
endif()

if(Edres-Wrapper_FOUND)
	if(WITH_EDRES)
		set(DISPARITY_TO_POINTCLOUD_SOURCES ${DISPARITY_TO_POINTCLOUD_SOURCES} "DisparityToPointCloudWithIntensityEdres.cpp")
		set(DISPARITY_TO_POINTCLOUD_INCLUDE_DIRS ${DISPARITY_TO_POINTCLOUD_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
		set(DISPARITY_TO_POINTCLOUD_DEPENDENCIES ${DISPARITY_TO_POINTCLOUD_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
	endif()
endif()

add_library(
    cdff_dfn_disparity_to_pointcloud_with_intensity
    ${DISPARITY_TO_POINTCLOUD_SOURCES}
)
target_include_directories(
    cdff_dfn_disparity_to_pointcloud_with_intensity
    SYSTEM PRIVATE ${DISPARITY_TO_POINTCLOUD_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_disparity_to_pointcloud_with_intensity
    ${DISPARITY_TO_POINTCLOUD_DEPENDENCIES}
)
