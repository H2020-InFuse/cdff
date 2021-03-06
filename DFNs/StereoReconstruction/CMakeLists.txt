set(STEREO_RECONSTRUCTION_SOURCES "StereoReconstructionInterface.cpp")
set(STEREO_RECONSTRUCTION_INCLUDE_DIRS "")
set(STEREO_RECONSTRUCTION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(PCL_FOUND)
	set(STEREO_RECONSTRUCTION_SOURCES ${STEREO_RECONSTRUCTION_SOURCES} "ScanlineOptimization.cpp")
	set(STEREO_RECONSTRUCTION_INCLUDE_DIRS ${STEREO_RECONSTRUCTION_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
	set(STEREO_RECONSTRUCTION_DEPENDENCIES ${STEREO_RECONSTRUCTION_DEPENDENCIES} ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_STEREO_LIBRARIES})

	if(OpenCV_FOUND)
		set(STEREO_RECONSTRUCTION_SOURCES ${STEREO_RECONSTRUCTION_SOURCES} "DisparityMapping.cpp" "HirschmullerDisparityMapping.cpp")
		#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
		set(STEREO_RECONSTRUCTION_DEPENDENCIES ${STEREO_RECONSTRUCTION_DEPENDENCIES} opencv_core opencv_calib3d ${PCL_FILTERS_LIBRARIES})
	endif()

endif()


if(TEST_ENABLED)
	set(STEREO_RECONSTRUCTION_DEPENDENCIES ${STEREO_RECONSTRUCTION_DEPENDENCIES} "cdff_visualizers_opencv")
endif()

add_library(
    cdff_dfn_stereo_reconstruction
    ${STEREO_RECONSTRUCTION_SOURCES}
)
target_include_directories(
    cdff_dfn_stereo_reconstruction
    SYSTEM PRIVATE ${STEREO_RECONSTRUCTION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_stereo_reconstruction
    ${STEREO_RECONSTRUCTION_DEPENDENCIES}
)
