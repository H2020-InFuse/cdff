set(IMAGE_PAIR_DEGRADATION_SOURCES "ImagePairDegradationInterface.cpp")
set(IMAGE_PAIR_DEGRADATION_INCLUDE_DIRS "")
set(IMAGE_PAIR_DEGRADATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(OpenCV_FOUND)
	set(IMAGE_PAIR_DEGRADATION_SOURCES ${IMAGE_PAIR_DEGRADATION_SOURCES} "ImagePairDegradation.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(IMAGE_PAIR_DEGRADATION_DEPENDENCIES ${IMAGE_PAIR_DEGRADATION_DEPENDENCIES} ${OpenCV_LIBS})
endif()

if(Edres-Wrapper_FOUND)
	if(WITH_EDRES)
		set(IMAGE_PAIR_DEGRADATION_SOURCES ${IMAGE_PAIR_DEGRADATION_SOURCES} "ImagePairDegradationEdres.cpp")
		set(IMAGE_PAIR_DEGRADATION_INCLUDE_DIRS ${IMAGE_PAIR_DEGRADATION_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
		set(IMAGE_PAIR_DEGRADATION_DEPENDENCIES ${IMAGE_PAIR_DEGRADATION_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
	endif()
endif()

add_library(
    cdff_dfn_image_pair_degradation
    ${IMAGE_PAIR_DEGRADATION_SOURCES}
)
target_include_directories(
    cdff_dfn_image_pair_degradation
    SYSTEM PRIVATE ${IMAGE_PAIR_DEGRADATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_image_pair_degradation
    ${IMAGE_PAIR_DEGRADATION_DEPENDENCIES}
)
