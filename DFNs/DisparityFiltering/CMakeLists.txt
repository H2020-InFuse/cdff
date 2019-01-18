set(DISPARITY_FILTERING_SOURCES "DisparityFilteringInterface.cpp")
set(DISPARITY_FILTERING_INCLUDE_DIRS "")
set(DISPARITY_FILTERING_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(Edres-Wrapper_FOUND)
	set(DISPARITY_FILTERING_SOURCES ${DISPARITY_FILTERING_SOURCES} "DisparityFilteringEdres.cpp")
	set(DISPARITY_FILTERING_INCLUDE_DIRS ${DISPARITY_FILTERING_INCLUDE_DIRS} ${Edres-Wrapper_INCLUDE_DIRS})
	set(DISPARITY_FILTERING_DEPENDENCIES ${DISPARITY_FILTERING_DEPENDENCIES} ${Edres-Wrapper_LIBRARIES})
endif()

add_library(
    cdff_dfn_disparity_filtering
    ${DISPARITY_FILTERING_SOURCES}
)
target_include_directories(
    cdff_dfn_disparity_filtering
    SYSTEM PRIVATE ${DISPARITY_FILTERING_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_disparity_filtering
    ${DISPARITY_FILTERING_DEPENDENCIES}
)
