set(VOXELIZATION_SOURCES "VoxelizationInterface.cpp")
set(VOXELIZATION_INCLUDE_DIRS "")
set(VOXELIZATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(VOXELIZATION_SOURCES ${VOXELIZATION_SOURCES} "Octree.cpp")
	set(VOXELIZATION_INCLUDE_DIRS ${VOXELIZATION_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
	set(VOXELIZATION_DEPENDENCIES ${VOXELIZATION_DEPENDENCIES} ${OpenCV_LIBRARIES})
endif()

add_library(
    cdff_dfn_voxelization
    ${VOXELIZATION_SOURCES}
)
target_include_directories(
    cdff_dfn_voxelization
    SYSTEM PRIVATE ${VOXELIZATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_voxelization
    ${VOXELIZATION_DEPENDENCIES}
)
