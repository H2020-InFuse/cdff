set(FEATURES_MATCHING_3D_SOURCES "FeaturesMatching3DInterface.cpp" "BestDescriptorMatch.cpp")
set(FEATURES_MATCHING_3D_INCLUDE_DIRS "")
set(FEATURES_MATCHING_3D_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(PCL_FOUND)
	set(FEATURES_MATCHING_3D_SOURCES ${FEATURES_MATCHING_3D_SOURCES} "Ransac3D.cpp" "Icp3D.cpp")
	set(FEATURES_MATCHING_3D_INCLUDE_DIRS ${FEATURES_MATCHING_3D_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
	set(FEATURES_MATCHING_3D_DEPENDENCIES ${FEATURES_MATCHING_3D_DEPENDENCIES}  ${PCL_COMMON_LIBRARIES} ${PCL_FEATURES_LIBRARIES} ${PCL_KDTREE_LIBRARIES} ${PCL_SEARCH_LIBRARIES})
endif()

add_library(
    cdff_dfn_features_matching_3d
    ${FEATURES_MATCHING_3D_SOURCES}
)
target_include_directories(
    cdff_dfn_features_matching_3d
    SYSTEM PRIVATE ${FEATURES_MATCHING_3D_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_features_matching_3d
    ${FEATURES_MATCHING_3D_DEPENDENCIES}
)
