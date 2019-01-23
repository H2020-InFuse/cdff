set(BUNDLE_ADJUSTMENT_SOURCES "BundleAdjustmentInterface.cpp")
set(BUNDLE_ADJUSTMENT_INCLUDE_DIRS "")
set(BUNDLE_ADJUSTMENT_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(BUNDLE_ADJUSTMENT_SOURCES ${BUNDLE_ADJUSTMENT_SOURCES} "SvdDecomposition.cpp" "SvdDecomposition.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(BUNDLE_ADJUSTMENT_DEPENDENCIES ${BUNDLE_ADJUSTMENT_DEPENDENCIES} opencv_core)
endif()

if(CERES_FOUND)
	set(BUNDLE_ADJUSTMENT_SOURCES ${BUNDLE_ADJUSTMENT_SOURCES} "CeresAdjustment.cpp")
	set(BUNDLE_ADJUSTMENT_INCLUDE_DIRS ${BUNDLE_ADJUSTMENT_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})
	set(BUNDLE_ADJUSTMENT_DEPENDENCIES ${BUNDLE_ADJUSTMENT_DEPENDENCIES} ${CERES_LIBRARIES})
endif()

add_library(
    cdff_dfn_bundle_adjustment
    ${BUNDLE_ADJUSTMENT_SOURCES}
)
target_include_directories(
    cdff_dfn_bundle_adjustment
    SYSTEM PRIVATE ${BUNDLE_ADJUSTMENT_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_bundle_adjustment
    ${BUNDLE_ADJUSTMENT_DEPENDENCIES}
)
