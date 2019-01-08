set(PRIMITIVE_FINDER_SOURCES "PrimitiveFinderInterface.cpp")
set(PRIMITIVE_FINDER_INCLUDE_DIRS "")
set(PRIMITIVE_FINDER_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
    set(PRIMITIVE_FINDER_SOURCES ${PRIMITIVE_FINDER_SOURCES} "BasicPrimitiveFinder.cpp")
    #OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
    set(PRIMITIVE_FINDER_DEPENDENCIES ${PRIMITIVE_FINDER_DEPENDENCIES} opencv_imgproc opencv_core)
endif()

add_library(
        cdff_dfn_primitive_finder
        ${PRIMITIVE_FINDER_SOURCES}
)
target_include_directories(
        cdff_dfn_primitive_finder
        SYSTEM PRIVATE ${PRIMITIVE_FINDER_INCLUDE_DIRS}
)
target_link_libraries(
        cdff_dfn_primitive_finder
        ${PRIMITIVE_FINDER_DEPENDENCIES}
)