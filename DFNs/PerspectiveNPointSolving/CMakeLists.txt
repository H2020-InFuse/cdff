set(PERSPECTIVE_N_POINT_SOLVING_SOURCES "PerspectiveNPointSolvingInterface.cpp")
set(PERSPECTIVE_N_POINT_SOLVING_INCLUDE_DIRS "")
set(PERSPECTIVE_N_POINT_SOLVING_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(PERSPECTIVE_N_POINT_SOLVING_SOURCES ${PERSPECTIVE_N_POINT_SOLVING_SOURCES} "IterativePnpSolver.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(PERSPECTIVE_N_POINT_SOLVING_DEPENDENCIES ${PERSPECTIVE_N_POINT_SOLVING_DEPENDENCIES} opencv_core)
endif()

add_library(
    cdff_dfn_perspective_n_point_solving
    ${PERSPECTIVE_N_POINT_SOLVING_SOURCES}
)
target_include_directories(
    cdff_dfn_perspective_n_point_solving
    SYSTEM PRIVATE ${PERSPECTIVE_N_POINT_SOLVING_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_perspective_n_point_solving
    ${PERSPECTIVE_N_POINT_SOLVING_DEPENDENCIES}
)
