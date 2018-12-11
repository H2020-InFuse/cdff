set(PERSPECTIVE_N_POINT_SOLVING_SOURCES "PerspectiveNPointSolvingInterface.cpp")
set(PERSPECTIVE_N_POINT_SOLVING_INCLUDE_DIRS "")
set(PERSPECTIVE_N_POINT_SOLVING_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(PERSPECTIVE_N_POINT_SOLVING_SOURCES ${PERSPECTIVE_N_POINT_SOLVING_SOURCES} "IterativePnpSolver.cpp")
	set(PERSPECTIVE_N_POINT_SOLVING_INCLUDE_DIRS ${PERSPECTIVE_N_POINT_SOLVING_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
	set(PERSPECTIVE_N_POINT_SOLVING_DEPENDENCIES ${PERSPECTIVE_N_POINT_SOLVING_DEPENDENCIES} ${OpenCV_LIBRARIES})
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
