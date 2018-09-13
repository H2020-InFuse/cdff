add_library(
    perspective_n_point_solving
    PerspectiveNPointSolvingInterface.cpp
    PerspectiveNPointSolvingExecutor.cpp
)
set_target_properties(perspective_n_point_solving PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})

add_library(
    iterative_pnp_solver
    IterativePnpSolver.cpp
)
set_target_properties(iterative_pnp_solver PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
target_link_libraries(
    iterative_pnp_solver
    perspective_n_point_solving cdff_types cdff_helpers yaml-cpp opencv_features2d opencv_calib3d opencv_core converters_opencv
)
