add_library(
    perspective_n_point_solving
    PerspectiveNPointSolvingInterface.cpp
)

add_library(
    iterative_pnp_solver
    IterativePnpSolver.cpp
)
target_link_libraries(
    iterative_pnp_solver
    perspective_n_point_solving cdff_types cdff_helpers yaml-cpp opencv_features2d opencv_calib3d opencv_core cdff_converters
)
