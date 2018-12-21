set(FUNDAMENTAL_MATRIX_COMPUTATION_SOURCES "FundamentalMatrixComputationInterface.cpp")
set(FUNDAMENTAL_MATRIX_COMPUTATION_INCLUDE_DIRS "")
set(FUNDAMENTAL_MATRIX_COMPUTATION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND)
	set(FUNDAMENTAL_MATRIX_COMPUTATION_SOURCES ${FUNDAMENTAL_MATRIX_COMPUTATION_SOURCES} "FundamentalMatrixRansac.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
	set(FUNDAMENTAL_MATRIX_COMPUTATION_DEPENDENCIES ${FUNDAMENTAL_MATRIX_COMPUTATION_DEPENDENCIES} ${OpenCV_LIBS})
endif()

add_library(
    cdff_dfn_fundamental_matrix_computation
    ${FUNDAMENTAL_MATRIX_COMPUTATION_SOURCES}
)
target_include_directories(
    cdff_dfn_fundamental_matrix_computation
    SYSTEM PRIVATE ${FUNDAMENTAL_MATRIX_COMPUTATION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_fundamental_matrix_computation
    ${FUNDAMENTAL_MATRIX_COMPUTATION_DEPENDENCIES}
)
