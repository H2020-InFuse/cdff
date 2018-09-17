add_library(
    cdff_helpers
    ParameterHelperInterface.cpp
    ParametersListHelper.cpp
)
target_link_libraries(
    cdff_helpers
    yaml-cpp
)
install(
    TARGETS cdff_helpers
    DESTINATION lib
)
set_target_properties(cdff_helpers PROPERTIES COMPILE_FLAGS ${STATIC_LIB_COMPILE_FLAGS})
