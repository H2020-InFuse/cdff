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
