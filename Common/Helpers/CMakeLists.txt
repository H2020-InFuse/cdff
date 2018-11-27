# libcdff_helpers

add_library(cdff_helpers
    ParameterHelperInterface.cpp
    ParametersListHelper.cpp)

target_link_libraries(cdff_helpers
    PUBLIC yaml-cpp)

install(TARGETS cdff_helpers
    DESTINATION "${CMAKE_INSTALL_LIBDIR}")
