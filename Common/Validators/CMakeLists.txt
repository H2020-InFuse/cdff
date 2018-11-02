# libcdff_validators

add_library(cdff_validators
    Frame.cpp)

target_link_libraries(cdff_validators
    PUBLIC cdff_types)

install(TARGETS cdff_validators
    DESTINATION "${CMAKE_INSTALL_LIBDIR}")
