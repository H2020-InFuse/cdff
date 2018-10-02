
add_library(cdff_validators Frame.cpp)
target_link_libraries(cdff_validators cdff_types)
target_include_directories(cdff_validators PUBLIC ..)