file(GLOB_RECURSE source_list "C/*.c" "C/*.h")

add_library(
    cdff_types
    $source_list
)
install(
    TARGETS cdff_types
    DESTINATION lib
)
