file(GLOB_RECURSE source_list
 "*/*.c"
 "*/*.h"
)

add_library(
    cdff_types
    ${source_list}
)
install(
    TARGETS cdff_types
    DESTINATION lib
)
