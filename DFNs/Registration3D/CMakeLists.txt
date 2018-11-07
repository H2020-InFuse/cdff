add_library(
    registration_3d
    Registration3DInterface.cpp
)

add_library(
    registration_icp_3d
    Icp3D.cpp
)
target_link_libraries(
    registration_icp_3d
    registration_3d cdff_types yaml-cpp cdff_helpers cdff_converters
)

add_library(
    registration_icp_cc
    IcpCC.cpp
)
target_link_libraries(
    registration_icp_cc
    registration_3d cdff_types yaml-cpp cdff_helpers cdff_converters cc_core_lib
)

add_library(
    registration_icp_matcher
    IcpMatcher.cpp
)
target_link_libraries(
    registration_icp_matcher
    registration_3d cdff_types yaml-cpp cdff_helpers cdff_converters cc_core_lib
)

