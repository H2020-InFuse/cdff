add_library(
    haptic_scanning
    HapticScanningInterface.cpp
)

target_link_libraries(
    haptic_scanning
)

add_library(
    dfpc_haptic_scanning
    HapticScanning.cpp
)
target_link_libraries(
    dfpc_haptic_scanning
    haptic_scanning cdff_types yaml-cpp cdff_helpers cdff_opencv_visualizer cdff_pcl_visualizer dfpc_configurator
)
