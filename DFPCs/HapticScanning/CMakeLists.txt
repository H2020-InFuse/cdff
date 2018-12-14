set(HAPTIC_SCANNING_SOURCES "HapticScanningInterface.cpp" "HapticScanning.cpp")
set(HAPTIC_SCANNING_INCLUDE_DIRS "")
set(HAPTIC_SCANNING_DEPENDENCIES "cdff_dfpc_configurator")

add_library(
    cdff_dfpc_haptic_scanning
    ${HAPTIC_SCANNING_SOURCES}
    )
target_include_directories(
    cdff_dfpc_haptic_scanning 
    SYSTEM PRIVATE ${HAPTIC_SCANNING_INCLUDE_DIRS}
    )
target_link_libraries(
    cdff_dfpc_haptic_scanning
    ${HAPTIC_SCANNING_DEPENDENCIES}
    )
