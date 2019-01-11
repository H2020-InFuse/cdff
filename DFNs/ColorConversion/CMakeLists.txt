set(COLOR_CONVERSION_SOURCES "ColorConversionInterface.cpp")
set(COLOR_CONVERSION_INCLUDE_DIRS "")
set(COLOR_CONVERSION_DEPENDENCIES "cdff_types" "yaml-cpp" "cdff_helpers")

if(OpenCV_FOUND)
	set(COLOR_CONVERSION_SOURCES ${COLOR_CONVERSION_SOURCES} "ColorConversion.cpp")
	#OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
        set(COLOR_CONVERSION_DEPENDENCIES ${COLOR_CONVERSION_DEPENDENCIES} opencv_core opencv_imgproc)
endif()

add_library(
    cdff_dfn_color_conversion
    ${COLOR_CONVERSION_SOURCES}
)
target_include_directories(
    cdff_dfn_color_conversion
    SYSTEM PRIVATE ${COLOR_CONVERSION_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_color_conversion
    ${COLOR_CONVERSION_DEPENDENCIES}
)
