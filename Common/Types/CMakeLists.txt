# ASN.1 data types

option(COMPILE_ASN1 "Transcompile the ASN.1 data types to C code" ON)

if(NOT IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/C")

    # If the C/ directory does not exist, we must compile the ASN.1 data types
    # to C code, or obtain precompiled types

    message(STATUS "Transcompiled ASN.1 data types not found")

    if(COMPILE_ASN1)

        # Transcompilation requires Mono's core components; they are in the
        # h2020infuse/cdff:latest Docker image for instance, so we can run
        # a contained transcompilation of the ASN.1 data types
        set(action "Transcompiling the")
        set(script "${CMAKE_SOURCE_DIR}/Tools/ASN.1/GeneratorScript.sh")

    else()

        # Otherwise we can download transcompiled data types from the CDFF's
        # continuous integration server, where Mono's core components are
        # installed (only works for branches which have been pushed to this
        # server)
        set(action "Downloading transcompiled")
        set(script "${CMAKE_SOURCE_DIR}/Tools/ASN.1/FetcherScript.sh")

    endif()

    message(STATUS "${action} ASN.1 data types with: ${script}")

    execute_process(COMMAND "${script}"
        RESULT_VARIABLE error OUTPUT_VARIABLE stdout ERROR_VARIABLE stderr)

    message(STATUS "${action} ASN.1 data types: return code: ${error}")
    message(STATUS "${action} ASN.1 data types: standard output:\n${stdout}")
    message(STATUS "${action} ASN.1 data types: standard error:\n${stderr}")

    if(error)
        message(FATAL_ERROR "${action} ASN.1 data types failed")
    endif()

endif()

if(IS_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/C")

    message(STATUS "Found transcompiled ASN.1 data types: ${CMAKE_CURRENT_SOURCE_DIR}/C")

endif()

# libcdff_types

file(GLOB datatypes
    RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}"
    "C/*.c")

message(STATUS "Transcompiled ASN.1 datatypes: ${datatypes}")

set(wrappers
    CPP/Array3D.cpp
    CPP/BaseTypes.cpp
    CPP/CorrespondenceMap2D.cpp
    CPP/CorrespondenceMap3D.cpp
    CPP/CorrespondenceMaps2DSequence.cpp
    CPP/CorrespondenceMaps3DSequence.cpp
    CPP/Frame.cpp
    CPP/FramesSequence.cpp
    CPP/Matrix.cpp
    CPP/PointCloud.cpp
    CPP/Pose.cpp
    CPP/PosesSequence.cpp
    CPP/VisualPointFeatureVector2D.cpp
    CPP/VisualPointFeatureVector3D.cpp)

message(STATUS "C++ wrappers for the transcompiled ASN.1 datatypes: ${wrappers}")

add_library(cdff_types
    ${datatypes}
    ${wrappers})

target_link_libraries(cdff_types
    PUBLIC cdff_logger
)

install(TARGETS cdff_types
    DESTINATION "${CMAKE_INSTALL_LIBDIR}")
