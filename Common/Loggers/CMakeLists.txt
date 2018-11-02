# libcdff_logger

add_library(cdff_logger
    Logger.cpp
    LoggerFactory.cpp
    StandardOutputLogger.cpp)

install(TARGETS cdff_logger
    DESTINATION "${CMAKE_INSTALL_LIBDIR}")
