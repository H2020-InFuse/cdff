add_library(
    cdff_logger
    Logger.cpp
    LoggerFactory.cpp
    StandardOutputLogger.cpp
)
install(
    TARGETS cdff_logger
    DESTINATION lib
)
