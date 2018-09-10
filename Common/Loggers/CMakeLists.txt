add_library(
    cdff_logger
    Logger.cpp
    LoggerFactory.cpp
    StandardOutputLogger.cpp
)
set_target_properties(cdff_logger PROPERTIES COMPILE_FLAGS -fPIC)
install(
    TARGETS cdff_logger
    DESTINATION lib
)
