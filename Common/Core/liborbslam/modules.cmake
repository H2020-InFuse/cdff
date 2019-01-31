# Here you can specify the type of library to generate: "STATIC" or "SHARED"
INCLUDE(CMakeDependentOption)
#set(MOD1_SHARED_LIBS "STATIC" CACHE STRING "shared or static link")
#set_property(CACHE MOD1_SHARED_LIBS PROPERTY STRINGS SHARED STATIC)

set(LIBORBSLAM_SHARED_LIBS "STATIC" CACHE STRING "shared or static link")
