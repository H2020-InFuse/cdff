set(CMAKE_CXX_FLAGS "-g")

include_directories("${CMAKE_SOURCE_DIR}/CentralDPM/src")

add_executable(CentralDPMTest CentralDPMTest.cpp)

target_link_libraries(CentralDPMTest cdff_types centralDPM)