set(CMAKE_CXX_FLAGS "-g")

add_executable(initialTest initialTest.cpp)

target_link_libraries(initialTest ${ENVIRE_CORE_LIBRARIES} cdff_types class_loader boost_system)

