set(TEST_CONFIGURATIONS ${CMAKE_CURRENT_SOURCE_DIR}/ConfigurationFiles/)
set(TEST_DATA ${CMAKE_CURRENT_SOURCE_DIR}/Data/)
FILE (COPY ConfigurationFiles/ DESTINATION ./tests/ConfigurationFiles/)
FILE (COPY Data/ DESTINATION ./tests/Data/)

add_subdirectory(Mocks) ## used by the graphical tests of the DFPCs

add_subdirectory(DataGenerators)

add_subdirectory(UnitTests)
add_subdirectory(GuiTests)
add_subdirectory(PerformanceTests)
add_subdirectory(KeyPerformanceMeasuresTests)
