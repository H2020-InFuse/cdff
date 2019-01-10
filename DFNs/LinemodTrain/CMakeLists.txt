set(LINEMOD_TRAIN_SOURCES "LinemodTrainInterface.cpp")
set(LINEMOD_TRAIN_INCLUDE_DIRS "")
set(LINEMOD_TRAIN_DEPENDENCIES "cdff_types" "cdff_helpers" "cdff_converters")

if(OpenCV_FOUND AND VTK_FOUND)
    set(LINEMOD_TRAIN_SOURCES ${LINEMOD_TRAIN_SOURCES} "LinemodTrain.cpp"
       "linemod-wrapper/LinemodTemplateGenerator.cpp" "linemod-wrapper/LinemodTemplateGeneratorIteratorSinCos.cpp"
       "linemod-wrapper/LinemodTrainImpl.cpp")
    #OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
    set(LINEMOD_TRAIN_DEPENDENCIES ${LINEMOD_TRAIN_DEPENDENCIES} ${VTK_LIBRARIES} ${OpenCV_LIBS})
endif()

add_library(
    cdff_dfn_linemod_train
    ${LINEMOD_TRAIN_SOURCES}
)
target_include_directories(
    cdff_dfn_linemod_train
    SYSTEM PRIVATE ${LINEMOD_TRAIN_INCLUDE_DIRS}
)
target_link_libraries(
    cdff_dfn_linemod_train
    ${LINEMOD_TRAIN_DEPENDENCIES}
)
