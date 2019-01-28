if(WITH_OPENCV_CONTRIB AND VTK_FOUND AND Boost_FOUND)
    set(LINEMOD_TRAINING_SOURCES "LinemodTraining.cpp"
       "linemod-wrapper/LinemodTemplateGenerator.cpp" "linemod-wrapper/LinemodTemplateGeneratorIteratorSinCos.cpp"
       "linemod-wrapper/LinemodTrainingImpl.cpp")
    #OpenCV uses imported targets, no need for the INCLUDE_DIRS directory in target_include_directories
    set(LINEMOD_TRAINING_DEPENDENCIES ${VTK_LIBRARIES} ${Boost_LIBRARIES} opencv_rgbd opencv_core opencv_calib3d opencv_imgproc)

    add_executable(LinemodTraining ${LINEMOD_TRAINING_SOURCES})
    target_link_libraries(LinemodTraining ${LINEMOD_TRAINING_DEPENDENCIES})
endif()
