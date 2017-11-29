project(infuse_dev)
cmake_minimum_required(VERSION 2.6)

include(FindPkgConfig)

#This compiles in testing mode
add_definitions(-DTESTING)

#This policy disable warning for the use of relative libraries path
cmake_policy(SET CMP0015 NEW)

#Reference directories
include_directories(
    DFNs Common Tests Tools Common/Types/C/ASNCommon Common/Types/C Common/Converters External/opencv3/include External/tinyxml2/include
)
link_directories(
	 External/opencv3/lib External/tinyxml2/lib
)


#Main Libraries
add_library(Common Common/Loggers/Logger.cpp Common/Loggers/StandardOutputLogger.cpp Common/Loggers/LoggerFactory.cpp Common/Types/C/ImageType.c Common/Types/C/VisualPointFeatureVector2D.c Common/Converters/ImageTypeToMatConverter.cpp Common/Converters/MatToVisualPointFeatureVector2DConverter.cpp Common/Types/C/asn_SEQUENCE_OF.c Common/Types/C/asn_SET_OF.c Common/Converters/MatToImageTypeConverter.cpp Common/Converters/VisualPointFeatureVector2DToMatConverter.cpp Common/Types/C/OCTET_STRING.c Common/Types/C/per_opentype.c Common/Types/C/asn_codecs_prim.c Common/Types/C/per_support.c Common/Types/C/NativeReal.c Common/Types/C/NativeInteger.c Common/Types/C/NativeEnumerated.c Common/Types/C/xer_decoder.c Common/Types/C/xer_decoder.c Common/Types/C/ber_tlv_tag.c Common/Types/C/ber_decoder.c Common/Types/C/ber_tlv_length.c Common/Types/C/constraints.c Common/Types/C/BIT_STRING.c Common/Types/C/der_encoder.c Common/Types/C/per_encoder.c Common/Types/C/per_decoder.c Common/Types/C/constr_SEQUENCE_OF.c Common/Types/C/constr_SET_OF.c Common/Types/C/constr_TYPE.c Common/Types/C/constr_SEQUENCE.c Common/Types/C/xer_support.c)
set_target_properties(Common PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ./lib)

add_library(FeaturesExtraction2D DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Common/XmlHelper/XmlHelper.cpp)
set_target_properties(FeaturesExtraction2D PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ./lib)
target_link_libraries(
	FeaturesExtraction2D Common tinyxml2
)


#Testing programs
add_library(StubFeaturesExtraction2D Tests/Stubs/DFNs/FeaturesExtraction2D/StubFeaturesExtraction2D.cpp DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp)
set_target_properties(StubFeaturesExtraction2D PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ./test_lib)
target_link_libraries(
	StubFeaturesExtraction2D Common
)

add_executable(StubFeaturesExtraction2DTest Tests/UnitTests/DFNs/FeaturesExtraction2D/StubFeaturesExtraction2D.cpp)
set_target_properties(StubFeaturesExtraction2DTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	StubFeaturesExtraction2DTest Common StubFeaturesExtraction2D opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(HarrisDetector2DTest Tests/UnitTests/DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Tests/Mocks/Common/Converters/ImageTypeToMatConverter.cpp Tests/Mocks/Mock.cpp Tests/Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.cpp DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Common/XmlHelper/XmlHelper.cpp)
set_target_properties(HarrisDetector2DTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector2DTest Common opencv_core opencv_imgproc opencv_imgcodecs tinyxml2
)

add_executable(HarrisDetector2DGuiTest Tests/GuiTests/DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Tests/Mocks/Common/Converters/ImageTypeToMatConverter.cpp Tests/Mocks/Mock.cpp Tests/Mocks/Common/Converters/MatToVisualPointFeatureVector2DConverter.cpp DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.cpp DFNs/FeaturesExtraction2D/HarrisDetector2D.cpp Common/XmlHelper/XmlHelper.cpp Tests/GuiTests/ParametersInterface.cpp Tests/GuiTests/ButtonsInterface.cpp Tests/GuiTests/MainInterface.cpp Tests/GuiTests/DFNs/DFNTestInterface.cpp)
set_target_properties(HarrisDetector2DGuiTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	HarrisDetector2DGuiTest Common opencv_core opencv_imgproc opencv_imgcodecs tinyxml2 opencv_highgui
)

add_executable(MatImageTypeConvertersTest Tests/UnitTests/Common/Converters/MatImageTypeConvertersTest.cpp )
set_target_properties(MatImageTypeConvertersTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	MatImageTypeConvertersTest Common opencv_core opencv_imgproc opencv_imgcodecs
)

add_executable(MatVisualPointFeatureVector2DConvertersTest Tests/UnitTests/Common/Converters/MatVisualPointFeatureVector2DConvertersTest.cpp )
set_target_properties(MatVisualPointFeatureVector2DConvertersTest PROPERTIES RUNTIME_OUTPUT_DIRECTORY ./tests)
target_link_libraries(
	MatVisualPointFeatureVector2DConvertersTest Common opencv_core opencv_imgproc opencv_imgcodecs
)

#Moving configurations
FILE (COPY Tests/ConfigurationFiles/ DESTINATION ./tests/ConfigurationFiles/)
FILE (COPY Tests/Data/ DESTINATION ./tests/Data/)




#Installation scripts Examples
#install(
#    TARGETS Common FeaturesExtraction2D
#    DESTINATION lib
#)
#
#install(
#    FILES DFNs/FeaturesExtraction2D/HarrisDetector2D.hpp DFNs/FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp DFNs/DFNCommonInterface.hpp
#    DESTINATION include/DFNs
#)

