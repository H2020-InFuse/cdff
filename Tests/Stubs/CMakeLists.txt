add_library(
    StubFeaturesExtraction2D
    DFNs/FeaturesExtraction2D/StubFeaturesExtraction2D.cpp
)
set_target_properties(StubFeaturesExtraction2D PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ./test_lib)
target_link_libraries(
	StubFeaturesExtraction2D feature_extraction_2d
)
