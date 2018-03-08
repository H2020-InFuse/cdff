add_library(
    StubFeaturesExtraction2D
    DFNs/FeaturesExtraction2D/StubFeaturesExtraction2D.cpp
)
target_link_libraries(
	StubFeaturesExtraction2D feature_extraction_2d
)

add_library(
    stub_observed_scene
    DFPCs/PointCloudModelLocalisation/ObservedScene.cpp
)

add_library(
    reconstruction_3d_stub_observed_scene
    DFPCs/Reconstruction3D/ObservedScene.cpp
)
