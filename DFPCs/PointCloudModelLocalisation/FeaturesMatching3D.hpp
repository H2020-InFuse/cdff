/**
 * @author Alessandro Bianco
 */

/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef POINTCLOUDMODELLOCALISATION_FEATURESMATCHING3D_HPP
#define POINTCLOUDMODELLOCALISATION_FEATURESMATCHING3D_HPP

#include <PointCloudModelLocalisation/PointCloudModelLocalisationInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <DfpcConfigurator.hpp>
#include <VisualPointFeatureVector3D.hpp>
#include <PointCloud.hpp>

namespace CDFF
{
namespace DFPC
{
namespace PointCloudModelLocalisation
{
	/**
	 * This DFN Chains detects a point cloud model within a point cloud scene by execution of the following DFNs:
	 *
	 * (i) 3d keypoints extraction on the model and the scene;
	 * (ii) computation of the features descriptors for the keypoints extracted from the model and the scene;
	 * (iii) matching of the features and computation of the model pose within the coordinate system of the scene.
	 */
	class FeaturesMatching3D : public PointCloudModelLocalisationInterface
	{
	public:
		FeaturesMatching3D();
		~FeaturesMatching3D();
		void run();
		void setup();

	private:
		DfpcConfigurator configurator;

		CDFF::DFN::FeaturesExtraction3DInterface* featuresExtractor3d;
		CDFF::DFN::FeaturesDescription3DInterface* optionalFeaturesDescriptor3d;
		CDFF::DFN::FeaturesMatching3DInterface* featuresMatcher3d;

		PointCloudWrapper::PointCloudPtr sceneCloud;
		PointCloudWrapper::PointCloudPtr lastModelCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr sceneKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr modelKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr sceneFeaturesVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr modelFeaturesVector;

		void AssignDfnsAlias();

		void ExtractSceneFeatures();
		void ExtractModelFeatures();
		void DescribeSceneFeatures();
		void DescribeModelFeatures();
		bool EstimateModelPose();
	};
}
}
}

#endif // POINTCLOUDMODELLOCALISATION_FEATURESMATCHING3D_HPP

/** @} */
