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
#include <FeaturesExtraction3D/FeaturesExtraction3DExecutor.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DExecutor.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DExecutor.hpp>
#include <DfpcConfigurator.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Types/CPP/PointCloud.hpp>

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
		void modelInput(const asn1SccPointcloud& data);

	private:
		DfpcConfigurator configurator;

		CDFF::DFN::FeaturesExtraction3DExecutor* featuresExtractor3d;
		CDFF::DFN::FeaturesDescription3DExecutor* optionalFeaturesDescriptor3d;
		CDFF::DFN::FeaturesMatching3DExecutor* featuresMatcher3d;

		bool modelFeaturesAvailable;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr modelFeatureVector;

		void InstantiateDFNExecutors();

		void ExtractSceneFeatures();
		void ExtractModelFeatures();
		void DescribeSceneFeatures();
		void DescribeModelFeatures();
		bool EstimateModelPose();

		/*
		* Inline Methods
		*
		*/

		template <typename Type>
		void DeleteIfNotNull(Type* &pointer)
			{
			if (pointer != NULL) 
				{
				delete(pointer);
				pointer = NULL;
				}
			}
	};
}
}
}

#endif // POINTCLOUDMODELLOCALISATION_FEATURESMATCHING3D_HPP

/** @} */
