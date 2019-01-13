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
		void run() override;
		void setup() override;

		//This overrides the modelInput variable, as we would like to record when a new model is provided.
		void modelInput(const asn1SccPointcloud& data) override;

	private:
		//General configuration helper
		DfpcConfigurator configurator;

		//Pointers to DFN instances
		CDFF::DFN::FeaturesExtraction3DInterface* featuresExtractor3d;
		CDFF::DFN::FeaturesDescription3DInterface* optionalFeaturesDescriptor3d;
		CDFF::DFN::FeaturesMatching3DInterface* featuresMatcher3d;

		//Model variables
		bool modelFeaturesAvailable;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr modelFeatureVector;

		//This methods instantiates the DFNs
		void InstantiateDFNs();

		//Core computation method that execute a step of the DFPC pipeline
		void ExtractSceneFeatures();
		void ExtractModelFeatures();
		void DescribeSceneFeatures();
		void DescribeModelFeatures();
		bool EstimateModelPose();

		/*
		* Inline helper Method
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
