/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3D.hpp
 * @date 23/02/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN implements the Structure From Motion as implementation of the DPFC for Point Cloud Model Localisation.
 *  
 *
 * @{
 */

#ifndef FEATURES_MATCHING_3D_HPP
#define FEATURES_MATCHING_3D_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudModelLocalisation/PointCloudModelLocalisationInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>


namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class FeaturesMatching3D : public PointCloudModelLocalisationInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		FeaturesMatching3D();
		~FeaturesMatching3D();
		void process();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */	
	private:
		dfn_ci::FeaturesExtraction3DInterface* featuresExtractor3d;
		dfn_ci::FeaturesDescription3DInterface* optionalFeaturesDescriptor3d;
		dfn_ci::FeaturesMatching3DInterface* featuresMatcher3d;

		PointCloudWrapper::PointCloudConstPtr sceneCloud;
		PointCloudWrapper::PointCloudConstPtr lastModelCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr modelKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneFeaturesVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr modelFeaturesVector;
		PoseWrapper::Pose3DConstPtr modelPoseInScene;

		void AssignDfnsAlias();

		void ExtractSceneFeatures();
		void ExtractModelFeatures();
		void DescribeSceneFeatures();
		void DescribeModelFeatures();
		bool EstimateModelPose();
    };
}
#endif
/* FeaturesMatching3D.hpp */
/** @} */
