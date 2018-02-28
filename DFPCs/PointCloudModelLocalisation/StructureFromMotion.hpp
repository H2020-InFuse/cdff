/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file StructureFromMotion.hpp
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

#ifndef STRUCTURE_FROM_MOTION_HPP
#define STRUCTURE_FROM_MOTION_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloudModelLocalisation/PointCloudModelLocalisationInterface.hpp>

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>

#include "Map.hpp"


namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class StructureFromMotion : public PointCloudModelLocalisationInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		StructureFromMotion(Map* map);
		~StructureFromMotion();
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
		Map* map;
		float searchRadius;

		dfn_ci::ImageFilteringInterface* filter;
		dfn_ci::FeaturesExtraction2DInterface* featuresExtractor;
		dfn_ci::FeaturesDescription2DInterface* optionalFeaturesDescriptor;
		dfn_ci::FeaturesMatching2DInterface* featuresMatcher;	
		dfn_ci::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		dfn_ci::CamerasTransformEstimationInterface* cameraTransformEstimator;
		dfn_ci::PointCloudReconstruction2DTo3DInterface* reconstructor3D;
		dfn_ci::FeaturesExtraction3DInterface* featuresExtractor3d;
		dfn_ci::FeaturesDescription3DInterface* optionalFeaturesDescriptor3d;
		dfn_ci::FeaturesMatching3DInterface* featuresMatcher3d;

		FrameWrapper::FrameConstPtr pastImage;
		FrameWrapper::FrameConstPtr currentImage;
		FrameWrapper::FrameConstPtr filteredPastImage;
		FrameWrapper::FrameConstPtr filteredCurrentImage;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr pastKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr pastFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentFeaturesVector;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap;
		MatrixWrapper::Matrix3dConstPtr fundamentalMatrix;
		PoseWrapper::Pose3DConstPtr pastToCurrentCameraTransform;
		PointCloudWrapper::PointCloudConstPtr pointCloud;
		PointCloudWrapper::PointCloudConstPtr sceneCloud;
		PointCloudWrapper::PointCloudConstPtr lastModelCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr modelKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneFeaturesVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr modelFeaturesVector;
		PoseWrapper::Pose3DConstPtr modelPoseInScene;

		void AssignDfnsAlias();

		bool ComputeCloudInSight();
		void UpdateScene();
		bool LookForModel();

		void FilterCurrentImage();
		void FilterPastImage();
		void ExtractCurrentFeatures();
		void ExtractPastFeatures();
		void DescribeCurrentFeatures();
		void DescribePastFeatures();
		void MatchCurrentAndPastFeatures();
		bool ComputeFundamentalMatrix();
		bool ComputePastToCurrentTransform();
		void ComputePointCloud();
		void ExtractSceneFeatures();
		void ExtractModelFeatures();
		void DescribeSceneFeatures();
		void DescribeModelFeatures();
		bool EstimateModelPose();
    };
}
#endif
/* StructureFromMotion.hpp */
/** @} */
