/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionFromStereo.hpp
 * @date 12/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN chain implements the Reconstruction From Stereo as implementation of the DPFC for Reconsrtruction3D.
 *  This chain operates as follows: 
 *  the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map
 *  camera movement is estimated by matching features in the left images taken at two distinct time instants,
 *  point clouds at different time instants are merged together taking into account the movement of the camera.
 * 
 * @{
 */

#ifndef RECONSTRUCTION_FROM_STEREO_HPP
#define RECONSTRUCTION_FROM_STEREO_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>

#include "Map.hpp"
#include "ObservedScene.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>


namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class ReconstructionFromStereo : public Reconstruction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		ReconstructionFromStereo(Map* map = NULL);
		~ReconstructionFromStereo();
		void run();
		void setup();

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
		DfpcConfigurator configurator;
		Map* map;

		struct ReconstructionFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			};

		Helpers::ParametersListHelper parametersHelper;
		ReconstructionFromStereoOptionsSet parameters;
		static const ReconstructionFromStereoOptionsSet DEFAULT_PARAMETERS;

		dfn_ci::ImageFilteringInterface* leftFilter;
		dfn_ci::ImageFilteringInterface* rightFilter;
		dfn_ci::FeaturesExtraction2DInterface* featuresExtractor;
		dfn_ci::FeaturesDescription2DInterface* optionalFeaturesDescriptor;
		dfn_ci::FeaturesMatching2DInterface* featuresMatcher;	
		dfn_ci::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		dfn_ci::CamerasTransformEstimationInterface* cameraTransformEstimator;
		dfn_ci::StereoReconstructionInterface* reconstructor3D;

		FrameWrapper::FrameConstPtr pastLeftImage;
		FrameWrapper::FrameConstPtr currentLeftImage;
		FrameWrapper::FrameConstPtr currentRightImage;
		FrameWrapper::FrameConstPtr filteredPastLeftImage;
		FrameWrapper::FrameConstPtr filteredCurrentLeftImage;
		FrameWrapper::FrameConstPtr filteredCurrentRightImage;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr pastLeftKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentLeftKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentRightKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr pastLeftFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentLeftFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentRightFeaturesVector;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr pastToCurrentCorrespondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftToRightCorrespondenceMap;
		MatrixWrapper::Matrix3dConstPtr fundamentalMatrix;
		PoseWrapper::Pose3DConstPtr pastToCurrentCameraTransform;
		PointCloudWrapper::PointCloudConstPtr pointCloud;

		void ConfigureExtraParameters();
		void AssignDfnsAlias();

		bool ComputeCameraMovement();
		void ComputePointCloud();
		void UpdateScene();

		void FilterCurrentLeftImage();
		void FilterPastLeftImage();
		void FilterCurrentRightImage();
		void ExtractCurrentLeftFeatures();
		void ExtractPastLeftFeatures();
		void ExtractCurrentRightFeatures();
		void DescribeCurrentLeftFeatures();
		void DescribePastLeftFeatures();
		void DescribeCurrentRightFeatures();
		void MatchCurrentAndPastFeatures();
		void MatchLeftAndRightFeatures();
		bool ComputeFundamentalMatrix();
		bool ComputePastToCurrentTransform();
		void ComputeStereoPointCloud();
    };
}
#endif
/* ReconstructionFromStereo.hpp */
/** @} */
