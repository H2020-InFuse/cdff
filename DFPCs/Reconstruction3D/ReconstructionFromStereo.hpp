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
#include <Frame.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Matrix.hpp>


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

		CDFF::DFN::ImageFilteringInterface* leftFilter;
		CDFF::DFN::ImageFilteringInterface* rightFilter;
		CDFF::DFN::FeaturesExtraction2DInterface* featuresExtractor;
		CDFF::DFN::FeaturesDescription2DInterface* optionalFeaturesDescriptor;
		CDFF::DFN::FeaturesMatching2DInterface* featuresMatcher;	
		CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		CDFF::DFN::CamerasTransformEstimationInterface* cameraTransformEstimator;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3D;

		FrameWrapper::FrameConstPtr pastLeftImage;
		FrameWrapper::FramePtr currentLeftImage;
		FrameWrapper::FramePtr currentRightImage;
		FrameWrapper::FramePtr filteredPastLeftImage;
		FrameWrapper::FramePtr filteredCurrentLeftImage;
		FrameWrapper::FramePtr filteredCurrentRightImage;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr pastLeftKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr currentLeftKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr currentRightKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr pastLeftFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr currentLeftFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr currentRightFeaturesVector;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr pastToCurrentCorrespondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr leftToRightCorrespondenceMap;
		MatrixWrapper::Matrix3dPtr fundamentalMatrix;
		PoseWrapper::Pose3DPtr pastToCurrentCameraTransform;
		PointCloudWrapper::PointCloudPtr pointCloud;

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
