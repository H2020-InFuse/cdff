/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ReconstructionFromMotion.hpp
 * @date 08/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNs
 * 
 *  This DFN chain implements the Reconstruction From Motion as implementation of the DPFC for Reconsrtruction3D.
 *  This chain operates as follows: 
 *  the left images are compared in time to compute the motion of the camera from one position to the next
 *  the simultaneous left and right images of the stereo camera are used to compute a point cloud
 *  point clouds at different time are merged together taking into account the time displacement of the camera.
 * 
 * @{
 */

#ifndef RECONSTRUCTION_FROM_MOTION_HPP
#define RECONSTRUCTION_FROM_MOTION_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
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
    class ReconstructionFromMotion : public Reconstruction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		ReconstructionFromMotion(Map* map = NULL);
		~ReconstructionFromMotion();
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
		struct CameraPose
			{
			float positionX;
			float positionY;
			float positionZ;
			float orientationX;
			float orientationY;
			float orientationZ;
			float orientationW;			
			};

		struct ReconstructionFromMotionOptionsSet
			{
			float pointCloudMapResolution;
			CameraPose rightToLeftCameraPose;
			};

		Helpers::ParametersListHelper parametersHelper;
		ReconstructionFromMotionOptionsSet parameters;
		static const ReconstructionFromMotionOptionsSet DEFAULT_PARAMETERS;

		DfpcConfigurator configurator;
		Map* map;
		float searchRadius;
		PoseWrapper::Pose3DPtr rightToLeftCameraPose;

		dfn_ci::ImageFilteringInterface* leftFilter;
		dfn_ci::ImageFilteringInterface* rightFilter;		
		dfn_ci::FeaturesExtraction2DInterface* featuresExtractor;
		dfn_ci::FeaturesDescription2DInterface* optionalFeaturesDescriptor;
		dfn_ci::FeaturesMatching2DInterface* featuresMatcher;	
		dfn_ci::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		dfn_ci::CamerasTransformEstimationInterface* cameraTransformEstimator;
		dfn_ci::PointCloudReconstruction2DTo3DInterface* reconstructor3D;

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
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftRightCorrespondenceMap;
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
/* ReconstructionFromMotion.hpp */
/** @} */
