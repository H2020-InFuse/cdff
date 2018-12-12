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
 * @addtogroup DFPCs
 * 
 *  This DFN chain implements the Reconstruction From Motion as implementation of the DPFC for Reconsrtruction3D.
 *  This chain operates as follows: 
 *  the left images are compared in time to compute the motion of the camera from one position to the next
 *  the simultaneous left and right images of the stereo camera are used to compute a point cloud
 *  point clouds at different time are merged together taking into account the time displacement of the camera.
 * 
 * @{
 */

#ifndef RECONSTRUCTION3D_RECONSTRUCTIONFROMMOTION_HPP
#define RECONSTRUCTION3D_RECONSTRUCTIONFROMMOTION_HPP

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
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/Matrix.hpp>


namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

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
		explicit ReconstructionFromMotion(Map* map = NULL);
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
			float searchRadius;
			float pointCloudMapResolution;
			CameraPose rightToLeftCameraPose;
			};

		Helpers::ParametersListHelper parametersHelper;
		ReconstructionFromMotionOptionsSet parameters;
		static const ReconstructionFromMotionOptionsSet DEFAULT_PARAMETERS;

		DfpcConfigurator configurator;
		Map* map;
		PoseWrapper::Pose3DPtr rightToLeftCameraPose;

		CDFF::DFN::ImageFilteringInterface* leftFilter;
		CDFF::DFN::ImageFilteringInterface* rightFilter;		
		CDFF::DFN::FeaturesExtraction2DInterface* featuresExtractor;
		CDFF::DFN::FeaturesDescription2DInterface* optionalFeaturesDescriptor;
		CDFF::DFN::FeaturesMatching2DInterface* featuresMatcher;	
		CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		CDFF::DFN::CamerasTransformEstimationInterface* cameraTransformEstimator;
		CDFF::DFN::PointCloudReconstruction2DTo3DInterface* reconstructor3D;

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
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr leftRightCorrespondenceMap;
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
}
}

#endif // RECONSTRUCTION3D_RECONSTRUCTIONFROMMOTION_HPP

/** @} */
