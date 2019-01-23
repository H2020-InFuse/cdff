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
 *  (i) the left images are compared in time to compute the motion of the camera from one position to the next;
 *  (ii) the simultaneous left and right images of the stereo camera are used to compute a point cloud;
 *  (iii) point clouds at different time are merged together taking into account the time displacement of the camera.
 *
 * This DFPC is configured according to the following parameters (beyond those that are needed to configure the DFN components):
 * @param SearchRadius, the output is given by the point of the reconstructed cloud contained within a sphere of center given by the current camera pose and radius given by this parameter;
 * @param PointCloudMapResolution, the voxel resolution of the output point cloud, if the cloud is denser it will be filtered by PCL voxel filter;
 * @param RightToLeftCameraPose, pose of the right camera with respect to the left camera.
 *
 * Notes: no set of DFNs implementation has produced good result for this DFPC during testing;
 * Notes: this class has never been refactored to the standards of the other implementation of the Reconstruction3D DFPC.
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
		void run() override;
		void setup() override;

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

		//Additional DFPC Parameters
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

		//General configuration helper
		DfpcConfigurator configurator;

		//State tracker variables
		Map* map;
		PoseWrapper::Pose3DPtr rightToLeftCameraPose;

		//DFN Interfaces
		CDFF::DFN::ImageFilteringInterface* leftFilter;
		CDFF::DFN::ImageFilteringInterface* rightFilter;		
		CDFF::DFN::FeaturesExtraction2DInterface* featuresExtractor;
		CDFF::DFN::FeaturesDescription2DInterface* optionalFeaturesDescriptor;
		CDFF::DFN::FeaturesMatching2DInterface* featuresMatcher;	
		CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		CDFF::DFN::CamerasTransformEstimationInterface* cameraTransformEstimator;
		CDFF::DFN::PointCloudReconstruction2DTo3DInterface* reconstructor3D;

		//Intermediate output helper variables -- Legacy code --
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

		//Parameters Configuration method
		void ConfigureExtraParameters();

		//DFN instantuation method
		void AssignDfnsAlias();

		//Core macro computation methods that execute a step of the DFPC pipeline
		bool ComputeCameraMovement();
		void ComputePointCloud();
		void UpdateScene();

		//Core micro computation methods that execute a single DFN -- Legacy code -- made obsolete by executors
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

#endif // RECONSTRUCTION3D_RECONSTRUCTIONFROMMOTION_HPP

/** @} */
