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
 *  camera movement is estimated by matching features in the left images taken at two distinct times,
 *  point clouds at different time are merged together taking into account the time displacement of the camera.
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
#include <Helpers/ParametersListHelper.hpp>


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
		ReconstructionFromStereo(Map* map);
		~ReconstructionFromStereo();
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
		/*struct CameraPose
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
			CameraPose rightToLeftCameraPose;
			};

		Helpers::ParametersListHelper parametersHelper;
		ReconstructionFromMotionOptionsSet parameters;
		static const ReconstructionFromMotionOptionsSet DEFAULT_PARAMETERS;

		void ConfigureChain();*/

		Map* map;
		float searchRadius;
		//PoseWrapper::Pose3DPtr rightToLeftCameraPose;

		dfn_ci::ImageFilteringInterface* filter;
		dfn_ci::FeaturesExtraction2DInterface* featuresExtractor;
		dfn_ci::FeaturesDescription2DInterface* optionalFeaturesDescriptor;
		dfn_ci::FeaturesMatching2DInterface* featuresMatcher;	
		dfn_ci::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		dfn_ci::CamerasTransformEstimationInterface* cameraTransformEstimator;
		dfn_ci::StereoReconstructionInterface* reconstructor3D;

		FrameWrapper::FrameConstPtr pastImage;
		FrameWrapper::FrameConstPtr currentImage;
		FrameWrapper::FrameConstPtr currentRightImage;
		FrameWrapper::FrameConstPtr filteredPastImage;
		FrameWrapper::FrameConstPtr filteredCurrentImage;
		FrameWrapper::FrameConstPtr filteredCurrentRightImage;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr pastKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentRightKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr pastFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr currentRightFeaturesVector;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr correspondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftRightCorrespondenceMap;
		MatrixWrapper::Matrix3dConstPtr fundamentalMatrix;
		PoseWrapper::Pose3DConstPtr pastToCurrentCameraTransform;
		PointCloudWrapper::PointCloudConstPtr pointCloud;
		PointCloudWrapper::PointCloudConstPtr sceneCloud;

		void AssignDfnsAlias();

		bool ComputeCameraMovement();
		void UpdateScene();

		void FilterCurrentImage();
		void FilterPastImage();
		void FilterCurrentRightImage();
		void ExtractCurrentFeatures();
		void ExtractPastFeatures();
		void ExtractCurrentRightFeatures();
		void DescribeCurrentFeatures();
		void DescribePastFeatures();
		void DescribeCurrentRightFeatures();
		void MatchCurrentAndPastFeatures();
		void MatchLeftAndRightFeatures();
		bool ComputeFundamentalMatrix();
		bool ComputePastToCurrentTransform();
		void ComputePointCloud();
    };
}
#endif
/* ReconstructionFromStereo.hpp */
/** @} */
