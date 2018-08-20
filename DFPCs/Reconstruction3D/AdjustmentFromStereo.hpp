/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file AdjustmentFromStereo.hpp
 * @date 15/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFPCs
 * 
 *  This DFN chain implements the Adjustment From Stereo as implementation of the DPFC for Reconstruction3D.
 *  This chain operates as follows: 
 *  the input is a stream of pairs of stereo images;
 *  the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map
 *  2d features and their descriptors are extracted from the left and right images; 
 *  the left and right features are matched against each other and against the previous N left and right features;
 *  the point cloud and the matches are stored in a database for future processing;
 *  The correspondences among the latest N image pairs are used for the computation of the camera pose by means of bundle adjustment.    
 * 
 * @{
 */

#ifndef RECONSTRUCTION3D_ADJUSTMENTFROMSTEREO_HPP
#define RECONSTRUCTION3D_ADJUSTMENTFROMSTEREO_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <BundleAdjustment/BundleAdjustmentInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>

#include <VisualPointFeatureVector2D.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Frame.hpp>

#include "PointCloudMap.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>
#include <Frame.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Matrix.hpp>

#ifdef TESTING
#include <fstream>
#endif


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
    class AdjustmentFromStereo : public Reconstruction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		AdjustmentFromStereo();
		~AdjustmentFromStereo();
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
		PointCloudMap pointCloudMap;
		int currentInputNumber;
		int oldestCameraIndex;
		bool firstTimeBundle;

		struct AdjustmentFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			int numberOfAdjustedStereoPairs;
			bool useBundleInitialEstimation;
			float baseline;
			};

		Helpers::ParametersListHelper parametersHelper;
		AdjustmentFromStereoOptionsSet parameters;
		static const AdjustmentFromStereoOptionsSet DEFAULT_PARAMETERS;

		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::FeaturesExtraction2DInterface* featuresExtractor2d;
		CDFF::DFN::FeaturesDescription2DInterface* optionalFeaturesDescriptor2d;
		CDFF::DFN::FeaturesMatching2DInterface* featuresMatcher2d;
		CDFF::DFN::BundleAdjustmentInterface* bundleAdjuster;

		CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;
		CDFF::DFN::CamerasTransformEstimationInterface* cameraTransformEstimator;
		CDFF::DFN::PointCloudReconstruction2DTo3DInterface* reconstructor3dfrom2dmatches;

		#ifdef TESTING
		std::ofstream logFile;
		#endif
		FrameWrapper::FramePtr leftImage;
		FrameWrapper::FramePtr rightImage;
		FrameWrapper::FramePtr filteredLeftImage;
		FrameWrapper::FramePtr filteredRightImage;
		PointCloudWrapper::PointCloudConstPtr imagesCloud;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr leftKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr rightKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr leftFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr rightFeaturesVector;
		std::vector<PointCloudWrapper::PointCloudConstPtr> pointCloudsList;
		std::vector<VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr> featuresVectorsList;
		std::vector< std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> > currentCorrespondenceMapsList;

		/*This is the storage of correspondence Maps, let (L0, R0), (L1, R1), ..., (LN, RN) be a sequence of image pair from the most recent to the oldest.
		* The correspondences between images are stored in the following order (L0-R0), (L0-L1), (L0-R1), ..., (L0-RN), (R0-L0), (R0-L1), (R0-R1), ..., 
		* (R0, LN), (L1-R1), (L1-L2), ..., (L1-RN), ...., (LN-RN). The number N is defined by the parameter numberOfAdjustedStereoPairs.
		* Only the most recent N image pairs are kept in storage, the others will be discarded. */
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr historyCorrespondenceMaps;
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequencePtr workingCorrespondenceMaps;
		PoseWrapper::Poses3DSequencePtr latestCameraPoses;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr emptyFeaturesVector;
		PoseWrapper::Pose3DPtr previousCameraPose;

		MatrixWrapper::Matrix3dPtr fundamentalMatrix;
		PoseWrapper::Pose3DPtr cameraTransform;
		PointCloudWrapper::PointCloudPtr estimatedPointCloud;
		PoseWrapper::Poses3DSequencePtr estimatedCameraPoses;

		void ConfigureExtraParameters();
		void AssignDfnsAlias();

		void FilterImages();
		void ComputeVisualPointFeatures();

		void FilterImage(FrameWrapper::FramePtr image, CDFF::DFN::ImageFilteringInterface* filter, FrameWrapper::FramePtr& filteredImage);
		void ComputeStereoPointCloud();
		void ExtractFeatures(FrameWrapper::FrameConstPtr filteredImage, VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& keypointsVector);
		void DescribeFeatures(
			FrameWrapper::FrameConstPtr image,
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr keypointsVector,
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr& featuresVector);
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr MatchFeatures(
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr sourceFeaturesVector,
			VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr sinkFeaturesVector);
		bool ComputeCameraPoses();

		bool EstimatePointCloud();
		bool EstimateCameraPoses();
		bool ComputeFundamentalMatrix(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputCorrespondenceMap, MatrixWrapper::Matrix3dPtr outputFundamentalMatrix);
		bool ComputeCameraTransform(
			CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputCorrespondenceMap, 
			MatrixWrapper::Matrix3dPtr inputFundamentalMatrix, 
			PoseWrapper::Pose3DPtr outputCameraTransform);
		void ComputeStereoPointCloud(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputCorrespondenceMap);
		void CleanBundleAdjustmentInputs();
		int StaticCastToInt(float value);
		void UpdateHistory();
		void ClearDiscardedData();

		PoseWrapper::Pose3DConstPtr AddAllPointCloudsToMap();
		PoseWrapper::Pose3DConstPtr AddLastPointCloudToMap();
    };
}
}
}

#endif // RECONSTRUCTION3D_ADJUSTMENTFROMSTEREO_HPP

/** @} */
