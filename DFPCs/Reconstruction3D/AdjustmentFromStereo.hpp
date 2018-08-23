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

#include <ImageFiltering/ImageFilteringExecutor.hpp>
#include <StereoReconstruction/StereoReconstructionExecutor.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DExecutor.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DExecutor.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DExecutor.hpp>
#include <BundleAdjustment/BundleAdjustmentExecutor.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationExecutor.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationExecutor.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DExecutor.hpp>

#include <VisualPointFeatureVector2D.hpp>
#include <CorrespondenceMap2D.hpp>
#include <CorrespondenceMaps2DSequence.hpp>
#include <PosesSequence.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Frame.hpp>
#include <Pose.hpp>
#include <Matrix.hpp>

#include "PointCloudMap.hpp"
#include "BundleHistory.hpp"
#include "MultipleCorrespondences2DRecorder.hpp"

#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>


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

		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr EMPTY_FEATURE_VECTOR;
		const std::string LEFT_FEATURE_CATEGORY;
		const std::string RIGHT_FEATURE_CATEGORY;
		const std::string STEREO_CLOUD_CATEGORY;
		const std::string TRIANGULATION_CLOUD_CATEGORY;

		//DFN Executors
		CDFF::DFN::ImageFilteringExecutor* optionalLeftFilter;
		CDFF::DFN::ImageFilteringExecutor* optionalRightFilter;
		CDFF::DFN::StereoReconstructionExecutor* reconstructor3d;
		CDFF::DFN::FeaturesExtraction2DExecutor* featuresExtractor2d;
		CDFF::DFN::FeaturesDescription2DExecutor* optionalFeaturesDescriptor2d;
		CDFF::DFN::FeaturesMatching2DExecutor* featuresMatcher2d;
		CDFF::DFN::BundleAdjustmentExecutor* bundleAdjuster;
		CDFF::DFN::FundamentalMatrixComputationExecutor* fundamentalMatrixComputer;
		CDFF::DFN::CamerasTransformEstimationExecutor* cameraTransformEstimator;
		CDFF::DFN::PointCloudReconstruction2DTo3DExecutor* reconstructor3dfrom2dmatches;

		#ifdef TESTING
		std::ofstream logFile;
		#endif

		//Helpers
		BundleHistory* bundleHistory;
		MultipleCorrespondences2DRecorder* correspondencesRecorder;
		PoseWrapper::Pose3D rightToLeftCameraPose;

		//Intermediate data
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr cleanCorrespondenceMap;
		PointCloudWrapper::PointCloudPtr triangulatedKeypointCloud;
		PoseWrapper::Poses3DSequencePtr estimatedCameraPoses;

		void ConfigureExtraParameters();
		void InstantiateDFNExecutors();

		void ComputeVisualPointFeatures(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map, PointCloudWrapper::PointCloudPtr cloud);
		void ComputeStereoPointCloud(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void CreateWorkingCorrespondences();
		void CreateWorkingCorrespondences(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featureVector);
		bool ComputeCameraPoses(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);

		void EstimateCameraPoses();

		void AddAllPointCloudsToMap(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);
		void AddLastPointCloudToMap(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);

		/*
		* Inline Methods
		*
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

#endif // RECONSTRUCTION3D_ADJUSTMENTFROMSTEREO_HPP

/** @} */
