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
 * @addtogroup DFNs
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

#ifndef ADJUSTMENTFROMSTEREO
#define ADJUSTMENTFROMSTEREO

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


namespace dfpc_ci {

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
		dfn_ci::ImageFilteringExecutor* optionalLeftFilter;
		dfn_ci::ImageFilteringExecutor* optionalRightFilter;
		dfn_ci::StereoReconstructionExecutor* reconstructor3d;
		dfn_ci::FeaturesExtraction2DExecutor* featuresExtractor2d;
		dfn_ci::FeaturesDescription2DExecutor* optionalFeaturesDescriptor2d;
		dfn_ci::FeaturesMatching2DExecutor* featuresMatcher2d;
		dfn_ci::BundleAdjustmentExecutor* bundleAdjuster;
		dfn_ci::FundamentalMatrixComputationExecutor* fundamentalMatrixComputer;
		dfn_ci::CamerasTransformEstimationExecutor* cameraTransformEstimator;
		dfn_ci::PointCloudReconstruction2DTo3DExecutor* reconstructor3dfrom2dmatches;

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
		void CleanLowScoringMatches(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftRightCorrespondenceMap, CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr output);
		void CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr map, PointCloudWrapper::PointCloudPtr cloud);
		void ComputeStereoPointCloud(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void CreateWorkingCorrespondences();
		void CreateWorkingCorrespondences(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featureVector);
		bool ComputeCameraPoses(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);

		void EstimateCameraPoses();

		void AddAllPointCloudsToMap(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);
		void AddLastPointCloudToMap(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);
    };
}
#endif
/* AdjustmentFromStereo.hpp */
/** @} */
