/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file EstimationFromStereo.hpp
 * @date 25/07/2018
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
 *  3d positions of the keypoints are found by triangulation;
 *  the 3d point correspondences among the latest N image pairs are used for the computation of the camera pose by means 3d transform estimation dfn.    
 * 
 * @{
 */

#ifndef ESTIMATIONFROMSTEREO
#define ESTIMATIONFROMSTEREO

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
#include <Transform3DEstimation/Transform3DEstimationExecutor.hpp>

#include <VisualPointFeatureVector2D.hpp>
#include <CorrespondenceMap3D.hpp>
#include <CorrespondenceMaps3DSequence.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Frame.hpp>

#include "PointCloudMap.hpp"
#include "BundleHistory.hpp"
#include "MultipleCorrespondencesRecorder.hpp"

#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>
#include <Frame.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Matrix.hpp>

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
    class EstimationFromStereo : public Reconstruction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		EstimationFromStereo();
		~EstimationFromStereo();
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

		struct EstimationFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			int numberOfAdjustedStereoPairs;
			float baseline;
			};

		Helpers::ParametersListHelper parametersHelper;
		EstimationFromStereoOptionsSet parameters;
		static const EstimationFromStereoOptionsSet DEFAULT_PARAMETERS;
		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr EMPTY_FEATURE_VECTOR;
		const std::string LEFT_FEATURE_CATEGORY;
		const std::string RIGHT_FEATURE_CATEGORY;
		const std::string STEREO_CLOUD_CATEGORY;
		const std::string TRIANGULATION_CLOUD_CATEGORY;

		#ifdef TESTING
		std::ofstream logFile;
		#endif

		//DFN Executors
		dfn_ci::ImageFilteringExecutor* optionalLeftFilter;
		dfn_ci::ImageFilteringExecutor* optionalRightFilter;
		dfn_ci::StereoReconstructionExecutor* reconstructor3d;
		dfn_ci::FeaturesExtraction2DExecutor* featuresExtractor2d;
		dfn_ci::FeaturesDescription2DExecutor* optionalFeaturesDescriptor2d;
		dfn_ci::FeaturesMatching2DExecutor* featuresMatcher2d;
		dfn_ci::PointCloudReconstruction2DTo3DExecutor* reconstructor3dfrom2dmatches;
		dfn_ci::Transform3DEstimationExecutor* transformEstimator;

		//Helpers
		BundleHistory* bundleHistory;
		MultipleCorrespondencesRecorder* correspondencesRecorder;
		PoseWrapper::Pose3D rightToLeftCameraPose;

		//Intermediate data
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr cleanCorrespondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr leftTimeCorrespondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr rightTimeCorrespondenceMap;

		void ConfigureExtraParameters();
		void InstantiateDFNExecutors();

		void ComputeVisualPointFeatures(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void CleanLowScoringMatches(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftRightCorrespondenceMap);
		void ComputeStereoPointCloud(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void CreateWorkingCorrespondences();
		bool ComputeCameraPoses(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);

		void AddAllPointCloudsToMap(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);
		void AddLastPointCloudToMap(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);

		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr CreateCorrespondenceMapsSequence();
    };
}
#endif
/* EstimationFromStereo.hpp */
/** @} */
