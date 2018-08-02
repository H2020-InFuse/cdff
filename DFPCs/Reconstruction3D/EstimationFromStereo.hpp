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

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <BundleAdjustment/BundleAdjustmentInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <Transform3DEstimation/Transform3DEstimationInterface.hpp>

#include <VisualPointFeatureVector2D.hpp>
#include <CorrespondenceMap3D.hpp>
#include <CorrespondenceMaps3DSequence.hpp>
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

		//Necessary DFNs
		dfn_ci::ImageFilteringInterface* optionalLeftFilter;
		dfn_ci::ImageFilteringInterface* optionalRightFilter;
		dfn_ci::StereoReconstructionInterface* reconstructor3d;
		dfn_ci::FeaturesExtraction2DInterface* featuresExtractor2d;
		dfn_ci::FeaturesDescription2DInterface* optionalFeaturesDescriptor2d;
		dfn_ci::FeaturesMatching2DInterface* featuresMatcher2d;
		dfn_ci::PointCloudReconstruction2DTo3DInterface* reconstructor3dfrom2dmatches;
		dfn_ci::Transform3DEstimationInterface* transformEstimator;

		//Required Variables for DFNs processing
		FrameWrapper::FramePtr leftImage;
		FrameWrapper::FramePtr rightImage;
		FrameWrapper::FramePtr filteredLeftImage;
		FrameWrapper::FramePtr filteredRightImage;
		PointCloudWrapper::PointCloudConstPtr imageCloud;
		std::vector<PointCloudWrapper::PointCloudConstPtr> imageCloudList;

		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr leftKeypointVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr rightKeypointVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr leftFeatureVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr rightFeatureVector;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftRightCorrespondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftTimeCorrespondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr rightTimeCorrespondenceMap;
		PointCloudWrapper::PointCloudConstPtr triangulatedKeypointCloud;

		#ifdef TESTING
		std::vector<FrameWrapper::FramePtr> leftImageList;
		std::vector<FrameWrapper::FramePtr> rightImageList;
		#endif
		std::vector<VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr> leftFeatureVectorList;
		std::vector<VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr> rightFeatureVectorList;
		std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> leftRightCorrespondenceMapList;
		std::vector<PointCloudWrapper::PointCloudConstPtr> triangulatedKeypointCloudList;

		/*This is the storage of correspondence Maps, let (L0, R0), (L1, R1), ..., (LN, RN) be a sequence of image pair from the most recent to the oldest.
		* The correspondences between images are stored in the following order (L0-R0), (L0-L1), (L0-R1), ..., (L0-RN), (R0-L0), (R0-L1), (R0-R1), ..., 
		* (R0, LN), (L1-R1), (L1-L2), ..., (L1-RN), ...., (LN-RN). The number N is defined by the parameter numberOfAdjustedStereoPairs.
		* Only the most recent N image pairs are kept in storage, the others will be discarded. */
		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr correspondenceMapSequence;
		PoseWrapper::Poses3DSequencePtr cameraPoseList;
		PoseWrapper::Pose3DPtr previousCameraPose;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr emptyFeaturesVector;


		void ConfigureExtraParameters();
		void AssignDfnsAlias();


		void FilterImages();
		void ComputeVisualPointFeatures();
		void FilterImage(FrameWrapper::FramePtr image, dfn_ci::ImageFilteringInterface* filter, FrameWrapper::FramePtr& filteredImage);
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
		void ComputeKeypointCloud(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inputCorrespondenceMap);
		CorrespondenceMap3DWrapper::CorrespondenceMap3DPtr Extract3DCorrespondencesFromTwoImagePairs(std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> mapList,
			std::vector<PointCloudWrapper::PointCloudConstPtr> pointCloudList);

		PoseWrapper::Pose3DConstPtr AddAllPointCloudsToMap();
		PoseWrapper::Pose3DConstPtr AddLastPointCloudToMap();
    };
}
#endif
/* EstimationFromStereo.hpp */
/** @} */
