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
 *  The N-th to last camera pose is finalized, it is assumed to be the real camera pose.
 *  The N-th to last point cloud is merged with the 3d map at the finalized camera pose.
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

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <BundleAdjustment/BundleAdjustmentInterface.hpp>

#include <VisualPointFeatureVector2D.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <Frame.hpp>

#include "PointCloudMap.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>


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
			};

		Helpers::ParametersListHelper parametersHelper;
		AdjustmentFromStereoOptionsSet parameters;
		static const AdjustmentFromStereoOptionsSet DEFAULT_PARAMETERS;

		dfn_ci::ImageFilteringInterface* optionalLeftFilter;
		dfn_ci::ImageFilteringInterface* optionalRightFilter;
		dfn_ci::StereoReconstructionInterface* reconstructor3d;
		dfn_ci::FeaturesExtraction2DInterface* featuresExtractor2d;
		dfn_ci::FeaturesDescription2DInterface* optionalFeaturesDescriptor2d;
		dfn_ci::FeaturesMatching2DInterface* featuresMatcher2d;
		dfn_ci::BundleAdjustmentInterface* bundleAdjuster;

		FrameWrapper::FrameConstPtr leftImage;
		FrameWrapper::FrameConstPtr rightImage;
		FrameWrapper::FrameConstPtr filteredLeftImage;
		FrameWrapper::FrameConstPtr filteredRightImage;
		PointCloudWrapper::PointCloudConstPtr imagesCloud;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr leftKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr rightKeypointsVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr leftFeaturesVector;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr rightFeaturesVector;
		std::vector<PointCloudWrapper::PointCloudConstPtr> pointCloudsList;
		std::vector<VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr> featuresVectorsList;
		std::vector< std::vector<CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr> > currentCorrespondenceMapsList;
		CorrespondenceMap2DWrapper::CorrespondenceMaps2DSequenceConstPtr latestCorrespondenceMaps;
		PoseWrapper::Poses3DSequenceConstPtr latestCameraPoses;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr emptyFeaturesVector;
		PoseWrapper::Pose3DPtr previousCameraPose;

		void ConfigureExtraParameters();
		void AssignDfnsAlias();

		void FilterImages();
		void ComputeVisualPointFeatures();

		void FilterImage(FrameWrapper::FrameConstPtr image, dfn_ci::ImageFilteringInterface* filter, FrameWrapper::FrameConstPtr& filteredImage);
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

		PoseWrapper::Pose3DConstPtr AddAllPointCloudsToMap();
		PoseWrapper::Pose3DConstPtr AddLastPointCloudToMap();
    };
}
#endif
/* AdjustmentFromStereo.hpp */
/** @} */
