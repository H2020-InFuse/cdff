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
#include <PerspectiveNPointSolving/PerspectiveNPointSolvingInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>

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

		//DFN Interfaces
		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::FeaturesExtraction2DInterface* featuresExtractor2d;
		CDFF::DFN::FeaturesDescription2DInterface* optionalFeaturesDescriptor2d;
		CDFF::DFN::FeaturesMatching2DInterface* featuresMatcher2d;
		CDFF::DFN::BundleAdjustmentInterface* bundleAdjuster;
		CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;
		CDFF::DFN::PerspectiveNPointSolvingInterface* perspectiveNPointSolver;
		CDFF::DFN::PointCloudReconstruction2DTo3DInterface* reconstructor3dfrom2dmatches;

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

		//Support data when using initial estimation
		PoseWrapper::Poses3DSequencePtr estimatedCameraPoses;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr presentKeypointVector; 
		PointCloudWrapper::PointCloudPtr keypointCloud;

		void ConfigureExtraParameters();
		void InstantiateDFNs();

		void ComputeVisualPointFeatures(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map, PointCloudWrapper::PointCloudPtr cloud);
		void ComputeStereoPointCloud(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void CreateWorkingCorrespondences();
		void CreateWorkingCorrespondences(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr featureVector);
		bool ComputeCameraPoses(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);
		void EstimatePose(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr pastLeftRightCorrespondenceMap, 
			CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr leftPresentPastCorrespondenceMap, 
			PointCloudWrapper::PointCloudConstPtr pastCloud, PoseWrapper::Pose3DConstPtr& pose, bool& success);

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
