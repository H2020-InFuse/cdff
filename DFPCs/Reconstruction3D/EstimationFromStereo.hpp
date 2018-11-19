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
 * @addtogroup DFPCs
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

#ifndef RECONSTRUCTION3D_ESTIMATIONFROMSTEREO_HPP
#define RECONSTRUCTION3D_ESTIMATIONFROMSTEREO_HPP

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
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <CamerasTransformEstimation/CamerasTransformEstimationInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>
#include <Transform3DEstimation/Transform3DEstimationInterface.hpp>

#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/CorrespondenceMap3D.hpp>
#include <Types/CPP/CorrespondenceMaps3DSequence.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/PosesSequence.hpp>
#include <Types/CPP/Matrix.hpp>

#include "PointCloudMap.hpp"
#include "BundleHistory.hpp"
#include "MultipleCorrespondencesRecorder.hpp"

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

		//DFN Interfaces
		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::FeaturesExtraction2DInterface* featuresExtractor2d;
		CDFF::DFN::FeaturesDescription2DInterface* optionalFeaturesDescriptor2d;
		CDFF::DFN::FeaturesMatching2DInterface* featuresMatcher2d;
		CDFF::DFN::PointCloudReconstruction2DTo3DInterface* reconstructor3dfrom2dmatches;
		CDFF::DFN::Transform3DEstimationInterface* transformEstimator;
		CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;

		//Helpers
		BundleHistory* bundleHistory;
		MultipleCorrespondencesRecorder* correspondencesRecorder;
		PoseWrapper::Pose3D rightToLeftCameraPose;

		//Intermediate data
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr leftTimeCorrespondenceMap;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr rightTimeCorrespondenceMap;

		void ConfigureExtraParameters();
		void InstantiateDFNs();

		void ComputeVisualPointFeatures(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void ComputeStereoPointCloud(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		void CreateWorkingCorrespondences();
		bool ComputeCameraPoses(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);

		void AddAllPointCloudsToMap(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);
		void AddLastPointCloudToMap(PoseWrapper::Poses3DSequenceConstPtr& cameraPoses);

		CorrespondenceMap3DWrapper::CorrespondenceMaps3DSequencePtr CreateCorrespondenceMapsSequence();

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

#endif // RECONSTRUCTION3D_ESTIMATIONFROMSTEREO_HPP

/** @} */
