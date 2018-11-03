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
 * @addtogroup DFPCs
 * 
 *  This DFN chain implements the Reconstruction From Stereo as implementation of the DPFC for Reconsrtruction3D.
 *  This chain operates as follows: 
 *  the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map
 *  camera movement is estimated by matching features in the past left image with the 3d points extracted from the current stereo pair,
 *  point clouds at different time instants are merged together taking into account the movement of the camera.
 * 
 * @{
 */

#ifndef RECONSTRUCTION3D_RECONSTRUCTIONFROMSTEREO_HPP
#define RECONSTRUCTION3D_RECONSTRUCTIONFROMSTEREO_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>

#include <ImageFiltering/ImageFilteringExecutor.hpp>
#include <StereoReconstruction/StereoReconstructionExecutor.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DExecutor.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DExecutor.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DExecutor.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationExecutor.hpp>
#include <PerspectiveNPointSolving/PerspectiveNPointSolvingExecutor.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DExecutor.hpp>

#include "PointCloudMap.hpp"
#include "BundleHistory.hpp"

#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/VisualPointFeatureVector2D.hpp>
#include <Types/CPP/CorrespondenceMap2D.hpp>
#include <Types/CPP/Matrix.hpp>

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
    class ReconstructionFromStereo : public Reconstruction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		ReconstructionFromStereo();
		~ReconstructionFromStereo();
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
		bool firstInput;

		struct ReconstructionFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			float baseline;
			};

		Helpers::ParametersListHelper parametersHelper;
		ReconstructionFromStereoOptionsSet parameters;
		static const ReconstructionFromStereoOptionsSet DEFAULT_PARAMETERS;

		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr EMPTY_FEATURE_VECTOR;
		const std::string LEFT_FEATURE_CATEGORY;
		const std::string RIGHT_FEATURE_CATEGORY;
		const std::string STEREO_CLOUD_CATEGORY;
		const std::string TRIANGULATION_CLOUD_CATEGORY;

		CDFF::DFN::ImageFilteringExecutor* optionalLeftFilter;
		CDFF::DFN::ImageFilteringExecutor* optionalRightFilter;
		CDFF::DFN::FeaturesExtraction2DExecutor* featuresExtractor;
		CDFF::DFN::FeaturesDescription2DExecutor* optionalFeaturesDescriptor;
		CDFF::DFN::FeaturesMatching2DExecutor* featuresMatcher;	
		CDFF::DFN::FundamentalMatrixComputationExecutor* fundamentalMatrixComputer;	
		CDFF::DFN::PerspectiveNPointSolvingExecutor* perspectiveNPointSolver;
		CDFF::DFN::StereoReconstructionExecutor* reconstructor3d;
		CDFF::DFN::PointCloudReconstruction2DTo3DExecutor* reconstructor3dfrom2dmatches;

		#ifdef TESTING
		std::ofstream logFile;
		#endif

		//Data Helpers
		PointCloudWrapper::PointCloudPtr perspectiveCloud;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr perspectiveVector;
		PointCloudWrapper::PointCloudPtr triangulatedKeypointCloud;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr cleanCorrespondenceMap;

		//Helpers
		BundleHistory* bundleHistory;
		PoseWrapper::Pose3D rightToLeftCameraPose;

		void ConfigureExtraParameters();
		void InstantiateDFNExecutors();

		void ComputeCurrentMatches(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		bool ComputeCameraMovement(PoseWrapper::Pose3DConstPtr& previousPoseToPose);
		void CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map, PointCloudWrapper::PointCloudPtr cloud);


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

#endif // RECONSTRUCTION3D_RECONSTRUCTIONFROMSTEREO_HPP

/** @} */
