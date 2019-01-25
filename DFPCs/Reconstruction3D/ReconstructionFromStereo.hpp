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
 *  (i) the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map;
 *  (ii) camera movement is estimated by matching features in the past left image with the 3d points extracted from the current stereo pair,
 *  (iii) point clouds at different time instants are merged together taking into account the movement of the camera.
 *
 * This DFPC is configured according to the following parameters (beyond those that are needed to configure the DFN components):
 * @param SearchRadius, the output is given by the point of the reconstructed cloud contained within a sphere of center given by the current camera pose and radius given by this parameter;
 * @param PointCloudMapResolution, the voxel resolution of the output point cloud, if the cloud is denser it will be filtered by PCL voxel filter;
 * @param Baseline, the baseline of the stereo camera pair.
 *
 * Notes: no set of DFNs implementation has produced good result for this DFPC implementation during testing.
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

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <FeaturesMatching2D/FeaturesMatching2DInterface.hpp>
#include <FeaturesExtraction2D/FeaturesExtraction2DInterface.hpp>
#include <FeaturesDescription2D/FeaturesDescription2DInterface.hpp>
#include <FundamentalMatrixComputation/FundamentalMatrixComputationInterface.hpp>
#include <PerspectiveNPointSolving/PerspectiveNPointSolvingInterface.hpp>
#include <PointCloudReconstruction2DTo3D/PointCloudReconstruction2DTo3DInterface.hpp>

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
		void run() override;
		void setup() override;

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

		//General configuration helper
		DfpcConfigurator configurator;

		//Additional DFPC Parameters
		struct ReconstructionFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			float baseline;
			};

		Helpers::ParametersListHelper parametersHelper;
		ReconstructionFromStereoOptionsSet parameters;
		static const ReconstructionFromStereoOptionsSet DEFAULT_PARAMETERS;

		//Costants
		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr EMPTY_FEATURE_VECTOR;

		//Category for the BundleHistory instance
		const std::string LEFT_FEATURE_CATEGORY;
		const std::string RIGHT_FEATURE_CATEGORY;
		const std::string STEREO_CLOUD_CATEGORY;
		const std::string TRIANGULATION_CLOUD_CATEGORY;

		//Pointers to DFN instances
		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::FeaturesExtraction2DInterface* featuresExtractor;
		CDFF::DFN::FeaturesDescription2DInterface* optionalFeaturesDescriptor;
		CDFF::DFN::FeaturesMatching2DInterface* featuresMatcher;	
		CDFF::DFN::FundamentalMatrixComputationInterface* fundamentalMatrixComputer;	
		CDFF::DFN::PerspectiveNPointSolvingInterface* perspectiveNPointSolver;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::PointCloudReconstruction2DTo3DInterface* reconstructor3dfrom2dmatches;

		//Support variable for storing intermediate data
		PointCloudWrapper::PointCloudPtr perspectiveCloud;
		VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DPtr perspectiveVector;
		PointCloudWrapper::PointCloudPtr triangulatedKeypointCloud;
		CorrespondenceMap2DWrapper::CorrespondenceMap2DPtr cleanCorrespondenceMap;

		//State tracker variables
		BundleHistory* bundleHistory;
		PoseWrapper::Pose3D rightToLeftCameraPose;
		PointCloudMap pointCloudMap;
		bool firstInput;

		//Parameters Configuration method
		void ConfigureExtraParameters();

		//DFN instantuation method
		void InstantiateDFNs();

		//Core computation methods that execute a step of the DFPC pipeline
		void ComputeCurrentMatches(FrameWrapper::FrameConstPtr filteredLeftImage, FrameWrapper::FrameConstPtr filteredRightImage);
		bool ComputeCameraMovement(PoseWrapper::Pose3DConstPtr& previousPoseToPose);
		void CleanUnmatchedFeatures(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr map, PointCloudWrapper::PointCloudPtr cloud);

		/*
		* Inline helper Method
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
