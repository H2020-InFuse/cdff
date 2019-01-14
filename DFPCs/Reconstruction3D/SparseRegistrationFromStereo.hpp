/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file SparseRegistrationFromStereo.hpp
 * @date 05/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFPCs
 * 
 *  This DFN chain implements the Registration From Stereo as implementation of the DPFC for Reconstruction3D.
 *  This chain operates as follows: 
 *  (i) the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map;
 *  (ii) 3f features are extracted from the point cloud with a 3d detector
 *  (iii) camera movement is estimated by registration of the newly detected features on the features stored in the point cloud map reconstructed so far;
 *  (iv) point clouds at different time instants are merged together taking into account the movement of the camera.
 * 
 * This DFPC is configured according to the following parameters (beyond those that are needed to configure the DFN components):
 * @param SearchRadius, the output is given by the point of the reconstructed cloud contained within a sphere of center given by the current camera pose and radius given by this parameter;
 * @param PointCloudMapResolution, the voxel resolution of the output point cloud, if the cloud is denser it will be filtered by PCL voxel filter;
 * @param MatchToReconstructedCloud, whether the cloud is matched to the previous reconstruction or is matched to the previous frame;
 * @param UseAssemblerDfn, whether the assembler DFN is used, if this argument is false the assembly is done by simple overlapping and voxel filtering.
 *
 * Notes: no set of DFNs implementation has produced good result for this DFPC during testing.
 * @{
 */

#ifndef RECONSTRUCTION3D_SPARSEREGISTRATIONFROMSTEREO_HPP
#define RECONSTRUCTION3D_SPARSEREGISTRATIONFROMSTEREO_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <Registration3D/Registration3DInterface.hpp>
#include <PointCloudAssembly/PointCloudAssemblyInterface.hpp>
#include <PointCloudTransform/PointCloudTransformInterface.hpp>
#include <PointCloudFiltering/PointCloudFilteringInterface.hpp>

#include "PointCloudMap.hpp"
#include "BundleHistory.hpp"

#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>
#include <Types/CPP/Frame.hpp>
#include <Types/CPP/PointCloud.hpp>
#include <Types/CPP/Pose.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>

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
    class SparseRegistrationFromStereo : public Reconstruction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		SparseRegistrationFromStereo();
		~SparseRegistrationFromStereo();
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
		struct RegistrationFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			bool matchToReconstructedCloud;
			bool useAssemblerDfn;
			};

		Helpers::ParametersListHelper parametersHelper;
		RegistrationFromStereoOptionsSet parameters;
		static const RegistrationFromStereoOptionsSet DEFAULT_PARAMETERS;

		//Pointers to DFN instances
		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::FeaturesExtraction3DInterface* featuresExtractor3d;
		CDFF::DFN::Registration3DInterface* registrator3d;
		CDFF::DFN::PointCloudAssemblyInterface* cloudAssembler;
		CDFF::DFN::PointCloudTransformInterface* cloudTransformer;
		CDFF::DFN::PointCloudFilteringInterface* cloudFilter;

		//State tracker variables
		BundleHistory* bundleHistory;
		PointCloudWrapper::PointCloudPtr featureCloud;
		PointCloudMap pointCloudMap;
		bool firstInput;

		//Parameters Configuration method
		void ConfigureExtraParameters();

		//DFN instantuation method
		void InstantiateDFNs();

		//Core computation method that execute a step of the DFPC pipeline
		void ComputeFeatureCloud(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr vector);
		void UpdatePose(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr keypointVector);
		void UpdatePointCloud(PointCloudWrapper::PointCloudConstPtr inputCloud);

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

#endif // RECONSTRUCTION3D_SPARSEREGISTRATIONFROMSTEREO_HPP

/** @} */
