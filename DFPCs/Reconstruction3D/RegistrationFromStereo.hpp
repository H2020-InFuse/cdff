/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file RegistrationFromStereo.hpp
 * @date 18/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFPCs
 * 
 * This DFN chain implements the Registration From Stereo as implementation of the DPFC for Reconstruction3D.
 * This chain operates as follows: 
 * (i) the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map;
 * (ii) camera movement is estimated by matching 3d features of the current point cloud with the 3d map reconstructed so far;
 * (iii) point clouds at different time instants are merged together taking into account the movement of the camera.
 *
 * This DFPC is configured according to the following parameters (beyond those that are needed to configure the DFN components):
 * @param SearchRadius, the output is given by the point of the reconstructed cloud contained within a sphere of center given by the current camera pose and radius given by this parameter;
 * @param PointCloudMapResolution, the voxel resolution of the output point cloud, if the cloud is denser it will be filtered by PCL voxel filter;
 * @param MatchToReconstructedCloud, whether the cloud is matched to the previous reconstruction or is matched to the previous frame;
 * @param UseAssemblerDfn, whether the assembler DFN is used, if this argument is false the assembly is done by simple overlapping and voxel filtering;
 * @param UseRegistratorDfn, whether the registration DFN is used to further refine the pose estimation obtained by FeaturesMatching3D DFN.
 *
 * Notes: no set of DFNs implementation has produced good result for this DFPC implementation during testing.
 * @{
 */

#ifndef RECONSTRUCTION3D_REGISTRATIONFROMSTEREO_HPP
#define RECONSTRUCTION3D_REGISTRATIONFROMSTEREO_HPP

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
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <PointCloudAssembly/PointCloudAssemblyInterface.hpp>
#include <PointCloudTransformation/PointCloudTransformationInterface.hpp>
#include <PointCloudFiltering/PointCloudFilteringInterface.hpp>
#include <Registration3D/Registration3DInterface.hpp>


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
    class RegistrationFromStereo : public Reconstruction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		RegistrationFromStereo();
		~RegistrationFromStereo();
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
			bool useRegistratorDfn;
			};

		Helpers::ParametersListHelper parametersHelper;
		RegistrationFromStereoOptionsSet parameters;
		static const RegistrationFromStereoOptionsSet DEFAULT_PARAMETERS;

		//DFN Interfaces
		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::FeaturesExtraction3DInterface* featuresExtractor3d;
		CDFF::DFN::FeaturesDescription3DInterface* optionalFeaturesDescriptor3d;
		CDFF::DFN::FeaturesMatching3DInterface* featuresMatcher3d;
		CDFF::DFN::PointCloudAssemblyInterface* cloudAssembler;
		CDFF::DFN::PointCloudTransformationInterface* cloudTransformer;
		CDFF::DFN::PointCloudFilteringInterface* cloudFilter;
		CDFF::DFN::Registration3DInterface* registrator3d;

		//State tracker variables
		BundleHistory* bundleHistory;
		PointCloudMap pointCloudMap;
		bool firstInput;

		//Parameters Configuration method
		void ConfigureExtraParameters();

		//DFN instantuation method
		void InstantiateDFNs();

		//Core computation methods that execute a step of the DFPC pipeline
		void UpdatePose(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr outputFeatures);
		void UpdatePointCloud(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr outputFeatures);
		void ComputeVisualFeatures(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputFeatures);

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

#endif // RECONSTRUCTION3D_REGISTRATIONFROMSTEREO_HPP

/** @} */
