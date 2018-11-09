/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file DenseRegistrationFromStereo.hpp
 * @date 31/05/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFPCs
 * 
 *  This DFN chain implements the Registration From Stereo as implementation of the DPFC for Reconstruction3D.
 *  This chain operates as follows: 
 *  the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map
 *  camera movement is estimated by registration of the newly detected point cloud on the point cloud map reconstructed so far.
 *  point clouds at different time instants are merged together taking into account the movement of the camera.
 * 
 * @{
 */

#ifndef RECONSTRUCTION3D_DENSEREGISTRATIONFROMSTEREO_HPP
#define RECONSTRUCTION3D_DENSEREGISTRATIONFROMSTEREO_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Reconstruction3D/Reconstruction3DInterface.hpp>

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
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

#include <Converters/PointCloudToPclPointCloudConverter.hpp>

#ifdef TESTING
#include <fstream>
#endif

namespace CDFF
{
namespace DFPC
{
namespace Reconstruction3D
{

/**
 * Dense 3D reconstruction. This DFPC computes a point cloud from a pair of stereo cameras, matches the computed cloud to a previous reconstruction or previous frame by means of a registration DFN, 
 * computes the pose of the new cloud into the reconstructed cloud, and extends the reconstruction by merging the new cloud into the reconstructed cloud at the computed pose.
 * This DFPC is configured according to the following parameters (beyond those that are needed to configure the DFN components):
 *
 * @param SearchRadius, the output is given by the point of the reconstructed cloud contained within a sphere of center given by the current camera pose and radius given by this parameter;
 * @param PointCloudMapResolution, the voxel resolution of the output point cloud, if the cloud is denser it will be filtered by PCL voxel filter;
 * @param MatchToReconstructedCloud, whether the cloud is matched to the previous reconstruction or is matched to the previous frame;
 * @param UseAssemblerDfn, whether the assembler DFN is used, if this argument is false the assembly is done by simple overlapping and voxel filtering;
 * @param CloudUpdateTime, the number of frames between two point cloud assembly, intermediate frames are used only to update the pose and will not extend the point cloud;
 * @param SaveCloudsToFile, whether to save the output clouds to file;
 * @param CloudSaveTime, the number of frames between two saving of the point cloud, clouds will not be save during intermediate frames;
 * @param cloudSavePath, the folder path where the point clouds are saved.
 */

    class DenseRegistrationFromStereo : public Reconstruction3DInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
		DenseRegistrationFromStereo();
		~DenseRegistrationFromStereo();
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

		enum CloudUpdateType
			{
			TimePassed,
			DistanceCovered,
			MaximumOverlapping
			};
		class CloudUpdateTypeHelper : public Helpers::ParameterHelper<CloudUpdateType, std::string>
			{
			public:
				CloudUpdateTypeHelper(const std::string& parameterName, CloudUpdateType& boundVariable, const CloudUpdateType& defaultValue);
			private:
				CloudUpdateType Convert(const std::string& value);
			};

		struct RegistrationFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			bool matchToReconstructedCloud;
			bool useAssemblerDfn;

			CloudUpdateType cloudUpdateType;
			int cloudUpdateTime;
			double cloudUpdateTranslationDistance;
			double cloudUpdateOrientationDistance;
			float overlapThreshold;

			bool saveCloudsToFile;
			int cloudSaveTime;
			std::string cloudSavePath;
			};

		Helpers::ParametersListHelper parametersHelper;
		RegistrationFromStereoOptionsSet parameters;
		static const RegistrationFromStereoOptionsSet DEFAULT_PARAMETERS;
		const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr EMPTY_FEATURE_VECTOR;

		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::Registration3DInterface* registrator3d;
		CDFF::DFN::PointCloudAssemblyInterface* cloudAssembler;
		CDFF::DFN::PointCloudTransformInterface* cloudTransformer;
		CDFF::DFN::PointCloudFilteringInterface* cloudFilter;

		#ifdef TESTING
		std::ofstream logFile;
		void WriteOutputToLogFile();
		#endif

		//Helpers
		BundleHistory* bundleHistory;
		PoseWrapper::Pose3D outputPoseAtLastMerge;
		bool outputPoseAtLastMergeSet;
		Converters::PointCloudToPclPointCloudConverter pointCloudToPclPointCloudConverter;

		void ConfigureExtraParameters();
		void InstantiateDFNs();

		void UpdatePose(PointCloudWrapper::PointCloudConstPtr inputCloud);
		void UpdatePointCloudOnTimePassed(PointCloudWrapper::PointCloudConstPtr inputCloud);
		void UpdatePointCloudOnDistanceCovered(PointCloudWrapper::PointCloudConstPtr inputCloud);
		void UpdatePointCloudOnMaximumOverlapping(PointCloudWrapper::PointCloudConstPtr inputCloud);
		void MergePointCloud(PointCloudWrapper::PointCloudConstPtr inputCloud);

		void SaveOutputCloud();
		float ComputeOverlappingRatio(PointCloudWrapper::PointCloudConstPtr cloud, PointCloudWrapper::PointCloudConstPtr sceneCloud);

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

#endif // RECONSTRUCTION3D_DENSEREGISTRATIONFROMSTEREO_HPP

/** @} */
