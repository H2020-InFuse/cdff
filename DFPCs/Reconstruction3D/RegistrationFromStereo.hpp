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
 *  This DFN chain implements the Registration From Stereo as implementation of the DPFC for Reconstruction3D.
 *  This chain operates as follows: 
 *  the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map
 *  camera movement is estimated by matching 3d features of the current point cloud with the 3d map reconstructed so far,
 *  point clouds at different time instants are merged together taking into account the movement of the camera.
 * 
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

#include <ImageFiltering/ImageFilteringExecutor.hpp>
#include <StereoReconstruction/StereoReconstructionExecutor.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DExecutor.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DExecutor.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DExecutor.hpp>
#include <PointCloudAssembly/PointCloudAssemblyExecutor.hpp>
#include <PointCloudTransform/PointCloudTransformExecutor.hpp>
#include <PointCloudFiltering/PointCloudFilteringExecutor.hpp>

#include "PointCloudMap.hpp"
#include "BundleHistory.hpp"

#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>
#include <Frame.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <VisualPointFeatureVector3D.hpp>

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

		CDFF::DFN::ImageFilteringExecutor* optionalLeftFilter;
		CDFF::DFN::ImageFilteringExecutor* optionalRightFilter;
		CDFF::DFN::StereoReconstructionExecutor* reconstructor3d;
		CDFF::DFN::FeaturesExtraction3DExecutor* featuresExtractor3d;
		CDFF::DFN::FeaturesDescription3DExecutor* optionalFeaturesDescriptor3d;
		CDFF::DFN::FeaturesMatching3DExecutor* featuresMatcher3d;
		CDFF::DFN::PointCloudAssemblyExecutor* cloudAssembler;
		CDFF::DFN::PointCloudTransformExecutor* cloudTransformer;
		CDFF::DFN::PointCloudFilteringExecutor* cloudFilter;

		#ifdef TESTING
		std::ofstream logFile;
		void WriteOutputToLogFile();
		#endif

		//Helpers
		BundleHistory* bundleHistory;

		void ConfigureExtraParameters();
		void InstantiateDFNExecutors();

		void UpdatePose(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr outputFeatures);
		void UpdatePointCloud(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr outputFeatures);
		void ComputeVisualFeatures(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& outputFeatures);

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

#endif // RECONSTRUCTION3D_REGISTRATIONFROMSTEREO_HPP

/** @} */
