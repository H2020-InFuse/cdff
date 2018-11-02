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

#include <ImageFiltering/ImageFilteringInterface.hpp>
#include <StereoReconstruction/StereoReconstructionInterface.hpp>
#include <FeaturesExtraction3D/FeaturesExtraction3DInterface.hpp>
#include <FeaturesDescription3D/FeaturesDescription3DInterface.hpp>
#include <FeaturesMatching3D/FeaturesMatching3DInterface.hpp>
#include <PointCloudAssembly/PointCloudAssemblyInterface.hpp>
#include <PointCloudTransform/PointCloudTransformInterface.hpp>
#include <PointCloudFiltering/PointCloudFilteringInterface.hpp>
#include <Registration3D/Registration3DInterface.hpp>


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
			bool useRegistratorDfn;
			};

		Helpers::ParametersListHelper parametersHelper;
		RegistrationFromStereoOptionsSet parameters;
		static const RegistrationFromStereoOptionsSet DEFAULT_PARAMETERS;

		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::FeaturesExtraction3DInterface* featuresExtractor3d;
		CDFF::DFN::FeaturesDescription3DInterface* optionalFeaturesDescriptor3d;
		CDFF::DFN::FeaturesMatching3DInterface* featuresMatcher3d;
		CDFF::DFN::PointCloudAssemblyInterface* cloudAssembler;
		CDFF::DFN::PointCloudTransformInterface* cloudTransformer;
		CDFF::DFN::PointCloudFilteringInterface* cloudFilter;
		CDFF::DFN::Registration3DInterface* registrator3d;

		#ifdef TESTING
		std::ofstream logFile;
		void WriteOutputToLogFile();
		#endif

		//Helpers
		BundleHistory* bundleHistory;

		void ConfigureExtraParameters();
		void InstantiateDFNs();

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
