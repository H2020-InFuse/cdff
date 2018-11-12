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
 *  the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map
 *  3f features are extracted from the point cloud with a 3d detector
 *  camera movement is estimated by registration of the newly detected features on the features stored in the point cloud map reconstructed so far.
 *  point clouds at different time instants are merged together taking into account the movement of the camera.
 * 
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

		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3d;
		CDFF::DFN::FeaturesExtraction3DInterface* featuresExtractor3d;
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
		PointCloudWrapper::PointCloudPtr featureCloud;

		void ConfigureExtraParameters();
		void InstantiateDFNs();
		void ComputeFeatureCloud(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr vector);

		void UpdatePose(PointCloudWrapper::PointCloudConstPtr inputCloud, VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr keypointVector);
		void UpdatePointCloud(PointCloudWrapper::PointCloudConstPtr inputCloud);

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

#endif // RECONSTRUCTION3D_SPARSEREGISTRATIONFROMSTEREO_HPP

/** @} */
