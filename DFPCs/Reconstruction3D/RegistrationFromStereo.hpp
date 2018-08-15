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
 * @addtogroup DFNs
 * 
 *  This DFN chain implements the Registration From Stereo as implementation of the DPFC for Reconstruction3D.
 *  This chain operates as follows: 
 *  the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map
 *  camera movement is estimated by matching 3d features of the current point cloud with the 3d map reconstructed so far,
 *  point clouds at different time instants are merged together taking into account the movement of the camera.
 * 
 * @{
 */

#ifndef REGISTRATION_FROM_STEREO_HPP
#define REGISTRATION_FROM_STEREO_HPP

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

#include "PointCloudMap.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>
#include <Frame.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>
#include <VisualPointFeatureVector3D.hpp>

namespace dfpc_ci {

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
			};

		Helpers::ParametersListHelper parametersHelper;
		RegistrationFromStereoOptionsSet parameters;
		static const RegistrationFromStereoOptionsSet DEFAULT_PARAMETERS;

		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3D;
		CDFF::DFN::FeaturesExtraction3DInterface* featuresExtractor3d;
		CDFF::DFN::FeaturesDescription3DInterface* optionalFeaturesDescriptor3d;
		CDFF::DFN::FeaturesMatching3DInterface* featuresMatcher3d;

		FrameWrapper::FramePtr leftImage;
		FrameWrapper::FramePtr rightImage;
		FrameWrapper::FramePtr filteredLeftImage;
		FrameWrapper::FramePtr filteredRightImage;
		PointCloudWrapper::PointCloudPtr pointCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr pointCloudKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr pointCloudFeaturesVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneFeaturesVector;
		PoseWrapper::Pose3DPtr cameraPoseInScene;
		PoseWrapper::Pose3DPtr previousCameraPoseInScene;

		void ConfigureExtraParameters();
		void AssignDfnsAlias();

		bool ComputeCameraMovement();
		void ComputePointCloud();

		void FilterLeftImage();
		void FilterRightImage();
		void ComputeStereoPointCloud();
		void ExtractPointCloudFeatures();
		void DescribePointCloudFeatures();
		bool MatchPointCloudWithSceneFeatures();
    };
}
#endif
/* RegistrationFromStereo.hpp */
/** @} */
