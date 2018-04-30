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

		dfn_ci::ImageFilteringInterface* optionalLeftFilter;
		dfn_ci::ImageFilteringInterface* optionalRightFilter;
		dfn_ci::StereoReconstructionInterface* reconstructor3D;
		dfn_ci::FeaturesExtraction3DInterface* featuresExtractor3d;
		dfn_ci::FeaturesDescription3DInterface* optionalFeaturesDescriptor3d;
		dfn_ci::FeaturesMatching3DInterface* featuresMatcher3d;

		FrameWrapper::FrameConstPtr leftImage;
		FrameWrapper::FrameConstPtr rightImage;
		FrameWrapper::FrameConstPtr filteredLeftImage;
		FrameWrapper::FrameConstPtr filteredRightImage;
		PointCloudWrapper::PointCloudConstPtr pointCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr pointCloudKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr pointCloudFeaturesVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneFeaturesVector;
		PoseWrapper::Pose3DConstPtr cameraPoseInScene;
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
