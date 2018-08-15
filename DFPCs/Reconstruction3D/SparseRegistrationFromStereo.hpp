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
 * @addtogroup DFNs
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

#ifndef SPARSE_REGISTRATION_FROM_STEREO_HPP
#define SPARSE_REGISTRATION_FROM_STEREO_HPP

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

#include "PointCloudMap.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>
#include <Frame.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>


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

		struct SparseRegistrationFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			};

		Helpers::ParametersListHelper parametersHelper;
		SparseRegistrationFromStereoOptionsSet parameters;
		static const SparseRegistrationFromStereoOptionsSet DEFAULT_PARAMETERS;

		CDFF::DFN::ImageFilteringInterface* optionalLeftFilter;
		CDFF::DFN::ImageFilteringInterface* optionalRightFilter;
		CDFF::DFN::StereoReconstructionInterface* reconstructor3D;
		CDFF::DFN::FeaturesExtraction3DInterface* featuresExtractor;
		CDFF::DFN::Registration3DInterface* cloudRegistrator;

		FrameWrapper::FramePtr leftImage;
		FrameWrapper::FramePtr rightImage;
		FrameWrapper::FramePtr filteredLeftImage;
		FrameWrapper::FramePtr filteredRightImage;
		PointCloudWrapper::PointCloudPtr imagesCloud;
		PointCloudWrapper::PointCloudConstPtr imagesSparseCloud;
		PointCloudWrapper::PointCloudConstPtr sceneSparseCloud;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DPtr imagesCloudKeypointsVector;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr sceneCloudKeypointsVector;
		PoseWrapper::Pose3DPtr cameraPoseInScene;
		PoseWrapper::Pose3DPtr previousCameraPoseInScene;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr emptyFeaturesVector;

		void ConfigureExtraParameters();
		void AssignDfnsAlias();

		void ComputePointCloud();

		void FilterLeftImage();
		void FilterRightImage();
		void ComputeStereoPointCloud();
		void ComputeImagesCloudKeypoints();
		PointCloudWrapper::PointCloudConstPtr FromKeypointsToCloud(VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr keypointsVector);
		bool RegisterImagesCloudOnScene();
    };
}
}
}
#endif
/* SparseRegistrationFromStereo.hpp */
/** @} */
