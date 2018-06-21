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
 * @addtogroup DFNs
 * 
 *  This DFN chain implements the Registration From Stereo as implementation of the DPFC for Reconstruction3D.
 *  This chain operates as follows: 
 *  the left and right images are used to reconstruct a 3D point cloud throught computation of a disparity map
 *  camera movement is estimated by registration of the newly detected point cloud on the point cloud map reconstructed so far.
 *  point clouds at different time instants are merged together taking into account the movement of the camera.
 * 
 * @{
 */

#ifndef DENSE_REGISTRATION_FROM_STEREO_HPP
#define DENSE_REGISTRATION_FROM_STEREO_HPP

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

#include "PointCloudMap.hpp"
#include <Helpers/ParametersListHelper.hpp>
#include <DfpcConfigurator.hpp>
#include <Frame.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>


namespace dfpc_ci {

/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
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

		struct DenseRegistrationFromStereoOptionsSet
			{
			float searchRadius;
			float pointCloudMapResolution;
			};

		Helpers::ParametersListHelper parametersHelper;
		DenseRegistrationFromStereoOptionsSet parameters;
		static const DenseRegistrationFromStereoOptionsSet DEFAULT_PARAMETERS;

		dfn_ci::ImageFilteringInterface* optionalLeftFilter;
		dfn_ci::ImageFilteringInterface* optionalRightFilter;
		dfn_ci::StereoReconstructionInterface* reconstructor3D;
		dfn_ci::Registration3DInterface* cloudRegistrator;

		FrameWrapper::FramePtr leftImage;
		FrameWrapper::FramePtr rightImage;
		FrameWrapper::FramePtr filteredLeftImage;
		FrameWrapper::FramePtr filteredRightImage;
		PointCloudWrapper::PointCloudPtr imagesCloud;
		PointCloudWrapper::PointCloudConstPtr sceneCloud;
		PoseWrapper::Pose3DPtr cameraPoseInScene;
		PoseWrapper::Pose3DPtr previousCameraPoseInScene;
		VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr emptyFeaturesVector;

		void ConfigureExtraParameters();
		void AssignDfnsAlias();

		void ComputePointCloud();

		void FilterLeftImage();
		void FilterRightImage();
		void ComputeStereoPointCloud();
		bool RegisterPointCloudOnScene();
    };
}
#endif
/* DenseRegistrationFromStereo.hpp */
/** @} */
