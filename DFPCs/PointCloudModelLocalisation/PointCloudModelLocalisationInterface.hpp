/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PointCloudModelLocalisationInterface.hpp
 * @date 23/02/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFPNs Chain that want to localise a 3d model in a 3d sceneray reconstructed by camera images.    
 *
 * @{
 */
#ifndef POINT_CLOUD_MODEL_LOCALISATION_INTERFACE_HPP
#define POINT_CLOUD_MODEL_LOCALISATION_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNsChainInterface.hpp>
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
    class PointCloudModelLocalisationInterface : public DFNsChainInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            PointCloudModelLocalisationInterface();
            virtual ~PointCloudModelLocalisationInterface();
            /**
            * Send value to input port image
            * @param image, a 2D image taken from a camera
            */
            virtual void imageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port pointCloud
            * @param pointCloud, This is the point cloud representing the 3D scene constructed so far.
            */
	    virtual PointCloudWrapper::PointCloudConstPtr pointCloudOutput();

            /**
            * Receive value from output port pose
            * @param pose, This is the pose of the model in the scene.
            */
            virtual PoseWrapper::Pose3DConstPtr poseOutput();

            /**
            * Receive value from output port success
            * @param success, this determines whether the dfpc could localise the model.
            */
            virtual bool successOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            FrameWrapper::FrameConstPtr inImage;
	    PointCloudWrapper::PointCloudConstPtr outPointCloud;
            PoseWrapper::Pose3DConstPtr outPose;
	    bool outSuccess;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* PointCloudModelLocalisationInterface.hpp */
/** @} */
