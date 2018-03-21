/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file Reconstruction3DInterface.hpp
 * @date 08/03/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs Chain that want to reconstruct a 3D scene from a pair of images.    
 *
 * @{
 */
#ifndef RECONSTRUCTION_3D_INTERFACE_HPP
#define RECONSTRUCTION_3D_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFPCCommonInterface.hpp>
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
    class Reconstruction3DInterface : public DFPCCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            Reconstruction3DInterface();
            virtual ~Reconstruction3DInterface();
            /**
            * Send value to input port leftImage
            * @param leftImage, a 2D left image taken from a stereo camera
            */
            virtual void leftImageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Send value to input port rightImage
            * @param rightImage, a 2D right image taken from a stereo camera
            */
            virtual void rightImageInput(FrameWrapper::FrameConstPtr data);

            /**
            * Receive value from output port pointCloud
            * @param pointCloud, This is the point cloud representing the 3D scene constructed so far.
            */
	    virtual PointCloudWrapper::PointCloudConstPtr pointCloudOutput();

            /**
            * Receive value from output port pose
            * @param pose, This is the pose of the camera in the scene.
            */
            virtual PoseWrapper::Pose3DConstPtr poseOutput();

            /**
            * Receive value from output port success
            * @param success, this determines whether the dfpc determine the camera transform.
            */
            virtual bool successOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            FrameWrapper::FrameConstPtr inLeftImage;
            FrameWrapper::FrameConstPtr inRightImage;
	    PointCloudWrapper::PointCloudConstPtr inModel;
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
/* Reconstruction3DInterface.hpp */
/** @} */
