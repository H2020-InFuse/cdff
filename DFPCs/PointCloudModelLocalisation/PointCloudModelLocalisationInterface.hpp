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
#include <DFPCCommonInterface.hpp>
#include <PointCloud.hpp>
#include <Pose.hpp>


namespace dfpc_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class PointCloudModelLocalisationInterface : public DFPCCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            PointCloudModelLocalisationInterface();
            virtual ~PointCloudModelLocalisationInterface();
            /**
            * Send value to input port pointCloud
            * @param pointCloud, a point cloud representing the environment
            */
            virtual void sceneInput(PointCloudWrapper::PointCloudConstPtr data);

            /**
            * Send value to input port model
            * @param model, the 3d point cloud of the model
            */
            virtual void modelInput(PointCloudWrapper::PointCloudConstPtr data);

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
            PointCloudWrapper::PointCloudConstPtr inScene;
	    PointCloudWrapper::PointCloudConstPtr inModel;
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
