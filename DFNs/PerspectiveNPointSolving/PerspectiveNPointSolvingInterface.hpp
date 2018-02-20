/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PerspectiveNPointSolvingInterface.hpp
 * @date 20/02/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that solve the Perspective N Point problem.    
 *
 * @{
 */
#ifndef PERSPECTIVE_N_POINT_SOLVING_INTERFACE_HPP
#define PERSPECTIVE_N_POINT_SOLVING_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <PointCloud.hpp>
#include <VisualPointFeatureVector2D.hpp>
#include <Pose.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class PerspectiveNPointSolvingInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            PerspectiveNPointSolvingInterface();
            virtual ~PerspectiveNPointSolvingInterface();
            /**
            * Send value to input port pointCloud
            * @param pointCloud, this is the set of 3d points representing the scene in some reference coordinate system.
            */
            virtual void pointCloudInput(PointCloudWrapper::PointCloudConstPtr data);

            /**
            * Send value to input port cameraFeaturesVector
            * @param cameraFeaturesVector, these are the 2d image coordinates of the 3d points as they are observed by the camera.
            */
            virtual void cameraFeaturesVectorInput(VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr data);

            /**
            * Receive value from output port pose
            * @param pose, This is the pose of the camera in the reference coordinate system of the point cloud.
            */
            virtual PoseWrapper::Pose3DConstPtr poseOutput();

            /**
            * Receive value from output port success
            * @param success, This boolean value determines whether a valid pose could be found. If this value is false, pose is meaningless.
            */
            virtual bool successOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            PointCloudWrapper::PointCloudConstPtr inPointCloud;
            VisualPointFeatureVector2DWrapper::VisualPointFeatureVector2DConstPtr inCameraFeaturesVector;
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
/* PerspectiveNPointSolvingInterface.hpp */
/** @} */
