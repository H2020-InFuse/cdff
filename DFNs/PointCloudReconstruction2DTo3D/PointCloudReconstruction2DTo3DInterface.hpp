/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file PointCloudReconstruction2DTo3DInterface.hpp
 * @date 29/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that construct a 3D point cloud from two images.    
 *
 * @{
 */
#ifndef POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_HPP
#define POINT_CLOUD_RECONSTRUCTION_2D_TO_3D_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>
#include <CorrespondenceMap2D.hpp>
#include <Pose.hpp>
#include <PointCloud.hpp>


namespace dfn_ci {


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class PointCloudReconstruction2DTo3DInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            PointCloudReconstruction2DTo3DInterface();
            virtual ~PointCloudReconstruction2DTo3DInterface();
            /**
            * Send value to input port correspondenceMap
            * @param correspondenceMap, this is a list coordinates of physical points in two distinct images taken from a camera. The set contains pairs of 2d coordinates ( (x1, y1), (x2, y2)), 
            * each pair represents the same physical point. (x1, y1) is its position in the first camera image and (x2, y2) is its position in the second camera image.
            */
            virtual void correspondenceMapInput(CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr data);

            /**
            * Send value to input port transform
            * @param transform, This is the transform between the two camera poses when the camera (or cameras) recorded the two images.
            */
            virtual void transformInput(PoseWrapper::Transform3DConstPtr data);

            /**
            * Receive value from output port pointCloud
            * @param pointCloud, This is the 3d point cloud representing the matching point in a 3D space. The system is aligned with the pose of the camera when the first image was taken.
            */
            virtual PointCloudWrapper::PointCloudConstPtr pointCloudOutput();

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
            CorrespondenceMap2DWrapper::CorrespondenceMap2DConstPtr inCorrespondenceMap;
	    PoseWrapper::Transform3DConstPtr inTransform;
            PointCloudWrapper::PointCloudConstPtr outPointCloud;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
#endif
/* PointCloudReconstruction2DTo3DInterface.hpp */
/** @} */
