/**
 * @addtogroup DFNs
 * @{
 */

#ifndef POINTCLOUDRECONSTRUCTION2DTO3D_POINTCLOUDRECONSTRUCTION2DTO3DINTERFACE_HPP
#define POINTCLOUDRECONSTRUCTION2DTO3D_POINTCLOUDRECONSTRUCTION2DTO3DINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/CorrespondenceMap2D.h>
#include <Types/C/Pose.h>
#include <Types/C/Pointcloud.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN that turns pairs of 2D matching keypoints into a reconstructed
     * 3D pointcloud of keypoints
     */
    class PointCloudReconstruction2DTo3DInterface : public DFNCommonInterface
    {
        public:

            PointCloudReconstruction2DTo3DInterface();
            virtual ~PointCloudReconstruction2DTo3DInterface();

            /**
             * Send value to input port "matches"
             * @param matches: keypoint matches between two images A and B.
             *        This is a list of pairs of 2D coordinates: ((x^A_1,
             *        y^A_1), (x^B_1, y^B_1)), ..., ((x^A_n, y^A_n), (x^B_n,
             *        y^B_n)). Each pair contains the coordinates of the same
             *        keypoint (hopefully, the same physical point) in each
             *        image.
             */
            virtual void matchesInput(const asn1SccCorrespondenceMap2D& data);
            /**
             * Send value to input port "frame"
             * @param frame: pose of the coordinate frame of the camera that
             *        captured image B relative to the coordinate frame of the
             *        camera that captured image A
             */
            virtual void poseInput(const asn1SccPose& data);

            /**
             * Query value from output port "pointcloud"
             * @return pointcloud: reconstructed 3D pointcloud of keypoints, in
             *        the coordinate frame of the camera that captured image A
             */
            virtual const asn1SccPointcloud& pointcloudOutput() const;

        protected:

            asn1SccCorrespondenceMap2D inMatches;
            asn1SccPose inPose;
            asn1SccPointcloud outPointcloud;
    };
}
}

#endif // POINTCLOUDRECONSTRUCTION2DTO3D_POINTCLOUDRECONSTRUCTION2DTO3DINTERFACE_HPP

/** @} */
