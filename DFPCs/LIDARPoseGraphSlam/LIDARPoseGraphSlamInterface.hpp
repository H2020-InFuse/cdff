/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef LIDARPOSEGRAPHSLAM_INTERFACE_HPP
#define LIDARPOSEGRAPHSLAM_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Pointcloud.h>
#include <TransformWithCovariance.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * This processing compound simultaneously builds an environment model composed of a series of LIDAR point clouds and provides pose estimates for the rover.
     */
    class LIDARPoseGraphSlamInterface : public DFPCCommonInterface
    {
        public:

            LIDARPoseGraphSlamInterface();
            virtual ~LIDARPoseGraphSlamInterface();

            /**
             * Send value to input port "lidarPointCloud"
             * @param lidarPointCloud: Point cloud from LIDAR
             */
            virtual void lidarPointCloudInput(const asn1SccPointcloud& data);
            /**
             * Send value to input port "odometryPose"
             * @param odometryPose: Estimated pose from localization
             */
            virtual void odometryPoseInput(const asn1SccTransformWithCovariance& data);

            /**
             * Query value from output port "poseEstimate"
             * @return poseEstimate: Ground truth pose from GPS localization
             */
            virtual const asn1SccTransformWithCovariance& poseEstimateOutput() const;


        protected:

            asn1SccPointcloud inLidarPointCloud;
            asn1SccTransformWithCovariance inOdometryPose;
            asn1SccTransformWithCovariance outPoseEstimate;

    };
}
}

#endif //  LIDARPOSEGRAPHSLAM_INTERFACE_HPP

/** @} */
