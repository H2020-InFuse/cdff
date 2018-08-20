/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef RECONSTRUCTION3D_RECONSTRUCTION3DINTERFACE_HPP
#define RECONSTRUCTION3D_RECONSTRUCTION3DINTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Frame.h>
#include <Pose.h>
#include <Pointcloud.h>

namespace CDFF
{
namespace DFPC
{
    class Reconstruction3DInterface : public DFPCCommonInterface
    {
        public:

            Reconstruction3DInterface();
            virtual ~Reconstruction3DInterface();

            /**
             * Send value to input port "leftImage"
             * @param leftImage: a 2D left image taken from a stereo camera
             */
            virtual void leftImageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "rightImage"
             * @param rightImage: a 2D right image taken from a stereo camera
             */
            virtual void rightImageInput(const asn1SccFrame& data);

            /**
             * Query value from output port "pointCloud"
             * @return pointCloud: This is the point cloud representing the 3D scene constructed so far.
             */
            virtual const asn1SccPointcloud& pointCloudOutput() const;
            /**
             * Query value from output port "pose"
             * @return pose: This is the pose of the camera in the scene.
             */
            virtual const asn1SccPose& poseOutput() const;
            /**
             * Query value from output port "success"
             * @return success: this determines whether the dfpc determine the camera transform
             */
            virtual bool successOutput() const;

        protected:

            asn1SccFrame inLeftImage;
            asn1SccFrame inRightImage;
            asn1SccPointcloud outPointCloud;
            asn1SccPose outPose;
            bool outSuccess;
    };
}
}

#endif // RECONSTRUCTION3D_RECONSTRUCTION3DINTERFACE_HPP

/** @} */
