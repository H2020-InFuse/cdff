/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef RECONSTRUCTIONANDIDENTIFICATION_RECONSTRUCTIONANDIDENTIFICATIONINTERFACE_HPP
#define RECONSTRUCTIONANDIDENTIFICATION_RECONSTRUCTIONANDIDENTIFICATIONINTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Types/C/Pose.h>
#include <Types/C/Pointcloud.h>
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFPC
{
    class ReconstructionAndIdentificationInterface : public DFPCCommonInterface
    {
        public:

            ReconstructionAndIdentificationInterface();
            virtual ~ReconstructionAndIdentificationInterface();

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
             * Send value to input port "scene"
             * @param scene: a point cloud representing the environment
             */
            virtual void modelInput(const asn1SccPointcloud& data);
            /**
             * Send value to input port "computeModelFeatures"
             * @param computeModelFeatures: this determines whether the model features will be computed. It has to be true the first time a new model is provided. For optimal performance it should be false on all subsequent calls.
             */
            virtual void computeModelFeaturesInput(bool data);

            /**
             * Query value from output port "pointCloud"
             * @return pointCloud: This is the point cloud representing the 3D scene constructed so far.
             */
            virtual const asn1SccPointcloud& pointCloudOutput() const;

            /**
             * Query value from output port "pose"
             * @return pose: This is the pose of the model in the scene
             */
            virtual const asn1SccPose& poseOutput() const;
            /**
             * Query value from output port "success"
             * @return success: this determines whether the dfpc could localise the model
             */
            virtual bool successOutput() const;


        protected:

            asn1SccFrame inLeftImage;
            asn1SccFrame inRightImage;
            asn1SccPointcloud inModel;
            bool inComputeModelFeatures = false;
            asn1SccPointcloud outPointCloud;
            asn1SccPose outPose;
            bool outSuccess = false;

    };
}
}

#endif // RECONSTRUCTIONANDIDENTIFICATION_RECONSTRUCTIONANDIDENTIFICATIONINTERFACE_HPP

/** @} */
