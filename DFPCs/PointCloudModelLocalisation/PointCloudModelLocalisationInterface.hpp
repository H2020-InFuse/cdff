/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef POINTCLOUDMODELLOCALISATION_INTERFACE_HPP
#define POINTCLOUDMODELLOCALISATION_INTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Pose.h>
#include <Pointcloud.h>

namespace CDFF
{
namespace DFPC
{
    class PointCloudModelLocalisationInterface : public CDFF::DFPC::DFPCCommonInterface
    {
        public:

            PointCloudModelLocalisationInterface();
            virtual ~PointCloudModelLocalisationInterface();

            /**
             * Send value to input port "scene"
             * @param scene: a point cloud representing the environment
             */
            virtual void sceneInput(const asn1SccPointcloud& data);
            /**
             * Send value to input port "model"
             * @param model: the 3d point cloud of the model
             */
            virtual void modelInput(const asn1SccPointcloud& data);
            /**
             * Send value to input port "computeModelFeatures"
             * @param computeModelFeatures: this determines whether the model features will be computed. It has to be true the first time a new model is provided. For optimal performance it should be false on all subsequent calls.
             */
            virtual void computeModelFeaturesInput(bool data);

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

            asn1SccPointcloud inScene;
            asn1SccPointcloud inModel;
            bool inComputeModelFeatures;
            asn1SccPose outPose;
            bool outSuccess;

    };
}
}

#endif //  POINTCLOUDMODELLOCALISATION_INTERFACE_HPP

/** @} */
