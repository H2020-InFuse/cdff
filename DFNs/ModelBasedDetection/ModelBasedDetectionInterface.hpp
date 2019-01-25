/**
 * @addtogroup DFNs
 * @{
 */

#ifndef MODELBASEDDETECTION_MODELBASEDDETECTIONINTERFACE_HPP
#define MODELBASEDDETECTION_MODELBASEDDETECTIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFN
{

/**
 * DFN that performs model-based detection and pose detection. A training stage is required to learn some object characteristics, using the CAD model of the object. During the detection, the current object is matched to the training set, allowing to detect the object and to retrieve the corresponding trained pose.
 */
class ModelBasedDetectionInterface : public DFNCommonInterface
{
public:
        ModelBasedDetectionInterface();
        virtual ~ModelBasedDetectionInterface();

        /**
        * Send value to input port image
        * @param image, This is a colored image.
        */
        virtual void imageInput(asn1SccFrame& data);

        /**
        * Send value to input port depth
        * @param image, This is a depth image.
        */
        virtual void depthInput(asn1SccFrame& data);

        /**
         * Query value from output port "camera"
         * @return camera: the pose of the camera
         */
        virtual const asn1SccPose& cameraOutput() const;
        /**
         * Query value from output port "success"
         * @return success: boolean flag indicating successful Linemod
         *        detection. If false, the returned
         *        pose is meaningless.
         */
        virtual bool successOutput() const;

protected:
    asn1SccFrame inimage;
    asn1SccFrame indepth;
    asn1SccPose outCamera;
    bool outSuccess = false;
};

}
}

#endif // MODELBASEDDETECTION_MODELBASEDDETECTIONINTERFACE_HPP

/** @} */
