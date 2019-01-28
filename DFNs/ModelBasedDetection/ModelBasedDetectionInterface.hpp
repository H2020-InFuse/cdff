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
 * DFN that performs model-based detection and pose detection. The DFN detects an object of interest and computes the 3D pose of a camera with respect to the object model observed by the camera.
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
