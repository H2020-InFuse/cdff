/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef MIDRANGE3DMODELTRACKING_MIDRANGE3DMODELTRACKINGINTERFACE_HPP
#define MIDRANGE3DMODELTRACKING_MIDRANGE3DMODELTRACKINGINTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Types/C/Frame.h>
#include <Types/C/Pose.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * Tracking of the pose of a robot in a scene given its URDF description
     */
    class MidRange3DModelTrackingInterface : public DFPCCommonInterface
    {
        public:

            MidRange3DModelTrackingInterface();
            virtual ~MidRange3DModelTrackingInterface();

            /**
             * Send value to input port "image"
             * @param image
             *     2D image captured by a camera
             */
            virtual void imageInput(const asn1SccFrame& data);
            /**
             * Send value to input port "depth"
             * @param depth
             *     2D depth image
             */
            virtual void depthInput(const asn1SccFrame& data);
            /**
             * Send value to input port "robotName"
             * @param robotName
             *     Name of the robot being tracked
             */
            virtual void robotNameInput(const asn1SccT_String& data);

            /**
             * Query value from output port "pose"
             * @return pose
             *     Pose of the robot
             */
            virtual const asn1SccPose& poseOutput() const;


        protected:

            asn1SccFrame inImage;
            asn1SccFrame inDepth;
            asn1SccT_String inRobotName;
            asn1SccPose outPose;

    };
}
}

#endif // MIDRANGE3DMODELTRACKING_MIDRANGE3DMODELTRACKINGINTERFACE_HPP

/** @} */
