/**
 * @addtogroup DFPCs
 * @{
 */

#ifndef MODELBASEDVISUALTRACKING_MODELBASEDVISUALTRACKINGINTERFACE_HPP
#define MODELBASEDVISUALTRACKING_MODELBASEDVISUALTRACKINGINTERFACE_HPP

#include "DFPCCommonInterface.hpp"
#include <Types/C/RigidBodyState.h>
#include <Types/C/Time.h>
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFPC
{
    /**
     * DFPC that performs model-based tracking based on edge and contour
     * matching in stereo images
     */
    class ModelBasedVisualTrackingInterface : public DFPCCommonInterface
    {
        public:

            ModelBasedVisualTrackingInterface();
            virtual ~ModelBasedVisualTrackingInterface();

            /**
             * Send value to input port "imageLeft"
             * @param imageLeft: grayscale image from left camera
             */
            virtual void imageLeftInput(const asn1SccFrame& data);
            /**
             * Send value to input port "imageRight"
             * @param imageRight: grayscale image from right camera
             */
            virtual void imageRightInput(const asn1SccFrame& data);
            /**
             * Send value to input port "imageTime"
             * @param imageTime: image acquisition time (frame counter x frame rate)
             */
            virtual void imageTimeInput(const asn1SccTime& data);
            /**
             * Send value to input port "init"
             * @param init: initial velocity and pose
             */
            virtual void initInput(const asn1SccRigidBodyState& data);
            /**
             * Send value to input port "initTime"
             * @param initTime: initial time when tracker is initialized (synchronize pose initializer and tracker)
             */
            virtual void initTimeInput(const asn1SccTime& data);
            /**
             * Send value to input port "doInit"
             * @param doInit: whether to initialize the tracker pose and velocity, or not
             */
            virtual void doInitInput(bool data);
            /**
             * Send value to input port "egoMotion"
             * @param egoMotion: egomotion (position and orientation) of a manipulator with respect to its base
             */
            virtual void egoMotionInput(const asn1SccRigidBodyState& data);

            /**
             * Query value from output port "state"
             * @param state: pose and body velocity of the target
             */
            virtual const asn1SccRigidBodyState& stateOutput() const;
            /**
             * Query value from output port "success"
             * @param success: indicates whether the DFPC was successful in tracking the target
             */
            virtual bool successOutput() const;

        protected:

            asn1SccFrame inImageLeft;
            asn1SccFrame inImageRight;
            asn1SccTime inImageTime;
            asn1SccRigidBodyState inInit;
            asn1SccTime inInitTime;
            bool inDoInit;
            asn1SccRigidBodyState inEgoMotion;
            asn1SccRigidBodyState outState;
            bool outSuccess;

    };
}
}

#endif // MODELBASEDVISUALTRACKING_MODELBASEDVISUALTRACKINGINTERFACE_HPP

/** @} */
