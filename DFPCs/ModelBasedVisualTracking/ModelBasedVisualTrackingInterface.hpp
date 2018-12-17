/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file ModelBasedVisualTrackingInterface.hpp
 * @date 23/05/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFPCs Chain that want to localise a 3d object from camera images.    
 *
 * @{
 */
#ifndef MODEL_BASED_VISUAL_TRACKING_INTERFACE_HPP
#define MODEL_BASED_VISUAL_TRACKING_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "DFPCCommonInterface.hpp"
#include <Types/C/RigidBodyState.h>
#include <Types/C/Time.h>
#include <Types/C/Eigen.h>
#include <Types/C/Frame.h>

namespace CDFF
{
namespace DFPC
{


/* --------------------------------------------------------------------------
 *
 * Class definition
 *
 * --------------------------------------------------------------------------
 */
    class ModelBasedVisualTrackingInterface : public DFPCCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */
        public:
            ModelBasedVisualTrackingInterface();
            virtual ~ModelBasedVisualTrackingInterface();
            /**
            * Send value to input port image
            * @param image, a gray image from left camera
            */
            virtual void imageLeftInput(const asn1SccFrame& data);
  	   /**
            * Send value to input port image
            * @param image, a gray image from right camera
            */
            virtual void imageRightInput(const asn1SccFrame& data);
	   /**
            * Send value to input port imageTime
            * @param imageTime, an image acquistion time, product of a frame counter and frame rate
            */
            virtual void imageTimeInput(const asn1SccTime& data);
	    /**
            * Send value to input port initTime
            * @param initTime, an initial time when tracker is initialized-synchronizes pose initializer and tracker
            */
	    virtual void initTimeInput(const asn1SccTime& data);
            /**
            * Send value to input port init
            * @param init, the initialization velocity and pose
            */
            virtual void initInput(const asn1SccRigidBodyState& data);
	    /**
            * Send value to input port doInit
            * @param doInit, whether to initialize the tracker pose & velcotiy or not
            */
            virtual void doInitInput(bool data);
	   /**
            * Send value to input port egoMotion
            * @param egoMotion, the egomotion (position and orientation) of a manipulator with respect to its base
            */
            virtual void egoMotionInput(const asn1SccRigidBodyState& data);

            /**
            * Receive value from output port pose
            * @param pose, the rigid body state pose, velocity of the target.
            */
            virtual asn1SccRigidBodyState stateOutput() const ;

            /**
            * Receive value from output port success
            * @param success, determines whether the dfpc could localise the target.
            */
            virtual bool successOutput() const;

	/* --------------------------------------------------------------------
	 * Protected
	 * --------------------------------------------------------------------
	 */
        protected:
             asn1SccFrame inImageLeft;
	     asn1SccFrame inImageRight;
	     asn1SccRigidBodyState inInit;
	     asn1SccTime inImageTime;
	     asn1SccTime inInitTime;
	     bool inDoInit;
	     asn1SccRigidBodyState inEgoMotion;
             asn1SccRigidBodyState outState;
	     bool outSuccess;

	/* --------------------------------------------------------------------
	 * Private
	 * --------------------------------------------------------------------
	 */
	private:
    };
}
}

#endif
/* ModelBasedVisualTrackingInterface.hpp */
/** @} */
