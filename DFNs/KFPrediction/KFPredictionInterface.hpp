/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file KalmanPredictionInterface.hpp
 * @date 24/04/2018
 * @author Nassir W. Oumer 
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that apply Kalman Filter.    
 *
 * @{
 */

#ifndef KFPREDICTION_INTERFACE_HPP
#define KFPREDICTION_INTERFACE_HPP

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <DFNCommonInterface.hpp>

#include "RigidBodyState.h"
#include "Time.h"

namespace dfn_ci {
    class KFPredictionInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */        
	public:
            KFPredictionInterface();
            virtual ~KFPredictionInterface();
            /**
            * Send value to input port previousState
            * @param previousState, initial or previous rigid body state
            */
            virtual void previousStateInput(const asn1SccRigidBodyState& data);
            /**
            * Send value to input port currentTime
            * @param currentTime, current time stamp
            */
            virtual void currentTimeInput(const asn1SccTime& data);

            /**
            * Receive value from output port predictedState
            * @return predictedState, predicted rigid body state
            */
            virtual asn1SccRigidBodyState predictedStateOutput();
  	   /**
            * Receive value from output port predictedStateCovariance
            * @return predictedState, predicted rigid body state covariance
            */
            virtual asn1SccRigidBodyState predictedStateCovarianceOutput();

        protected:
            asn1SccRigidBodyState inpreviousstate;
            asn1SccTime incurrenttime;
            asn1SccRigidBodyState outpredictedstate;
            asn1SccRigidBodyState outpredictedStateCovariance;
    };
}
#endif
