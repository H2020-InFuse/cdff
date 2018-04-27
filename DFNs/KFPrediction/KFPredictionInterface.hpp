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
extern "C"{
#include "RigidBodyState.h"
#include "Time.h"
}

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
            virtual void previousStateInput(const RigidBodyState& data);
            /**
            * Send value to input port currentTime
            * @param currentTime, current time stamp
            */
            virtual void currentTimeInput(const Time& data);

            /**
            * Receive value from output port predictedState
            * @return predictedState, predicted rigid body state
            */
            virtual RigidBodyState predictedStateOutput();
  	   /**
            * Receive value from output port predictedStateCovariance
            * @return predictedState, predicted rigid body state covariance
            */
            virtual RigidBodyState predictedStateCovarianceOutput();

        protected:
            RigidBodyState inpreviousstate;
            Time incurrenttime;
            RigidBodyState outpredictedstate;
	    RigidBodyState outpredictedStateCovariance;
    };
}
#endif
