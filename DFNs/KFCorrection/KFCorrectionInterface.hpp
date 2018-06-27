/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file KalmanCorrectionInterface.hpp
 * @date 05/05/2018
 * @author Nassir W. Oumer 
 */

/*!
 * @addtogroup DFNs
 * 
 *  This is the common interface of all DFNs that apply Kalman Filter.    
 *
 * @{
 */

#ifndef KFCORRECTION_INTERFACE_HPP
#define KFCORRECTION_INTERFACE_HPP

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
    class KFCorrectionInterface : public DFNCommonInterface
    {
	/* --------------------------------------------------------------------
	 * Public
	 * --------------------------------------------------------------------
	 */        
	public:
           KFCorrectionInterface();
            virtual ~KFCorrectionInterface();
            /**
            * Send value to input port predictedState
            * @param predictedState, predictedState rigid body state
            */
            virtual void predictedStateInput(const asn1SccRigidBodyState& data);
	    /**
            * Send value to input port  measurement
            * @param  measurement,  measurement rigid body pose parameters
            */
            virtual void  measurementInput(const asn1SccRigidBodyState& data);

	   /**
            * Send valueve value from input port predictedStateCovariance
            * @param predictedStateCovariance, state covariance
            */
            virtual void predictedStateCovarianceInput(const asn1SccRigidBodyState& data);

            /**
            * Receive value from output port correctedState
            * @return correctedState, correctedState
            */
            virtual asn1SccRigidBodyState correctedStateOutput();
  	   /**
            * Receive value from output port stateCovariance
            * @return stateCovariance, state covariance
            */
            virtual asn1SccRigidBodyState stateCovarianceOutput();

        protected:
            asn1SccRigidBodyState inPredictedState;
	    asn1SccRigidBodyState inmeasurement;
	    asn1SccRigidBodyState inPredictedStateCovariance;
            asn1SccRigidBodyState outCorrectedState;
	    asn1SccRigidBodyState outStateCovariance;
    };
}
#endif
