/**
 * @addtogroup DFNs
 * @{
 */

#ifndef KFCORRECTION_KFCORRECTIONINTERFACE_HPP
#define KFCORRECTION_KFCORRECTIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Types/C/RigidBodyState.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN for Kalman filtering
     */
    class KFCorrectionInterface : public DFNCommonInterface
    {
        public:

            KFCorrectionInterface();
            virtual ~KFCorrectionInterface();

            /**
             * Send value to input port "predictedState"
             * @param predictedState: predicted rigid body state (returned by the DFN KFPrediction, for instance)
             */
            virtual void predictedStateInput(const asn1SccRigidBodyState& data);
            /**
             * Send value to input port "measurement"
             * @param measurement: measured rigid body pose parameters
             */
            virtual void measurementInput(const asn1SccRigidBodyState& data);
            /**
             * Send value to input port "predictedStateCovariance"
             * @param predictedStateCovariance: predicted state covariance
             */
            virtual void predictedStateCovarianceInput(const asn1SccRigidBodyState& data);

            /**
             * Query value from output port "correctedState"
             * @return correctedState: corrected rigid body state
             */
            virtual const asn1SccRigidBodyState& correctedStateOutput() const;
            /**
             * Query value from output port "stateCovariance"
             * @return stateCovariance: corrected state covariance
             */
            virtual const asn1SccRigidBodyState& stateCovarianceOutput() const;

        protected:

            asn1SccRigidBodyState inPredictedState;
            asn1SccRigidBodyState inMeasurement;
            asn1SccRigidBodyState inPredictedStateCovariance;
            asn1SccRigidBodyState outCorrectedState;
            asn1SccRigidBodyState outStateCovariance;
    };
}
}

#endif // KFCORRECTION_KFCORRECTIONINTERFACE_HPP

/** @} */
