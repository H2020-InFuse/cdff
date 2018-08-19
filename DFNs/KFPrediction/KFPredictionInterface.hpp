/**
 * @addtogroup DFNs
 * @{
 */

#ifndef KFPREDICTION_KFPREDICTIONINTERFACE_HPP
#define KFPREDICTION_KFPREDICTIONINTERFACE_HPP

#include "DFNCommonInterface.hpp"
#include <Time.h>
#include <RigidBodyState.h>

namespace CDFF
{
namespace DFN
{
    /**
     * DFN for Kalman filtering
     */
    class KFPredictionInterface : public DFNCommonInterface
    {
        public:

            KFPredictionInterface();
            virtual ~KFPredictionInterface();

            /**
             * Send value to input port "previousState"
             * @param previousState: initial or previous rigid body state
             */
            virtual void previousStateInput(const asn1SccRigidBodyState& data);
            /**
             * Send value to input port "currentTime"
             * @param currentTime: current timestamp
             */
            virtual void currentTimeInput(const asn1SccTime& data);

            /**
             * Query value from output port "predictedState"
             * @return predictedState: predicted rigid body state
             */
            virtual const asn1SccRigidBodyState& predictedStateOutput() const;
            /**
             * Query value from output port "predictedStateCovariance"
             * @return predictedStateCovariance: predicted state covariance
             */
            virtual const asn1SccRigidBodyState& predictedStateCovarianceOutput() const;

        protected:

            asn1SccRigidBodyState inPreviousState;
            asn1SccTime inCurrentTime;
            asn1SccRigidBodyState outPredictedState;
            asn1SccRigidBodyState outPredictedStateCovariance;
    };
}
}

#endif // KFPREDICTION_KFPREDICTIONINTERFACE_HPP

/** @} */
