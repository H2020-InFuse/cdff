/**
 * @addtogroup DFNs
 * @{
 */

#include "KFCorrectionInterface.hpp"

namespace dfn_ci
{

KFCorrectionInterface::KFCorrectionInterface()
{
}

KFCorrectionInterface::~KFCorrectionInterface()
{
}

void KFCorrectionInterface::predictedStateInput(const asn1SccRigidBodyState& data)
{
    inPredictedState = data;
}

void KFCorrectionInterface::measurementInput(const asn1SccRigidBodyState& data)
{
    inMeasurement = data;
}

void KFCorrectionInterface::predictedStateCovarianceInput(const asn1SccRigidBodyState& data)
{
    inPredictedStateCovariance = data;
}

const asn1SccRigidBodyState& KFCorrectionInterface::correctedStateOutput() const
{
    return outCorrectedState;
}

const asn1SccRigidBodyState& KFCorrectionInterface::stateCovarianceOutput() const
{
    return outStateCovariance;
}

}

/** @} */
