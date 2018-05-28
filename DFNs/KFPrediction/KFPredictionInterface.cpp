/**
 * @addtogroup DFNs
 * @{
 */

#include "KFPredictionInterface.hpp"

namespace dfn_ci
{

KFPredictionInterface::KFPredictionInterface()
{
}

KFPredictionInterface::~KFPredictionInterface()
{
}

void KFPredictionInterface::previousStateInput(const asn1SccRigidBodyState& data)
{
    inPreviousState = data;
}

void KFPredictionInterface::currentTimeInput(const asn1SccTime& data)
{
    inCurrentTime = data;
}

const asn1SccRigidBodyState& KFPredictionInterface::predictedStateOutput() const
{
    return outPredictedState;
}

const asn1SccRigidBodyState& KFPredictionInterface::predictedStateCovarianceOutput() const
{
    return outPredictedStateCovariance;
}

}

/** @} */
