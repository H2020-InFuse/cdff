/**
 * @addtogroup DFNs
 * @{
 */

#include "KFPredictionInterface.hpp"

namespace CDFF
{
namespace DFN
{

KFPredictionInterface::KFPredictionInterface()
{
    asn1SccRigidBodyState_Initialize(& inPreviousState);
    asn1SccTime_Initialize(& inCurrentTime);
    asn1SccRigidBodyState_Initialize(& outPredictedState);
    asn1SccRigidBodyState_Initialize(& outPredictedStateCovariance);
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
}

/** @} */
