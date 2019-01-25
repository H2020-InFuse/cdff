/**
 * @addtogroup DFNs
 * @{
 */

#include "KFCorrectionInterface.hpp"

namespace CDFF
{
namespace DFN
{

KFCorrectionInterface::KFCorrectionInterface()
{
    asn1SccRigidBodyState_Initialize(& inPredictedState);
    asn1SccRigidBodyState_Initialize(& inMeasurement);
    asn1SccRigidBodyState_Initialize(& inPredictedStateCovariance);
    asn1SccRigidBodyState_Initialize(& outCorrectedState);
    asn1SccRigidBodyState_Initialize(& outStateCovariance);
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
}

/** @} */
