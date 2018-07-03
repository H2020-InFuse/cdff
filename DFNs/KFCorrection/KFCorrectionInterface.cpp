/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file KalmanPredictionInterface.hpp
 * @date 5/05/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 *
 * Implementation of the KalmanCorrectionInterface class
 *
 *
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */

#include "KFCorrectionInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

KFCorrectionInterface::KFCorrectionInterface()
{
}

KFCorrectionInterface::~KFCorrectionInterface()
{
}

void KFCorrectionInterface::predictedStateInput(const asn1SccRigidBodyState& data) {
    inPredictedState = data;
}

void KFCorrectionInterface::measurementInput(const asn1SccRigidBodyState& data) {
    inMeasurement = data;
}

void KFCorrectionInterface::predictedStateCovarianceInput(const asn1SccRigidBodyState& data)
{
    inPredictedStateCovariance = data;
}

asn1SccRigidBodyState KFCorrectionInterface::correctedStateOutput() {
    return outCorrectedState;
}

asn1SccRigidBodyState KFCorrectionInterface::stateCovarianceOutput() {
    return outStateCovariance;
}

}
