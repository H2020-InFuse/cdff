/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file KalmanPredictionInterface.hpp
 * @date 24/04/2018
 * @author Nassir W. Oumer 
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the KalmanPredictionInterface class
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

#include "KFPredictionInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

KFPredictionInterface::KFPredictionInterface()
{
}

KFPredictionInterface::~KFPredictionInterface()
{
}

void KFPredictionInterface::previousStateInput(const asn1SccRigidBodyState& data) {
    inpreviousstate = data;
}

void KFPredictionInterface::currentTimeInput(const asn1SccTime& data) {
    incurrenttime = data;
}

asn1SccRigidBodyState KFPredictionInterface::predictedStateOutput() {
    return outpredictedstate;
}

asn1SccRigidBodyState KFPredictionInterface:: predictedStateCovarianceOutput() {
    return outpredictedStateCovariance;
}

}
