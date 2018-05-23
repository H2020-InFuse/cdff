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

void KFPredictionInterface::previousStateInput(const RigidBodyState& data) {
    inpreviousstate = data;
}

void KFPredictionInterface::currentTimeInput(const Time& data) {
    incurrenttime = data;
}

RigidBodyState KFPredictionInterface::predictedStateOutput() {
    return outpredictedstate;
}

RigidBodyState KFPredictionInterface:: predictedStateCovarianceOutput() {
    return outpredictedStateCovariance;
}

}
