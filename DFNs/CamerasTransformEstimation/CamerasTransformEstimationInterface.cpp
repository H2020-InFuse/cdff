/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CamerasTransformEstimationInterface.cpp
 * @date 31/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the CamerasTransformEstimationInterface class
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
#include "CamerasTransformEstimationInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace CorrespondenceMap2DWrapper;
using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
CamerasTransformEstimationInterface::CamerasTransformEstimationInterface()
	{
	
	}

CamerasTransformEstimationInterface::~CamerasTransformEstimationInterface()
	{

	}

void CamerasTransformEstimationInterface::correspondenceMapInput(CorrespondenceMap2DConstPtr data) 
	{
    	inCorrespondenceMap = data;
	}

Transform3DConstPtr CamerasTransformEstimationInterface::transformOutput() 
	{
    	return outTransform;
	}

bool CamerasTransformEstimationInterface::successOutput()
	{
	return outSuccess;
	}




}

/** @} */

