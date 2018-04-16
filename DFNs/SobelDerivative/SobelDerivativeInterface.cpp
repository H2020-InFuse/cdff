/*----------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SobelDerivativeInterface.hpp
 * @date 12/04/2018
 * @author Nassir W. Oumer 
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the SobelDerivativeInterface class
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
#include "SobelDerivativeInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
SobelDerivativeInterface::SobelDerivativeInterface()
	{
	
	}

SobelDerivativeInterface::~SobelDerivativeInterface()
	{

	}

void SobelDerivativeInterface::imageInput(FrameConstPtr data) 
	{
    	inImage = data;
	}

FrameConstPtr SobelDerivativeInterface::sobelGradxOutput() 
	{
    	return outSobelGradx;
	}

FrameConstPtr SobelDerivativeInterface::sobelGradyOutput() 
	{
    	return outSobelGrady;
	}


}

/** @} */

