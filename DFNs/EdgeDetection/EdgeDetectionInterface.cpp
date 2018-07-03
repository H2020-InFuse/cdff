/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file EdgeDetectionInterface.hpp
 * @date 11/04/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 *
 * Implementation of the EdgeDetectionInterface class
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
#include "EdgeDetectionInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
EdgeDetectionInterface::EdgeDetectionInterface()
	{

	}

EdgeDetectionInterface::~EdgeDetectionInterface()
	{

	}

void EdgeDetectionInterface::imageInput(FrameConstPtr data)
	{
    	inImage = data;
	}

FrameConstPtr EdgeDetectionInterface::edgeMapOutput()
	{
    	return outEdgeMap;
	}
FrameConstPtr EdgeDetectionInterface::sobelGradientXOutput()
	{
    	return outSobelGradientX;
	}

FrameConstPtr EdgeDetectionInterface::sobelGradientYOutput()
	{
    	return outSobelGradientY;
	}




}

/** @} */
