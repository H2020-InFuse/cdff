/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file StereoReconstructionInterface.cpp
 * @date 08/03/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the StereoReconstructionInterface class
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
#include "StereoReconstructionInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace PointCloudWrapper;
using namespace FrameWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
StereoReconstructionInterface::StereoReconstructionInterface()
	{
	
	}

StereoReconstructionInterface::~StereoReconstructionInterface()
	{

	}

void StereoReconstructionInterface::leftImageInput(FrameConstPtr data)
	{
    	inLeftImage = data;
	}

void StereoReconstructionInterface::rightImageInput(FrameConstPtr data)
	{
    	inRightImage = data;
	}

PointCloudConstPtr StereoReconstructionInterface::pointCloudOutput() 
	{
    	return outPointCloud;
	}


}

/** @} */

