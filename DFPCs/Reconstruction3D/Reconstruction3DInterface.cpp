/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Reconstruction3DInterface.cpp
 * @date 08/03/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the Reconstruction3DInterface class
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
#include "Reconstruction3DInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfpc_ci {

using namespace FrameWrapper;
using namespace PoseWrapper;
using namespace PointCloudWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
Reconstruction3DInterface::Reconstruction3DInterface()
	{
	
	}

Reconstruction3DInterface::~Reconstruction3DInterface()
	{

	}

void Reconstruction3DInterface::leftImageInput(FrameConstPtr data) 
	{
    	inLeftImage = data;
	}

void Reconstruction3DInterface::rightImageInput(FrameConstPtr data) 
	{
    	inRightImage = data;
	}

PointCloudConstPtr Reconstruction3DInterface::pointCloudOutput()
	{
	return outPointCloud;
	}

Pose3DConstPtr Reconstruction3DInterface::poseOutput() 
	{
    	return outPose;
	}

bool Reconstruction3DInterface::successOutput()
	{
	return outSuccess;
	}


}

/** @} */

