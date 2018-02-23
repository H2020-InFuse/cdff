/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudModelLocalisationInterface.cpp
 * @date 23/02/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the PointCloudModelLocalisationInterface class
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
#include "PointCloudModelLocalisationInterface.hpp"
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
PointCloudModelLocalisationInterface::PointCloudModelLocalisationInterface()
	{
	
	}

PointCloudModelLocalisationInterface::~PointCloudModelLocalisationInterface()
	{

	}

void PointCloudModelLocalisationInterface::imageInput(FrameConstPtr data) 
	{
    	inImage = data;
	}

PointCloudConstPtr PointCloudModelLocalisationInterface::pointCloudOutput()
	{
	return outPointCloud;
	}

Pose3DConstPtr PointCloudModelLocalisationInterface::poseOutput() 
	{
    	return outPose;
	}

bool PointCloudModelLocalisationInterface::successOutput()
	{
	return outSuccess;
	}


}

/** @} */

