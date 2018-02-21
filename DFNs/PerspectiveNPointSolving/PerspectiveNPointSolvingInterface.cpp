/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PerspectiveNPointSolvingInterface.cpp
 * @date 20/02/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the PerspectiveNPointSolvingInterface class
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
#include "PerspectiveNPointSolvingInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector2DWrapper;
using namespace PoseWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PerspectiveNPointSolvingInterface::PerspectiveNPointSolvingInterface()
	{
	
	}

PerspectiveNPointSolvingInterface::~PerspectiveNPointSolvingInterface()
	{

	}

void PerspectiveNPointSolvingInterface::pointCloudInput(PointCloudConstPtr data) 
	{
    	inPointCloud = data;
	}

void PerspectiveNPointSolvingInterface::cameraFeaturesVectorInput(VisualPointFeatureVector2DConstPtr data) 
	{
    	inCameraFeaturesVector = data;
	}

Pose3DConstPtr PerspectiveNPointSolvingInterface::poseOutput() 
	{
    	return outPose;
	}

bool PerspectiveNPointSolvingInterface::successOutput() 
	{
    	return outSuccess;
	}




}

/** @} */

