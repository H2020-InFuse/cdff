/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudReconstruction2DTo3DInterface.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the PointCloudReconstruction2DTo3DInterface class
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
#include "PointCloudReconstruction2DTo3DInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace CorrespondenceMap2DWrapper;
using namespace PointCloudWrapper;
using namespace PoseWrapper;
using namespace MatrixWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
PointCloudReconstruction2DTo3DInterface::PointCloudReconstruction2DTo3DInterface()
	{
	
	}

PointCloudReconstruction2DTo3DInterface::~PointCloudReconstruction2DTo3DInterface()
	{

	}

void PointCloudReconstruction2DTo3DInterface::correspondenceMapInput(CorrespondenceMap2DConstPtr data) 
	{
    	inCorrespondenceMap = data;
	}

void PointCloudReconstruction2DTo3DInterface::fundamentalMatrixInput(Matrix3dConstPtr data)
	{
    	inFundamentalMatrix = data;
	}

void PointCloudReconstruction2DTo3DInterface::secondEpipoleInput(BaseTypesWrapper::Point2DConstPtr data)
	{
	inSecondEpipole = data;
	}

PointCloudConstPtr PointCloudReconstruction2DTo3DInterface::pointCloudOutput() 
	{
    	return outPointCloud;
	}


}

/** @} */

