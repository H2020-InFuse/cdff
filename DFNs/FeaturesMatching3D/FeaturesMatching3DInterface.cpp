/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3DInterface.cpp
 * @date 17/01/2017
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FeaturesMatching3DInterface class
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
#include "FeaturesMatching3DInterface.hpp"


namespace dfn_ci {

using namespace PointCloudWrapper;
using namespace CorrespondenceMap3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesMatching3DInterface::FeaturesMatching3DInterface()
	{
	}

FeaturesMatching3DInterface::~FeaturesMatching3DInterface()
	{
	}

void FeaturesMatching3DInterface::sourceCloudInput(PointCloudConstPtr data) 
	{
    	inSourceCloud = data;
	}

void FeaturesMatching3DInterface::sinkCloudInput(PointCloudConstPtr data) 
	{
    	inSinkCloud = data;
	}

CorrespondenceMap3DConstPtr FeaturesMatching3DInterface::correspondenceMapOutput() 
	{
    	return outCorrespondenceMap;
	}




}

/** @} */

