/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesDescription3DInterface.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FeaturesDescription3DInterface class
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
#include "FeaturesDescription3DInterface.hpp"


namespace dfn_ci {

using namespace PointCloudWrapper;
using namespace VisualPointFeatureVector3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesDescription3DInterface::FeaturesDescription3DInterface()
	{
	}

FeaturesDescription3DInterface::~FeaturesDescription3DInterface()
	{
	}

void FeaturesDescription3DInterface::pointCloudInput(PointCloudConstPtr data) 
	{
    	inPointCloud = data;
	}

void FeaturesDescription3DInterface::featuresSetInput(VisualPointFeatureVector3DConstPtr data) 
	{
    	inFeaturesSet = data;
	}

void FeaturesDescription3DInterface::normalsCloudInput(PointCloudWrapper::PointCloudConstPtr data)
	{
	inNormalsCloud = data;
	}

VisualPointFeatureVector3DConstPtr FeaturesDescription3DInterface::featuresSetWithDescriptorsOutput() 
	{
    	return outFeaturesSetWithDescriptors;
	}




}

/** @} */

