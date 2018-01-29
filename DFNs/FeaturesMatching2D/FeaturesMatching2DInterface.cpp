/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching2DInterface.cpp
 * @date 29/01/2018
 * @author Alessandro Bianco (with code generation support)
 */

/*!
 * @addtogroup DFNs
 * 
 * Implementation of the FeaturesMatching2DInterface class
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
#include "FeaturesMatching2DInterface.hpp"
#include "Errors/Assert.hpp"

namespace dfn_ci {

using namespace CorrespondenceMap2DWrapper;
using namespace VisualPointFeatureVector2DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
FeaturesMatching2DInterface::FeaturesMatching2DInterface()
	{
	
	}

FeaturesMatching2DInterface::~FeaturesMatching2DInterface()
	{

	}

void FeaturesMatching2DInterface::sourceFeaturesVectorInput(VisualPointFeatureVector2DConstPtr data) 
	{
    	inSourceFeaturesVector = data;
	}

void FeaturesMatching2DInterface::sinkFeaturesVectorInput(VisualPointFeatureVector2DConstPtr data) 
	{
    	inSinkFeaturesVector = data;
	}

CorrespondenceMap2DConstPtr FeaturesMatching2DInterface::correspondenceMapOutput() 
	{
    	return outCorrespondenceMap;
	}




}

/** @} */

