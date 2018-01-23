/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3DToPclPointCloudConverter.cpp
 * @date 19/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Mocks
 * 
 * Implementation of VisualPointFeatureVector3DToPclPointCloudConverter.
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

#include "VisualPointFeatureVector3DToPclPointCloudConverter.hpp"
#include <Errors/Assert.hpp>
#include <Mocks/MockMacro.hpp>

namespace Mocks {

using namespace VisualPointFeatureVector3DWrapper;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */

VisualPointFeatureVector3DToPclPointCloudConverter::~VisualPointFeatureVector3DToPclPointCloudConverter()
	{

	}

const Converters::SupportTypes::PointCloudWithFeatures VisualPointFeatureVector3DToPclPointCloudConverter::Convert(const VisualPointFeatureVector3DWrapper::VisualPointFeatureVector3DConstPtr& featuresVector)
	MOCK_METHOD(Converters::VisualPointFeatureVector3DToPclPointCloudConverter, Convert, Converters::SupportTypes::PointCloudWithFeatures, (featuresVector) )

}
/** @} */
