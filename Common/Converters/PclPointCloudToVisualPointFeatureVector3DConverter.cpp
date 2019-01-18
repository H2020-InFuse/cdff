/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PclPointCloudToVisualPointFeatureVector3DConverter.cpp
 * @date 19/01/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup Converters
 * 
 * Implementation of PclPointCloudToVisualPointFeatureVector3DConverter class.
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

#include "PclPointCloudToVisualPointFeatureVector3DConverter.hpp"
#include <Errors/Assert.hpp>
#include <boost/make_shared.hpp>


namespace Converters {

using namespace VisualPointFeatureVector3DWrapper;
using namespace SupportTypes;

/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
const VisualPointFeatureVector3DConstPtr PclPointCloudToVisualPointFeatureVector3DConverter::Convert(const PointCloudWithFeatures<pcl::SHOT352>& featuresCloud)
	{	
	VisualPointFeatureVector3DPtr conversion = NewVisualPointFeatureVector3D();
	
	for(unsigned pointIndex = 0; pointIndex < featuresCloud.pointCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = featuresCloud.pointCloud->points.at(pointIndex);
		pcl::SHOT352 feature = featuresCloud.featureCloud->points.at(pointIndex);
		AddPoint(*conversion, point.x, point.y, point.z);
		for(unsigned componentIndex = 0; componentIndex < featuresCloud.descriptorSize; componentIndex++)
			{
			AddDescriptorComponent(*conversion, pointIndex, feature.descriptor[componentIndex]);
			}
		}

	return conversion;
	}

const VisualPointFeatureVector3DConstPtr PclPointCloudToVisualPointFeatureVector3DConverter::Convert(const PointCloudWithFeatures<pcl::PFHSignature125>& featuresCloud)
	{	
	VisualPointFeatureVector3DPtr conversion = NewVisualPointFeatureVector3D();
	
	for(unsigned pointIndex = 0; pointIndex < featuresCloud.pointCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ point = featuresCloud.pointCloud->points.at(pointIndex);
		pcl::PFHSignature125 feature = featuresCloud.featureCloud->points.at(pointIndex);
		AddPoint(*conversion, point.x, point.y, point.z);
		for(unsigned componentIndex = 0; componentIndex < featuresCloud.descriptorSize; componentIndex++)
			{
			AddDescriptorComponent(*conversion, pointIndex, feature.histogram[componentIndex]);
			}
		}

	return conversion;
	}


}

/** @} */
