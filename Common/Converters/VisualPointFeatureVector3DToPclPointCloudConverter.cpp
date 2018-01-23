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
 * @addtogroup Converters
 * 
 * Implementation of VisualPointFeatureVector3DToPclPointCloudConverter class.
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
const PointCloudWithFeatures VisualPointFeatureVector3DToPclPointCloudConverter::Convert(const VisualPointFeatureVector3DConstPtr& featuresVector)
	{	
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<FeatureType>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<FeatureType> >();
	PointCloudWithFeatures conversion;
	conversion.pointCloud = pointCloud;
	conversion.featureCloud = featureCloud;	
	conversion.descriptorSize = 0;
	if (GetNumberOfPoints(*featuresVector) == 0)
		return conversion;

	int descriptorSize = GetNumberOfDescriptorComponents(*featuresVector, 0);
	int maxFeaturesNumber = static_cast<int>(MAX_FEATURES_NUMBER);
	ASSERT(descriptorSize <= maxFeaturesNumber, "VisualPointFeatureVector3DToPclPointCloudConverter: Cannot convert features vector, number of component is larger than maximum allowed conversion");
	conversion.descriptorSize = descriptorSize;

	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		pcl::PointXYZ newPoint;
		newPoint.x = GetXCoordinate(*featuresVector, pointIndex);
		newPoint.y = GetYCoordinate(*featuresVector, pointIndex);
		newPoint.z = GetZCoordinate(*featuresVector, pointIndex);
		pointCloud->points.push_back(newPoint);

		ASSERT(descriptorSize == GetNumberOfDescriptorComponents(*featuresVector, pointIndex), "VisualPointFeatureVector3DToPclPointCloudConverter: Descriptors do not have the same size.");
		FeatureType newFeature;
		for(int componentIndex = 0; componentIndex < descriptorSize; componentIndex++)
			{
			newFeature.histogram[componentIndex] = GetDescriptorComponent(*featuresVector, pointIndex, componentIndex);
			}
		for(int componentIndex = descriptorSize; componentIndex < maxFeaturesNumber; componentIndex++)
			{
			newFeature.histogram[componentIndex] = 0;
			}
		featureCloud->points.push_back(newFeature);
		}

	return conversion;
	}

const PointCloudWithFeatures VisualPointFeatureVector3DToPclPointCloudConverter::ConvertShared(const VisualPointFeatureVector3DSharedConstPtr& featuresVector)
	{	
	return Convert(featuresVector.get());
	}


}

/** @} */
