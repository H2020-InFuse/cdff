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
pcl::PointCloud<pcl::PointXYZ>::ConstPtr VisualPointFeatureVector3DToPclPointCloudConverter::ExtractPointCloud(const VisualPointFeatureVector3DConstPtr& featuresVector)
	{
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		pcl::PointXYZ newPoint;
		newPoint.x = GetXCoordinate(*featuresVector, pointIndex);
		newPoint.y = GetYCoordinate(*featuresVector, pointIndex);
		newPoint.z = GetZCoordinate(*featuresVector, pointIndex);
		pointCloud->points.push_back(newPoint);
		}

	return pointCloud;		
	}

void VisualPointFeatureVector3DToPclPointCloudConverter::ExtractFeaturesCloud(const VisualPointFeatureVector3DConstPtr& featuresVector, PointCloudWithFeatures<pcl::SHOT352>& conversion)
	{
	pcl::PointCloud<pcl::SHOT352>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<pcl::SHOT352> >();

	conversion.featureCloud = featureCloud;	
	conversion.descriptorSize = SHOT_DESCRIPTOR_LENGTH;

	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		ASSERT(conversion.descriptorSize == GetNumberOfDescriptorComponents(*featuresVector, pointIndex), 
			"VisualPointFeatureVector3DToPclPointCloudConverter: expected Shot descriptor with 352 components, descriptor with bad size found");
		pcl::SHOT352 newFeature;
		for(int componentIndex = 0; componentIndex < conversion.descriptorSize; componentIndex++)
			{
			newFeature.descriptor[componentIndex] = GetDescriptorComponent(*featuresVector, pointIndex, componentIndex);
			}
		featureCloud->points.push_back(newFeature);		
		}
	}

void VisualPointFeatureVector3DToPclPointCloudConverter::ExtractFeaturesCloud(const VisualPointFeatureVector3DConstPtr& featuresVector, PointCloudWithFeatures<pcl::PFHSignature125>& conversion)
	{
	pcl::PointCloud<pcl::PFHSignature125>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<pcl::PFHSignature125> >();

	conversion.featureCloud = featureCloud;	
	conversion.descriptorSize = PFH_DESCRIPTOR_LENGTH;

	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*featuresVector); pointIndex++)
		{
		ASSERT(conversion.descriptorSize == GetNumberOfDescriptorComponents(*featuresVector, pointIndex), 
			"VisualPointFeatureVector3DToPclPointCloudConverter: expected PFH descriptor with 125 components, descriptor with bad size found");
		pcl::PFHSignature125 newFeature;
		for(int componentIndex = 0; componentIndex < conversion.descriptorSize; componentIndex++)
			{
			newFeature.histogram[componentIndex] = GetDescriptorComponent(*featuresVector, pointIndex, componentIndex);
			}
		featureCloud->points.push_back(newFeature);		
		}
	}


}

/** @} */
