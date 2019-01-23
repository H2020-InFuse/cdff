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
#include <Errors/AssertOnTest.hpp>
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

void VisualPointFeatureVector3DToPclPointCloudConverter::ExtractFeaturesCloud(const VisualPointFeatureVector3DConstPtr& featuresVector, PointCloudWithFeatures<MaxSizeHistogram>& conversion)
	{
	pcl::PointCloud<MaxSizeHistogram >::Ptr featureCloud = boost::make_shared<pcl::PointCloud<MaxSizeHistogram > >();
	conversion.featureCloud = featureCloud;	

	unsigned numberOfPoints = GetNumberOfPoints(*featuresVector);
	conversion.descriptorSize = (numberOfPoints == 0 ? 0 : GetNumberOfDescriptorComponents(*featuresVector, 0) );
	ASSERT_ON_TEST( conversion.descriptorSize <= MAX_HISTOGRAM_SIZE, "VisualPointFeatureVector3DToPclPointCloudConverter error: histogram size is too large");

	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		ASSERT_ON_TEST(conversion.descriptorSize == GetNumberOfDescriptorComponents(*featuresVector, pointIndex), 
			"VisualPointFeatureVector3DToPclPointCloudConverter: histogram descriptor with bad size found");
		MaxSizeHistogram newFeature;
		for(int componentIndex = 0; componentIndex < conversion.descriptorSize; componentIndex++)
			{
			newFeature.histogram[componentIndex] = VisualPointFeatureVector3DWrapper::GetDescriptorComponent(*featuresVector, pointIndex, componentIndex);
			}
		for(int componentIndex = conversion.descriptorSize; componentIndex < MAX_HISTOGRAM_SIZE; componentIndex++)
			{
			newFeature.histogram[componentIndex] = 0;
			}
		featureCloud->points.push_back(newFeature);		
		}
	}

void VisualPointFeatureVector3DToPclPointCloudConverter::ExtractFeaturesCloud(const VisualPointFeatureVector3DConstPtr& featuresVector, PointCloudWithFeatures<pcl::SHOT352>& conversion)
	{
	pcl::PointCloud<pcl::SHOT352>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<pcl::SHOT352> >();

	conversion.featureCloud = featureCloud;	
	conversion.descriptorSize = SHOT_DESCRIPTOR_LENGTH;
	unsigned numberOfPoints = GetNumberOfPoints(*featuresVector);

	ASSERT_ON_TEST( numberOfPoints == 0 || GetFeatureType(*featuresVector) == SHOT_DESCRIPTOR, 
		"VisualPointFeatureVector3DToPclPointCloudConverter, Shot descriptor required by converter, but vector does not contain SHOT" );
	for(int pointIndex = 0; pointIndex <numberOfPoints; pointIndex++)
		{
		ASSERT_ON_TEST(conversion.descriptorSize == GetNumberOfDescriptorComponents(*featuresVector, pointIndex), 
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
	unsigned numberOfPoints = GetNumberOfPoints(*featuresVector);

	ASSERT_ON_TEST( numberOfPoints == 0 || GetFeatureType(*featuresVector) == PFH_DESCRIPTOR, 
		"VisualPointFeatureVector3DToPclPointCloudConverter, PFH descriptor required by converter, but vector does not contain PFH" );
	for(int pointIndex = 0; pointIndex < numberOfPoints; pointIndex++)
		{
		ASSERT_ON_TEST(conversion.descriptorSize == GetNumberOfDescriptorComponents(*featuresVector, pointIndex), 
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
