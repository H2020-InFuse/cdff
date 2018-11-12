/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file VisualPointFeatureVector3DPclPointCloudConvertersTest.cpp
 * @date 19/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from VisualPointFeatureVector3D to PclPointCloud and viceversa.
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
#include <catch.hpp>
#include <Converters/VisualPointFeatureVector3DToPclPointCloudConverter.hpp>
#include <Converters/PclPointCloudToVisualPointFeatureVector3DConverter.hpp>
#include <Types/CPP/VisualPointFeatureVector3D.hpp>
#include <Errors/Assert.hpp>
#include <boost/smart_ptr.hpp>

using namespace Converters;
using namespace SupportTypes;
using namespace VisualPointFeatureVector3DWrapper;

TEST_CASE( "PclPointCloud to VisualPointFeatureVector3D and Back", "[PclPointCloudToVisualPointFeatureVector3D]" )
	{
	PclPointCloudToVisualPointFeatureVector3DConverter firstConverter;
	VisualPointFeatureVector3DToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		pointCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}
	pcl::PointCloud<FeatureType>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<FeatureType> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		FeatureType feature;
		feature.histogram[0] = 2*pointIndex;
		feature.histogram[1] = pointIndex /2;
		int maxFeaturesNumber = static_cast<int>(MAX_FEATURES_NUMBER);
		for(int componentIndex = 2; componentIndex < maxFeaturesNumber; componentIndex++)
			{
			feature.histogram[componentIndex] = 0;
			}
		featureCloud->points.push_back( feature );
		}
	PointCloudWithFeatures inputCloud;
	inputCloud.pointCloud = pointCloud;
	inputCloud.featureCloud = featureCloud;
	inputCloud.descriptorSize = 2;
	REQUIRE( inputCloud.pointCloud->points.size() == inputCloud.featureCloud->points.size() );	

	VisualPointFeatureVector3DSharedConstPtr asnVector = firstConverter.ConvertShared(inputCloud);
	REQUIRE(GetNumberOfPoints(*asnVector) == inputCloud.pointCloud->points.size() );
	for(int pointIndex = 0; pointIndex < static_cast<int>( inputCloud.pointCloud->points.size() ); pointIndex++)
		{
		REQUIRE( GetXCoordinate(*asnVector, pointIndex) == inputCloud.pointCloud->points.at(pointIndex).x );
		REQUIRE( GetYCoordinate(*asnVector, pointIndex) == inputCloud.pointCloud->points.at(pointIndex).y );
		REQUIRE( GetZCoordinate(*asnVector, pointIndex) == inputCloud.pointCloud->points.at(pointIndex).z );
		
		REQUIRE( GetNumberOfDescriptorComponents(*asnVector, pointIndex) == inputCloud.descriptorSize );
		for(unsigned componentIndex = 0; componentIndex < inputCloud.descriptorSize; componentIndex++)
			{
			REQUIRE( GetDescriptorComponent(*asnVector, pointIndex, componentIndex) == inputCloud.featureCloud->points.at(pointIndex).histogram[componentIndex]);
			}
		}

	PointCloudWithFeatures outputCloud = secondConverter.ConvertShared(asnVector);
	REQUIRE(outputCloud.pointCloud->points.size() == inputCloud.pointCloud->points.size() );
	REQUIRE(outputCloud.featureCloud->points.size() == inputCloud.featureCloud->points.size() );
	REQUIRE(outputCloud.descriptorSize == inputCloud.descriptorSize);
	for(unsigned pointIndex = 0; pointIndex < outputCloud.pointCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ inputPoint = inputCloud.pointCloud->points.at(pointIndex);
		pcl::PointXYZ outputPoint = outputCloud.pointCloud->points.at(pointIndex);
		REQUIRE(inputPoint.x == outputPoint.x);	
		REQUIRE(inputPoint.y == outputPoint.y);			 
		REQUIRE(inputPoint.z == outputPoint.z);	
		
		FeatureType inputFeature = inputCloud.featureCloud->points.at(pointIndex);
		FeatureType outputFeature = outputCloud.featureCloud->points.at(pointIndex);
		for(unsigned componentIndex = 0; componentIndex < outputCloud.descriptorSize; componentIndex++)
			{
			REQUIRE(inputFeature.histogram[componentIndex] == outputFeature.histogram[componentIndex]);
			}	
		int maxFeaturesNumber = static_cast<int>(MAX_FEATURES_NUMBER);
		for(int componentIndex = outputCloud.descriptorSize; componentIndex < maxFeaturesNumber; componentIndex++)
			{
			REQUIRE(outputFeature.histogram[componentIndex] == 0);
			}	
		}

	asnVector.reset();
	} 

TEST_CASE( "VisualPointFeatureVector3D to PclPointCloud and Back", "[VisualPointFeatureVector3DToPclPointCloud]" )
	{
	PclPointCloudToVisualPointFeatureVector3DConverter firstConverter;
	VisualPointFeatureVector3DToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		pointCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}
	pcl::PointCloud<FeatureType>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<FeatureType> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		FeatureType feature;
		feature.histogram[0] = 2*pointIndex;
		feature.histogram[1] = pointIndex /2;
		int maxFeaturesNumber = static_cast<int>(MAX_FEATURES_NUMBER);
		for(int componentIndex = 2; componentIndex < maxFeaturesNumber; componentIndex++)
			{
			feature.histogram[componentIndex] = 0;
			}
		featureCloud->points.push_back( feature );
		}
	PointCloudWithFeatures inputCloud;
	inputCloud.pointCloud = pointCloud;
	inputCloud.featureCloud = featureCloud;
	inputCloud.descriptorSize = 2;
	REQUIRE( inputCloud.pointCloud->points.size() == inputCloud.featureCloud->points.size() );

	VisualPointFeatureVector3DSharedConstPtr asnVector = firstConverter.ConvertShared(inputCloud);
	PointCloudWithFeatures intermediateCloud = secondConverter.ConvertShared(asnVector);
	VisualPointFeatureVector3DSharedConstPtr outputCloud = firstConverter.ConvertShared(intermediateCloud);

	REQUIRE(GetNumberOfPoints(*outputCloud) == GetNumberOfPoints(*asnVector));
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*asnVector); pointIndex++)
		{
		REQUIRE( GetXCoordinate(*outputCloud, pointIndex) == GetXCoordinate(*asnVector, pointIndex) );
		REQUIRE( GetYCoordinate(*outputCloud, pointIndex) == GetYCoordinate(*asnVector, pointIndex) );
		REQUIRE( GetZCoordinate(*outputCloud, pointIndex) == GetZCoordinate(*asnVector, pointIndex) );
		REQUIRE( GetNumberOfDescriptorComponents(*outputCloud, pointIndex) == GetNumberOfDescriptorComponents(*asnVector, pointIndex) );
		for(int componentIndex = 0; componentIndex < GetNumberOfDescriptorComponents(*outputCloud, pointIndex); componentIndex++)
			{
			REQUIRE( GetDescriptorComponent(*outputCloud, pointIndex, componentIndex) == GetDescriptorComponent(*asnVector, pointIndex, componentIndex) );
			}
		}


	asnVector.reset();
	outputCloud.reset();
	} 

TEST_CASE("Empty Point Cloud conversion", "[EmptyPointCloud]")
	{
	PclPointCloudToVisualPointFeatureVector3DConverter firstConverter;
	VisualPointFeatureVector3DToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<FeatureType>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<FeatureType> >();
	PointCloudWithFeatures inputCloud;
	inputCloud.pointCloud = pointCloud;
	inputCloud.featureCloud = featureCloud;
	inputCloud.descriptorSize = 0;

	VisualPointFeatureVector3DSharedConstPtr asnVector = firstConverter.ConvertShared(inputCloud);
	PointCloudWithFeatures intermediateCloud = secondConverter.ConvertShared(asnVector);
	VisualPointFeatureVector3DSharedConstPtr outputCloud = firstConverter.ConvertShared(intermediateCloud);

	REQUIRE(GetNumberOfPoints(*asnVector) == 0);
	REQUIRE(intermediateCloud.pointCloud->points.size() == 0);
	REQUIRE(intermediateCloud.featureCloud->points.size() == 0);
	REQUIRE(intermediateCloud.descriptorSize == 0);
	REQUIRE(GetNumberOfPoints(*outputCloud) == 0);

	asnVector.reset();
	outputCloud.reset();
	}

TEST_CASE("Reference Features Conversion (Visual)", "[ReferenceFeaturesConversion]")
	{
	VisualPointFeatureVector3DToPclPointCloudConverter converter;

	VisualPointFeatureVector3DPtr featuresVector = NewVisualPointFeatureVector3D();
	AddPoint(*featuresVector, 10, 0);
	REQUIRE(GetPointType(*featuresVector, 0) == VISUAL_POINT_REFERENCE);
	REQUIRE(GetVectorType(*featuresVector) == ALL_REFERENCES_VECTOR );
	REQUIRE_THROWS( converter.Convert(featuresVector) );

	delete(featuresVector);				
	}

TEST_CASE("Hybrid Features Conversion (Visual)", "[HybridFeaturesConversion]")
	{
	VisualPointFeatureVector3DToPclPointCloudConverter converter;

	VisualPointFeatureVector3DPtr featuresVector = NewVisualPointFeatureVector3D();
	AddPoint(*featuresVector, 10, 0);
	AddPoint(*featuresVector, 1.00, 2.10, 3.00);
	REQUIRE(GetPointType(*featuresVector, 0) == VISUAL_POINT_REFERENCE);
	REQUIRE(GetPointType(*featuresVector, 1) == VISUAL_POINT_POSITION);
	REQUIRE(GetVectorType(*featuresVector) == HYBRID_VECTOR);
	REQUIRE_THROWS( converter.Convert(featuresVector) );

	delete(featuresVector);				
	}
