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

bool DescriptorsAreEqual(VisualPointFeatureVector3DSharedConstPtr asnVector, unsigned pointIndex, MaxSizeHistogram descriptor)
	{
	REQUIRE( GetNumberOfDescriptorComponents(*asnVector, pointIndex) == MAX_HISTOGRAM_SIZE);
	bool descriptorsAreEqual = true;
	for(unsigned componentIndex = 0; componentIndex < MAX_HISTOGRAM_SIZE && descriptorsAreEqual; componentIndex++)
		{
		descriptorsAreEqual = ( GetDescriptorComponent(*asnVector, pointIndex, componentIndex) == descriptor.histogram[componentIndex] );
		}
	return descriptorsAreEqual;
	}

bool DescriptorsAreEqual(VisualPointFeatureVector3DSharedConstPtr asnVector, unsigned pointIndex, pcl::SHOT352 descriptor)
	{
	REQUIRE( GetNumberOfDescriptorComponents(*asnVector, pointIndex) == SHOT_DESCRIPTOR_LENGTH);
	bool descriptorsAreEqual = true;
	for(unsigned componentIndex = 0; componentIndex < SHOT_DESCRIPTOR_LENGTH && descriptorsAreEqual; componentIndex++)
		{
		descriptorsAreEqual = ( GetDescriptorComponent(*asnVector, pointIndex, componentIndex) == descriptor.descriptor[componentIndex] );
		}
	return descriptorsAreEqual;
	}

bool DescriptorsAreEqual(VisualPointFeatureVector3DSharedConstPtr asnVector, unsigned pointIndex, pcl::PFHSignature125 descriptor)
	{
	REQUIRE( GetNumberOfDescriptorComponents(*asnVector, pointIndex) == PFH_DESCRIPTOR_LENGTH);
	bool descriptorsAreEqual = true;
	for(unsigned componentIndex = 0; componentIndex < PFH_DESCRIPTOR_LENGTH && descriptorsAreEqual; componentIndex++)
		{
		descriptorsAreEqual = ( GetDescriptorComponent(*asnVector, pointIndex, componentIndex) == descriptor.histogram[componentIndex] );
		}
	return descriptorsAreEqual;
	}

bool DescriptorsAreEqual(MaxSizeHistogram descriptor1, MaxSizeHistogram descriptor2)
	{
	bool descriptorsAreEqual = true;
	for(unsigned componentIndex = 0; componentIndex < MAX_HISTOGRAM_SIZE && descriptorsAreEqual; componentIndex++)
		{
		descriptorsAreEqual = ( descriptor1.histogram[componentIndex] == descriptor2.histogram[componentIndex] );
		}
	return descriptorsAreEqual;
	}

bool DescriptorsAreEqual(pcl::SHOT352 descriptor1, pcl::SHOT352 descriptor2)
	{
	bool descriptorsAreEqual = true;
	for(unsigned componentIndex = 0; componentIndex < SHOT_DESCRIPTOR_LENGTH && descriptorsAreEqual; componentIndex++)
		{
		descriptorsAreEqual = ( descriptor1.descriptor[componentIndex] == descriptor2.descriptor[componentIndex] );
		}
	return descriptorsAreEqual;
	}

bool DescriptorsAreEqual(pcl::PFHSignature125 descriptor1, pcl::PFHSignature125 descriptor2)
	{
	bool descriptorsAreEqual = true;
	for(unsigned componentIndex = 0; componentIndex < PFH_DESCRIPTOR_LENGTH && descriptorsAreEqual; componentIndex++)
		{
		descriptorsAreEqual = ( descriptor1.histogram[componentIndex] == descriptor2.histogram[componentIndex] );
		}
	return descriptorsAreEqual;
	}

template <class FeatureType>
void ConversionAndBack(PointCloudWithFeatures<FeatureType> inputCloud)
	{
	PclPointCloudToVisualPointFeatureVector3DConverter firstConverter;
	VisualPointFeatureVector3DToPclPointCloudConverter secondConverter;

	REQUIRE( inputCloud.pointCloud->points.size() == inputCloud.featureCloud->points.size() );	

	VisualPointFeatureVector3DSharedConstPtr asnVector = firstConverter.ConvertShared(inputCloud);
	REQUIRE(GetNumberOfPoints(*asnVector) == inputCloud.pointCloud->points.size() );
	for(unsigned pointIndex = 0; pointIndex < static_cast<unsigned>( inputCloud.pointCloud->points.size() ); pointIndex++)
		{
		REQUIRE( GetXCoordinate(*asnVector, pointIndex) == inputCloud.pointCloud->points.at(pointIndex).x );
		REQUIRE( GetYCoordinate(*asnVector, pointIndex) == inputCloud.pointCloud->points.at(pointIndex).y );
		REQUIRE( GetZCoordinate(*asnVector, pointIndex) == inputCloud.pointCloud->points.at(pointIndex).z );
		
		REQUIRE( GetNumberOfDescriptorComponents(*asnVector, pointIndex) == inputCloud.descriptorSize );

		FeatureType inputFeature = inputCloud.featureCloud->points.at(pointIndex);
		REQUIRE( DescriptorsAreEqual(asnVector, pointIndex, inputFeature) );
		}

	PointCloudWithFeatures<FeatureType> outputCloud = secondConverter.ConvertShared<FeatureType>(asnVector);
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
		REQUIRE( DescriptorsAreEqual(inputFeature, outputFeature) );	
		}

	asnVector.reset();	
	}

PointCloudWithFeatures< MaxSizeHistogram > GetPointCloudWithHistogramFeatures()
	{
	typedef MaxSizeHistogram FeatureType;

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
		for(int componentIndex = 0; componentIndex < MAX_HISTOGRAM_SIZE; componentIndex++)
			{
			feature.histogram[componentIndex] = 0;
			}
		featureCloud->points.push_back( feature );
		}
	PointCloudWithFeatures<FeatureType> inputCloud;
	inputCloud.pointCloud = pointCloud;
	inputCloud.featureCloud = featureCloud;
	inputCloud.descriptorSize = MAX_HISTOGRAM_SIZE;

	return inputCloud;
	}

PointCloudWithFeatures< pcl::SHOT352 > GetPointCloudWithShotFeatures()
	{
	typedef pcl::SHOT352 FeatureType;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		pointCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}
	pcl::PointCloud<FeatureType>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<FeatureType> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		FeatureType feature;
		feature.descriptor[0] = 2*pointIndex;
		feature.descriptor[1] = pointIndex /2;
		for(int componentIndex = 2; componentIndex < SHOT_DESCRIPTOR_LENGTH; componentIndex++)
			{
			feature.descriptor[componentIndex] = 0;
			}
		featureCloud->points.push_back( feature );
		}
	PointCloudWithFeatures<FeatureType> inputCloud;
	inputCloud.pointCloud = pointCloud;
	inputCloud.featureCloud = featureCloud;
	inputCloud.descriptorSize = SHOT_DESCRIPTOR_LENGTH;

	return inputCloud;
	}

PointCloudWithFeatures< pcl::PFHSignature125 > GetPointCloudWithPfhFeatures()
	{
	typedef pcl::PFHSignature125 FeatureType;

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
		for(int componentIndex = 2; componentIndex < PFH_DESCRIPTOR_LENGTH; componentIndex++)
			{
			feature.histogram[componentIndex] = 0;
			}
		featureCloud->points.push_back( feature );
		}
	PointCloudWithFeatures<FeatureType> inputCloud;
	inputCloud.pointCloud = pointCloud;
	inputCloud.featureCloud = featureCloud;
	inputCloud.descriptorSize = PFH_DESCRIPTOR_LENGTH;

	return inputCloud;
	}

TEST_CASE( "PclPointCloud to VisualPointFeatureVector3D and Back", "[PclPointCloudToVisualPointFeatureVector3D]" )
	{
	PointCloudWithFeatures<MaxSizeHistogram> inputCloud = GetPointCloudWithHistogramFeatures();
	ConversionAndBack(inputCloud);
	} 

TEST_CASE( "SHOT PclPointCloud to VisualPointFeatureVector3D and Back", "[ShotPclPointCloudToVisualPointFeatureVector3D]" )
	{
	PointCloudWithFeatures<pcl::SHOT352> inputCloud = GetPointCloudWithShotFeatures();
	ConversionAndBack(inputCloud);
	} 

TEST_CASE( "Pfh PclPointCloud to VisualPointFeatureVector3D and Back", "[PfhPclPointCloudToVisualPointFeatureVector3D]" )
	{
	PointCloudWithFeatures<pcl::PFHSignature125> inputCloud = GetPointCloudWithPfhFeatures();
	ConversionAndBack(inputCloud);
	} 

template <class FeatureType>
void ConversionAndBackAndConversionAgain(PointCloudWithFeatures<FeatureType> inputCloud)
	{
	PclPointCloudToVisualPointFeatureVector3DConverter firstConverter;
	VisualPointFeatureVector3DToPclPointCloudConverter secondConverter;

	REQUIRE( inputCloud.pointCloud->points.size() == inputCloud.featureCloud->points.size() );

	VisualPointFeatureVector3DSharedConstPtr asnVector = firstConverter.ConvertShared(inputCloud);
	PointCloudWithFeatures<FeatureType> intermediateCloud = secondConverter.ConvertShared<FeatureType>(asnVector);
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

TEST_CASE( "VisualPointFeatureVector3D to PclPointCloud and Back", "[VisualPointFeatureVector3DToPclPointCloud]" )
	{
	PointCloudWithFeatures<MaxSizeHistogram> inputCloud = GetPointCloudWithHistogramFeatures();
	ConversionAndBackAndConversionAgain(inputCloud);
	}

TEST_CASE( "Shot VisualPointFeatureVector3D to PclPointCloud and Back", "[ShotVisualPointFeatureVector3DToPclPointCloud]" )
	{
	PointCloudWithFeatures<pcl::SHOT352> inputCloud = GetPointCloudWithShotFeatures();
	ConversionAndBackAndConversionAgain(inputCloud);
	} 

TEST_CASE( "Pfh VisualPointFeatureVector3D to PclPointCloud and Back", "[PfhVisualPointFeatureVector3DToPclPointCloud]" )
	{
	PointCloudWithFeatures<pcl::PFHSignature125> inputCloud = GetPointCloudWithPfhFeatures();
	ConversionAndBackAndConversionAgain(inputCloud);
	} 

TEST_CASE("Empty Point Cloud conversion", "[EmptyPointCloud]")
	{
	PclPointCloudToVisualPointFeatureVector3DConverter firstConverter;
	VisualPointFeatureVector3DToPclPointCloudConverter secondConverter;

	std::cout << "Init" << std::endl;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::SHOT352>::Ptr featureCloud = boost::make_shared<pcl::PointCloud<pcl::SHOT352> >();
	PointCloudWithFeatures<pcl::SHOT352> inputCloud;
	inputCloud.pointCloud = pointCloud;
	inputCloud.featureCloud = featureCloud;
	inputCloud.descriptorSize = 0;

	std::cout << "First" << std::endl;
	VisualPointFeatureVector3DSharedConstPtr asnVector = firstConverter.ConvertShared(inputCloud);
	std::cout << "Second" << std::endl;
	PointCloudWithFeatures<pcl::SHOT352> intermediateCloud = secondConverter.ConvertShared<pcl::SHOT352>(asnVector);
	std::cout << "Third" << std::endl;
	VisualPointFeatureVector3DSharedConstPtr outputCloud = firstConverter.ConvertShared(intermediateCloud);

	std::cout << "Check" << std::endl;
	REQUIRE(GetNumberOfPoints(*asnVector) == 0);
	REQUIRE(intermediateCloud.pointCloud->points.size() == 0);
	REQUIRE(intermediateCloud.featureCloud->points.size() == 0);
	REQUIRE(intermediateCloud.descriptorSize == SHOT_DESCRIPTOR_LENGTH);
	REQUIRE(GetNumberOfPoints(*outputCloud) == 0);

	std::cout << "reset" << std::endl;
	asnVector.reset();
	outputCloud.reset();

	std::cout << "end" << std::endl;
	}

TEST_CASE("Reference Features Conversion (Visual)", "[ReferenceFeaturesConversion]")
	{
	VisualPointFeatureVector3DToPclPointCloudConverter converter;

	VisualPointFeatureVector3DPtr featuresVector = NewVisualPointFeatureVector3D();
	AddPoint(*featuresVector, 10, 0);
	REQUIRE(GetPointType(*featuresVector, 0) == VISUAL_POINT_REFERENCE);
	REQUIRE(GetVectorType(*featuresVector) == ALL_REFERENCES_VECTOR );
	REQUIRE_THROWS( converter.Convert<pcl::SHOT352>(featuresVector) );

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
	REQUIRE_THROWS( converter.Convert<pcl::SHOT352>(featuresVector) );

	delete(featuresVector);				
	}
