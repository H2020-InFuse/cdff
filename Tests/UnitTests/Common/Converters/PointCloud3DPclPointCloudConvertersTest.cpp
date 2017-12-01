/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Point3DPclPointCloudConvertersTest.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from Point3D to PclPointCloud and viceversa.
 * 
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Definitions
 * Catch definition must be before the includes, otherwise catch will not compile.
 *
 * --------------------------------------------------------------------------
 */
#define CATCH_CONFIG_MAIN


/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <PointCloud3DToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloud3DConverter.hpp>
#include <PointCloud3D.h>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Converters;

TEST_CASE( "PclPointCloud to PointCloud3D and Back", "[PclPointCloudToPointCloud3D]" )
	{
	PclPointCloudToPointCloud3DConverter firstConverter;
	PointCloud3DToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
	for(int pointIndex = 0; pointIndex < 100; pointIndex++)
		{
		inputCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloud3D* asnPointCloud = firstConverter.Convert(inputCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr outputCloud = secondConverter.Convert(asnPointCloud);

	REQUIRE(outputCloud->points.size() == inputCloud->points.size() );
	for(int pointIndex = 0; pointIndex < 100; pointIndex++)
		{
		pcl::PointXYZ inputPoint = inputCloud->points.at(pointIndex);
		pcl::PointXYZ outputPoint = outputCloud->points.at(pointIndex);
		REQUIRE(inputPoint.x == outputPoint.x);	
		REQUIRE(inputPoint.y == outputPoint.y);			 
		REQUIRE(inputPoint.z == outputPoint.z);			
		}

	asn_sequence_empty( &(asnPointCloud->data.list) );
	delete(asnPointCloud);	
	} 

TEST_CASE( "PointCloud3D to PclPointCloud and Back", "[PointCloud3DToPclPointCloud]" )
	{
	PclPointCloudToPointCloud3DConverter firstConverter;
	PointCloud3DToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
	for(int pointIndex = 0; pointIndex < 100; pointIndex++)
		{
		inputCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloud3D* asnPointCloud = firstConverter.Convert(inputCloud);
	pcl::PointCloud<pcl::PointXYZ>::Ptr intermediateCloud = secondConverter.Convert(asnPointCloud);
	PointCloud3D* outputCloud = firstConverter.Convert(intermediateCloud);

	REQUIRE(outputCloud->size == asnPointCloud->size);
	REQUIRE(outputCloud->size == 300);
	for(int pointIndex = 0; pointIndex < 300; pointIndex++)
		{
		REQUIRE( *(asnPointCloud->data.list.array[pointIndex]) == *(outputCloud->data.list.array[pointIndex])  );
		}


	asn_sequence_empty( &(asnPointCloud->data.list) );
	delete(asnPointCloud);	
	asn_sequence_empty( &(outputCloud->data.list) );
	delete(outputCloud);	
	} 
