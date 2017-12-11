/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudPclPointCloudConvertersTest.cpp
 * @date 01/12/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from PointCloud to PclPointCloud and viceversa.
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
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <PointCloud.hpp>
#include <Catch/catch.h>
#include <Errors/Assert.hpp>

using namespace Converters;
using namespace CppTypes;

TEST_CASE( "PclPointCloud to PointCloud and Back", "[PclPointCloudToPointCloud]" )
	{
	PclPointCloudToPointCloudConverter firstConverter;
	PointCloudToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		inputCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloud::ConstPtr asnPointCloud = firstConverter.Convert(inputCloud);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr outputCloud = secondConverter.Convert(asnPointCloud);

	REQUIRE(outputCloud->points.size() == inputCloud->points.size() );
	for(unsigned pointIndex = 0; pointIndex < outputCloud->points.size(); pointIndex++)
		{
		pcl::PointXYZ inputPoint = inputCloud->points.at(pointIndex);
		pcl::PointXYZ outputPoint = outputCloud->points.at(pointIndex);
		REQUIRE(inputPoint.x == outputPoint.x);	
		REQUIRE(inputPoint.y == outputPoint.y);			 
		REQUIRE(inputPoint.z == outputPoint.z);			
		}

	asnPointCloud.reset();
	} 

TEST_CASE( "PointCloud3D to PclPointCloud and Back", "[PointCloud3DToPclPointCloud]" )
	{
	PclPointCloudToPointCloudConverter firstConverter;
	PointCloudToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = pcl::PointCloud<pcl::PointXYZ>::Ptr( new pcl::PointCloud<pcl::PointXYZ>() );
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		inputCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloud::ConstPtr asnPointCloud = firstConverter.Convert(inputCloud);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr intermediateCloud = secondConverter.Convert(asnPointCloud);
	PointCloud::ConstPtr outputCloud = firstConverter.Convert(intermediateCloud);

	REQUIRE(outputCloud->GetNumberOfPoints() == asnPointCloud->GetNumberOfPoints());
	for(int pointIndex = 0; pointIndex < outputCloud->GetNumberOfPoints(); pointIndex++)
		{
		REQUIRE( outputCloud->GetXCoordinate(pointIndex) == asnPointCloud->GetXCoordinate(pointIndex) );
		REQUIRE( outputCloud->GetYCoordinate(pointIndex) == asnPointCloud->GetYCoordinate(pointIndex) );
		REQUIRE( outputCloud->GetZCoordinate(pointIndex) == asnPointCloud->GetZCoordinate(pointIndex) );
		}


	asnPointCloud.reset();
	outputCloud.reset();
	} 
