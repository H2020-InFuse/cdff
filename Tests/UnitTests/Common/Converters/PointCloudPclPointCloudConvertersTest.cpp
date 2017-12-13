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
#include <boost/smart_ptr.hpp>

using namespace Converters;
using namespace CppTypes;

TEST_CASE( "PclPointCloud to PointCloud and Back", "[PclPointCloudToPointCloud]" )
	{
	PclPointCloudToPointCloudConverter firstConverter;
	PointCloudToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		inputCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloud::ConstPtr asnPointCloud = firstConverter.Convert(inputCloud);
	REQUIRE(asnPointCloud->GetNumberOfPoints() == inputCloud->points.size() );
	for(int pointIndex = 0; pointIndex < static_cast<int>( inputCloud->points.size() ); pointIndex++)
		{
		REQUIRE( asnPointCloud->GetXCoordinate(pointIndex) == inputCloud->points.at(pointIndex).x );
		REQUIRE( asnPointCloud->GetYCoordinate(pointIndex) == inputCloud->points.at(pointIndex).y );
		REQUIRE( asnPointCloud->GetZCoordinate(pointIndex) == inputCloud->points.at(pointIndex).z );
		}

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

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
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

TEST_CASE("Empty Point Cloud conversion", "[EmptyPointCloud]")
	{
	PclPointCloudToPointCloudConverter firstConverter;
	PointCloudToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	PointCloud::ConstPtr asnPointCloud = firstConverter.Convert(inputCloud);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr intermediateCloud = secondConverter.Convert(asnPointCloud);
	PointCloud::ConstPtr outputCloud = firstConverter.Convert(intermediateCloud);

	REQUIRE(asnPointCloud->GetNumberOfPoints() == 0);
	REQUIRE(intermediateCloud->points.size() == 0);
	REQUIRE(outputCloud->GetNumberOfPoints() == 0);
	}
