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
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <catch.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include <PclPointCloudToPointCloudConverter.hpp>
#include <PointCloud.hpp>
#include <Errors/Assert.hpp>
#include <boost/smart_ptr.hpp>

using namespace Converters;
using namespace PointCloudWrapper;

TEST_CASE( "PclPointCloud to PointCloud and Back", "[PclPointCloudToPointCloud]" )
	{
	PclPointCloudToPointCloudConverter firstConverter;
	PointCloudToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		inputCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloudSharedConstPtr asnPointCloud = firstConverter.ConvertShared(inputCloud);
	REQUIRE(GetNumberOfPoints(*asnPointCloud) == inputCloud->points.size() );
	for(int pointIndex = 0; pointIndex < static_cast<int>( inputCloud->points.size() ); pointIndex++)
		{
		REQUIRE( GetXCoordinate(*asnPointCloud, pointIndex) == inputCloud->points.at(pointIndex).x );
		REQUIRE( GetYCoordinate(*asnPointCloud, pointIndex) == inputCloud->points.at(pointIndex).y );
		REQUIRE( GetZCoordinate(*asnPointCloud, pointIndex) == inputCloud->points.at(pointIndex).z );
		}

	pcl::PointCloud<pcl::PointXYZ>::ConstPtr outputCloud = secondConverter.ConvertShared(asnPointCloud);
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

TEST_CASE( "PointCloud3D to PclPointCloud and Back (Pcl)", "[PointCloud3DToPclPointCloud]" )
	{
	PclPointCloudToPointCloudConverter firstConverter;
	PointCloudToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		inputCloud->points.push_back( pcl::PointXYZ(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloudSharedConstPtr asnPointCloud = firstConverter.ConvertShared(inputCloud);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr intermediateCloud = secondConverter.ConvertShared(asnPointCloud);
	PointCloudSharedConstPtr outputCloud = firstConverter.ConvertShared(intermediateCloud);

	REQUIRE(GetNumberOfPoints(*outputCloud) == GetNumberOfPoints(*asnPointCloud));
	for(int pointIndex = 0; pointIndex < GetNumberOfPoints(*outputCloud); pointIndex++)
		{
		REQUIRE( GetXCoordinate(*outputCloud, pointIndex) == GetXCoordinate(*asnPointCloud, pointIndex) );
		REQUIRE( GetYCoordinate(*outputCloud, pointIndex) == GetYCoordinate(*asnPointCloud, pointIndex) );
		REQUIRE( GetZCoordinate(*outputCloud, pointIndex) == GetZCoordinate(*asnPointCloud, pointIndex) );
		}


	asnPointCloud.reset();
	outputCloud.reset();
	} 

TEST_CASE("Empty Point Cloud conversion (Pcl)", "[EmptyPointCloud]")
	{
	PclPointCloudToPointCloudConverter firstConverter;
	PointCloudToPclPointCloudConverter secondConverter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	PointCloudSharedConstPtr asnPointCloud = firstConverter.ConvertShared(inputCloud);
	pcl::PointCloud<pcl::PointXYZ>::ConstPtr intermediateCloud = secondConverter.ConvertShared(asnPointCloud);
	PointCloudSharedConstPtr outputCloud = firstConverter.ConvertShared(intermediateCloud);

	REQUIRE(GetNumberOfPoints(*asnPointCloud) == 0);
	REQUIRE(intermediateCloud->points.size() == 0);
	REQUIRE(GetNumberOfPoints(*outputCloud) == 0);
	}
