/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file PointCloudPclNormalsCloudConvertersTest.cpp
 * @date 24/01/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup CommonTests
 * 
 * Testing conversion from PointCloud to PclNormalsCloud and viceversa.
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
#include <Converters/PointCloudToPclNormalsCloudConverter.hpp>
#include <Converters/PclNormalsCloudToPointCloudConverter.hpp>
#include <PointCloud.hpp>
#include <Errors/Assert.hpp>
#include <boost/smart_ptr.hpp>

using namespace Converters;
using namespace PointCloudWrapper;

TEST_CASE( "PclNormalsCloud to PointCloud and Back", "[PclNormalsCloudToPointCloud]" )
	{
	PclNormalsCloudToPointCloudConverter firstConverter;
	PointCloudToPclNormalsCloudConverter secondConverter;

	pcl::PointCloud<pcl::Normal>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		inputCloud->points.push_back( pcl::Normal(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloudSharedConstPtr asnPointCloud = firstConverter.ConvertShared(inputCloud);
	REQUIRE(GetNumberOfPoints(*asnPointCloud) == inputCloud->points.size() );
	for(int pointIndex = 0; pointIndex < static_cast<int>( inputCloud->points.size() ); pointIndex++)
		{
		REQUIRE( GetXCoordinate(*asnPointCloud, pointIndex) == inputCloud->points.at(pointIndex).normal_x );
		REQUIRE( GetYCoordinate(*asnPointCloud, pointIndex) == inputCloud->points.at(pointIndex).normal_y );
		REQUIRE( GetZCoordinate(*asnPointCloud, pointIndex) == inputCloud->points.at(pointIndex).normal_z );
		}

	pcl::PointCloud<pcl::Normal>::ConstPtr outputCloud = secondConverter.ConvertShared(asnPointCloud);
	REQUIRE(outputCloud->points.size() == inputCloud->points.size() );
	for(unsigned pointIndex = 0; pointIndex < outputCloud->points.size(); pointIndex++)
		{
		pcl::Normal inputPoint = inputCloud->points.at(pointIndex);
		pcl::Normal outputPoint = outputCloud->points.at(pointIndex);
		REQUIRE(inputPoint.normal_x == outputPoint.normal_x);	
		REQUIRE(inputPoint.normal_y == outputPoint.normal_y);			 
		REQUIRE(inputPoint.normal_z == outputPoint.normal_z);			
		}

	asnPointCloud.reset();
	} 

TEST_CASE( "PointCloud to PclPointCloud and Back (Normals)", "[PointCloud3DToPclPointCloud]" )
	{
	PclNormalsCloudToPointCloudConverter firstConverter;
	PointCloudToPclNormalsCloudConverter secondConverter;

	pcl::PointCloud<pcl::Normal>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	for(int pointIndex = 0; pointIndex < 5; pointIndex++)
		{
		inputCloud->points.push_back( pcl::Normal(pointIndex, (float)pointIndex/3, std::sqrt(pointIndex)) );
		}

	PointCloudSharedConstPtr asnPointCloud = firstConverter.ConvertShared(inputCloud);
	pcl::PointCloud<pcl::Normal>::ConstPtr intermediateCloud = secondConverter.ConvertShared(asnPointCloud);
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

TEST_CASE("Empty Point Cloud conversion (Normals)", "[EmptyPointCloud]")
	{
	PclNormalsCloudToPointCloudConverter firstConverter;
	PointCloudToPclNormalsCloudConverter secondConverter;

	pcl::PointCloud<pcl::Normal>::Ptr inputCloud = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
	PointCloudSharedConstPtr asnPointCloud = firstConverter.ConvertShared(inputCloud);
	pcl::PointCloud<pcl::Normal>::ConstPtr intermediateCloud = secondConverter.ConvertShared(asnPointCloud);
	PointCloudSharedConstPtr outputCloud = firstConverter.ConvertShared(intermediateCloud);

	REQUIRE(GetNumberOfPoints(*asnPointCloud) == 0);
	REQUIRE(intermediateCloud->points.size() == 0);
	REQUIRE(GetNumberOfPoints(*outputCloud) == 0);
	}
