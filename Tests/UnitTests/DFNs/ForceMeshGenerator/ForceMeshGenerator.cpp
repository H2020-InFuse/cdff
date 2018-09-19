/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ForceMeshGenerator.cpp
 * @date 10/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Force Mesh Generator.
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
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ForceMeshGenerator/ThresholdForce.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>
#include "ForceMeshHelperFunctions.hpp"
#include <Sequences.h>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Force Mesh Generator" )
{
    // Read input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    Reader.read("../tests/Data/PointClouds/bunny0.ply", *cloud);

    // Create input data using the input cloud
    std::vector<std::pair<pcl::PointXYZ, double> > points = ForceMeshHelperFunctions::getInputData(cloud);

    // Instantiate DFN
    std::unique_ptr<CDFF::DFN::ForceMeshGenerator::ThresholdForce> generator (new CDFF::DFN::ForceMeshGenerator::ThresholdForce() );

    // Convert the input data
    asn1SccPointsSequence * positions = new asn1SccPointsSequence;
    asn1SccPointsSequence_Initialize(positions);
    asn1SccDoublesSequence * forces = new asn1SccDoublesSequence;
    asn1SccDoublesSequence_Initialize(forces);

    auto size = points.size();
    positions->nCount = size;
    forces->nCount = size;

    for ( unsigned int index = 0; index < size; index ++ )
    {
        positions->arr[index].arr[0] = points[index].first.x;
        positions->arr[index].arr[1] = points[index].first.y;
        positions->arr[index].arr[2] = points[index].first.z;
        forces->arr[index] = points[index].second;
    }

    asn1SccPose rover_pose = ForceMeshHelperFunctions::getRoverPose();

    // Send input data to DFN
    generator->roverPoseInput(rover_pose);
    generator->positionAndForceInput(*positions, *forces);

    // Run DFN
    generator->process();

    // Query output data from DFN
    const asn1SccPointcloud& output = generator->pointCloudOutput();

    auto output_cloud = Converters::PointCloudToPclPointCloudConverter().Convert(&output);
    for( const pcl::PointXYZ & out_point : output_cloud->points )
    {
        CHECK( ForceMeshHelperFunctions::checkPointCloudContainsPoint( out_point, cloud ) );
    }

    delete positions, forces;
}

/** @} */
