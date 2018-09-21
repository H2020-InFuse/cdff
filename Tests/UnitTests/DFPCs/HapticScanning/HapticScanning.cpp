/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HapticScanning.cpp
 * @date 18/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Unit Test for the DFPCs HapticScanning.
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
#include <HapticScanning/HapticScanning.hpp>
#include <Errors/Assert.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ForceMeshGenerator/ThresholdForce.hpp>
#include <PointCloudToPclPointCloudConverter.hpp>

#include "../../DFNs/ForceMeshGenerator/ForceMeshHelperFunctions.hpp"

using namespace CDFF::DFPC;


/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Success Call to Configure (HapticScanning)", "[configureSuccess]" )
{
    std::unique_ptr<HapticScanning> haptic_scanning( new HapticScanning() );
    haptic_scanning->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/HapticScanning/DfpcHapticScanning_conf01.yaml");
    haptic_scanning->setup();
}

TEST_CASE( "Success Call to Process (HapticScanning)", "[processSuccess]" )
{
    std::unique_ptr<HapticScanning> haptic_scanning( new HapticScanning() );
    haptic_scanning->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/HapticScanning/DfpcHapticScanning_conf01.yaml");
    haptic_scanning->setup();

    // Read input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    Reader.read("../tests/Data/PointClouds/bunny0.ply", *cloud);

    // Create input data using the input cloud
    std::vector<std::pair<pcl::PointXYZ, double> > points = ForceMeshHelperFunctions::getInputData(cloud);

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

    haptic_scanning->roverPoseInput(rover_pose);
    haptic_scanning->positionAndForceInput(*positions, *forces);

    haptic_scanning->run();

    const asn1SccPointcloud& output = haptic_scanning->pointCloudOutput();
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr output_cloud = Converters::PointCloudToPclPointCloudConverter().Convert(&output);

    for( const pcl::PointXYZ & out_point : output_cloud->points )
    {
        CHECK( ForceMeshHelperFunctions::checkPointCloudContainsPoint( out_point, cloud ) );
    }
}

/** @} */
