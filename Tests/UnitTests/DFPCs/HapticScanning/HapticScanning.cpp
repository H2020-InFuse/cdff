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
#include <Converters/PointCloudToPclPointCloudConverter.hpp>

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
    std::vector<asn1SccPose> arm_ee_poses(points.size());
    std::vector<asn1SccWrench> arm_ee_wrenches(points.size());

    for ( size_t index = 0; index < points.size(); ++index )
    {
        asn1SccPose arm_ee_pose;
        asn1SccPose_Initialize(&arm_ee_pose);
        arm_ee_pose.pos.arr[0] = points[index].first.x;
        arm_ee_pose.pos.arr[1] = points[index].first.y;
        arm_ee_pose.pos.arr[2] = points[index].first.z;

        asn1SccWrench arm_ee_wrench;
        asn1SccWrench_Initialize(&arm_ee_wrench);
        arm_ee_wrench.force.arr[0] = points[index].second;

        arm_ee_poses.push_back(arm_ee_pose);
        arm_ee_wrenches.push_back(arm_ee_wrench);
    }

    asn1SccPose rover_pose = ForceMeshHelperFunctions::getRoverPose();

    // Process all the input data
    haptic_scanning->armBasePoseInput(rover_pose);

    for (auto i = 0; i < points.size(); ++i) {
        haptic_scanning->armEndEffectorPoseInput(arm_ee_poses[i]);
        haptic_scanning->armEndEffectorWrenchInput(arm_ee_wrenches[i]);

        // Run DFN
        haptic_scanning->run();
    }

    // Query output data from DFN
    const asn1SccPointcloud& output = haptic_scanning->pointCloudOutput();

    auto output_cloud = Converters::PointCloudToPclPointCloudConverter().Convert(&output);
    for( const pcl::PointXYZ & out_point : output_cloud->points )
    {
        CHECK( ForceMeshHelperFunctions::checkPointCloudContainsPoint( out_point, cloud ) );
    }
}

/** @} */
