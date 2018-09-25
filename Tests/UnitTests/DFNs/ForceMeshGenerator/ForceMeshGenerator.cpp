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
#include <boost/cast.hpp>

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <ForceMeshGenerator/ThresholdForce.hpp>
#include <Converters/PointCloudToPclPointCloudConverter.hpp>
#include <Types/C/Sequences.h>

#include "ForceMeshHelperFunctions.hpp"


/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

 using namespace CDFF::DFN;

TEST_CASE( "Force Mesh Generator" )
{
    // Read input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    Reader.read("../tests/Data/PointClouds/bunny0.ply", *cloud);

    // Create input data using the input cloud
    std::vector<std::pair<pcl::PointXYZ, double> > points = ForceMeshHelperFunctions::getInputData(cloud);

    // Instantiate DFN
    std::unique_ptr<ForceMeshGenerator::ThresholdForce> generator (new ForceMeshGenerator::ThresholdForce() );

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
    generator->armBasePoseInput(rover_pose);

    for (auto i = 0; i < points.size(); ++i) {
        generator->armEndEffectorPoseInput(arm_ee_poses[i]);
        generator->armEndEffectorWrenchInput(arm_ee_wrenches[i]);

        // Run DFN
        generator->process();
    }

    // Query output data from DFN
    const asn1SccPointcloud& output = generator->pointCloudOutput();

    auto output_cloud = Converters::PointCloudToPclPointCloudConverter().Convert(&output);
    for( const pcl::PointXYZ & out_point : output_cloud->points )
    {
        CHECK( ForceMeshHelperFunctions::checkPointCloudContainsPoint( out_point, cloud ) );
    }
}

/** @} */
