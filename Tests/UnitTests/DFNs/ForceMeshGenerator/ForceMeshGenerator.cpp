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

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

namespace
{
    //=====================================================================================================================
    bool checkPointCloudContainsPoint ( const pcl::PointXYZ & out_point, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud )
    {
        if( std::isnan(out_point.x) )
        {
            return true;
        }

        pcl::KdTree<pcl::PointXYZ>::Ptr tree_ (new pcl::KdTreeFLANN<pcl::PointXYZ>);
        tree_->setInputCloud(cloud);

        std::vector<int> nn_indices (1);
        std::vector<float> nn_dists (1);

        tree_->nearestKSearch(out_point, 1, nn_indices, nn_dists);
        return (nn_dists[0] <= 0.0000000001);
    }

    //=====================================================================================================================
    Eigen::Vector3d getPosition()
    {
        return Eigen::Vector3d(1, 2, -1);
    }

    //=====================================================================================================================
    Eigen::Quaterniond getQuaternion()
    {
        Eigen::Quaterniond quaternion (2, 0, 1, -3);
        quaternion.normalize();
        return quaternion;
    }

    //=====================================================================================================================
    asn1SccPose getRoverPose()
    {
        asn1SccPose rover_pose;
        Eigen::Vector3d position = ::getPosition();
        Eigen::Quaterniond quaternion = ::getQuaternion();

        rover_pose.pos.arr[0] = position[0];
        rover_pose.pos.arr[1] = position[1];
        rover_pose.pos.arr[2] = position[2];

        asn1SccOrientation orient;
        rover_pose.orient.arr[0] = quaternion.w();
        rover_pose.orient.arr[1] = quaternion.x();
        rover_pose.orient.arr[2] = quaternion.y();
        rover_pose.orient.arr[3] = quaternion.z();

        return rover_pose;
    }

    //=====================================================================================================================
    std::vector<std::pair<pcl::PointXYZ, double> > getInputData (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
    {
        Eigen::Vector3d position = ::getPosition();
        Eigen::Quaterniond quaternion = ::getQuaternion();
        Eigen::Matrix3d rotation = quaternion.toRotationMatrix();

        std::vector<std::pair<pcl::PointXYZ, double> > points;
        int max = 1000, min = 1;
        for (unsigned int index = 0; index < cloud->size(); index = index + 100) {
            //Create random force sensor reading
            int force = rand() % (max - min + 1) + min;

            //Apply the inverse transformation to the points to get the values in the end effector coordinate system
            Eigen::Vector3d point_eigen(cloud->points[index].x, cloud->points[index].y, cloud->points[index].z);
            point_eigen = rotation.inverse() * point_eigen - position;

            pcl::PointXYZ point(point_eigen[0], point_eigen[1], point_eigen[2]);
            points.push_back(std::make_pair(point, force));
        }

        // Create random points without force feedback
        for (unsigned int index = 0; index < cloud->size() / 4; index++)
        {
            int x = rand() % (max - min + 1) + min;
            int y = rand() % (max - min + 1) + min;
            int z = rand() % (max - min + 1) + min;

            pcl::PointXYZ point(x, y, z);
            points.push_back(std::make_pair(point, 0.1));
        }

        return points;
    }

}

//=====================================================================================================================
TEST_CASE( "Force Mesh Generator" )
{
    // Read input cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PLYReader Reader;
    Reader.read("../tests/Data/PointClouds/bunny0.ply", *cloud);

    // Create input data using the input cloud
    std::vector<std::pair<pcl::PointXYZ, double> > points = ::getInputData(cloud);

    // Instantiate DFN
    std::unique_ptr<CDFF::DFN::ForceMeshGenerator::ThresholdForce> generator (new CDFF::DFN::ForceMeshGenerator::ThresholdForce() );

    // Convert the input data
    asn1SccPointArray positions;
    asn1SccDoubleArray forces;

    auto size = points.size();
    positions.nCount = size;
    forces.nCount = size;

    for ( unsigned int index = 0; index < size; index ++ )
    {
        positions.arr[index].arr[0] = points[index].first.x;
        positions.arr[index].arr[1] = points[index].first.y;
        positions.arr[index].arr[2] = points[index].first.z;
        forces.arr[index] = points[index].second;
    }

    asn1SccPose rover_pose = ::getRoverPose();

    // Send input data to DFN
    generator->roverPoseInput(rover_pose);
    generator->positionAndForceInput(positions, forces);

    // Run DFN
    generator->process();

    // Query output data from DFN
    const asn1SccPointcloud& output = generator->pointCloudOutput();

    auto output_cloud = Converters::PointCloudToPclPointCloudConverter().Convert(&output);
    for( const pcl::PointXYZ & out_point : output_cloud->points )
    {
        CHECK( ::checkPointCloudContainsPoint( out_point, cloud ) );
    }
}

/** @} */
