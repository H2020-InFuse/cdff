/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

#pragma once

#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <Types/C/Pose.h>


namespace ForceMeshHelperFunctions
{
    bool checkPointCloudContainsPoint ( const pcl::PointXYZ & out_point, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud );

    Eigen::Vector3d getPosition();

    Eigen::Quaterniond getQuaternion();

    asn1SccPose getEndEffectorPose();

    asn1SccPose getRoverPose();

    std::vector<std::pair<pcl::PointXYZ, double> > getInputData (const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
}
