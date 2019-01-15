/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file LidarBasedTracking.cpp
 * @date 10/01/2019
 * @author Loïc Le Cabec
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN LidarBasedTracking.
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
#include <LidarBasedTracking/LidarBasedTracking.hpp>
#include <Types/C/Pointcloud.h>
#include <Types/C/RigidBodyState.h>
#include <Types/C/TransformWithCovariance.h>
#include <fstream>
#include <iostream>
#include <sstream>

void convString(std::string const&stdtype,  asn1SccT_String &asntype)
{
    asntype.nCount = static_cast<int>(stdtype.size() +1);
    memcpy(asntype.arr, stdtype.data(), static_cast<size_t>(asntype.nCount));
}

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process (LidarBasedTracking)", "[process]" )
{
    // Prepare input data
    cv::Mat initialState = cv::Mat(12, 1, CV_32F, float(0));
    std::string targetCloudPath =
            "../tests/Data/PointClouds/SatelliteWithNormals.pcd";
    float targetCloudScale = 0.001;

    // Initialise a frame and set its metadata
    asn1SccPointcloud *sourceCloud = new asn1SccPointcloud();
    asn1SccPointcloud_Initialize(sourceCloud);

    // Read point cloud
    std::ifstream pointcloudFile(targetCloudPath, std::ios::in);
    std::string line;
    for (int i = 0; i < 11; i++)
    {
        getline(pointcloudFile, line);
    }

    Eigen::MatrixXd pointcloudData(992,3);
    int iline = 0;
    while (getline(pointcloudFile, line))
    {
        std::istringstream iss(line);
        iss >> pointcloudData(iline,0)
            >> pointcloudData(iline,1)
            >> pointcloudData(iline,2);
        iline++;
    }
    pointcloudData *= targetCloudScale;

    // Generate fake timeStamp
    asn1SccTime *timeStamp = new asn1SccTime();
    asn1SccTime_Initialize(timeStamp);
    timeStamp->microseconds = static_cast<asn1SccT_Int64>(123456789);
    timeStamp->usecPerSec = static_cast<asn1SccT_Int32>(1000000);

    // Generate fake pose
    asn1SccTransformWithCovariance pose;
    asn1SccTransformWithCovariance_Initialize(&pose);
    for (int i = 0; i < 6; i++)
    {
        for(int j = 0; j < 6; j++)
        {
            pose.data.cov.arr[i].arr[j] = 0.0;
        }
    }
    for (int i = 0; i < 3; i++)
    {
        pose.data.translation.arr[i] = 0.0;
        pose.data.orientation.arr[i+1] = 0.0;
        pose.data.cov.arr[i].arr[i] = 1.0;
    }
    pose.data.orientation.arr[0] = 1.0;
    pose.metadata.msgVersion = transformWithCovariance_version;
    convString("fake_sensor", pose.metadata.producerId);
    pose.metadata.parentTime = *timeStamp;
    pose.metadata.childTime = *timeStamp;

    // Fill the metadata
    sourceCloud->metadata.msgVersion = pointCloud_Version;
    convString("sourceFrame", sourceCloud->metadata.frameId);
    convString("fakeLidar", sourceCloud->metadata.sensorId);
    sourceCloud->metadata.timeStamp = *timeStamp;
    sourceCloud->metadata.height = static_cast<asn1SccT_Int32>(1);
    sourceCloud->metadata.width = static_cast<asn1SccT_Int32>(992);
    sourceCloud->metadata.isRegistered = static_cast<asn1SccT_Int32>(false);
    sourceCloud->metadata.hasFixedTransform = static_cast<asn1SccT_Boolean>(true);
    sourceCloud->metadata.isOrdered = static_cast<asn1SccT_Boolean>(false);

    convString("LocalTerrainFrame", pose.metadata.parentFrameId);
    convString("RobotFrame", pose.metadata.childFrameId);
    sourceCloud->metadata.pose_robotFrame_sensorFrame = pose;

    convString("RobotFrame", pose.metadata.parentFrameId);
    convString("LidarFrame", pose.metadata.childFrameId);
    sourceCloud->metadata.pose_fixedFrame_robotFrame = pose;

    // Fill the data
    for(int i = 0; i < 992; i++)
    {
        sourceCloud->data.points.arr[i].arr[0] = pointcloudData(i,0);
        sourceCloud->data.points.arr[i].arr[1] = pointcloudData(i,1);
        sourceCloud->data.points.arr[i].arr[2] = pointcloudData(i,2);
        sourceCloud->data.points.nCount++;
    }

    // Instantiate DFN
    CDFF::DFN::LidarBasedTracking::LidarBasedTracking *trackingDFN  = new CDFF::DFN::LidarBasedTracking::LidarBasedTracking();

    // Send input data to DFN
    trackingDFN->init(initialState, targetCloudPath, targetCloudScale);
    trackingDFN->sourceCloudInput(*sourceCloud);
    trackingDFN->m_parameters.targetCloudHasNormals = true;

    // Run DFN
    trackingDFN->process();

    // Query output data from DFN
    const asn1SccRigidBodyState& outputState = trackingDFN->stateOutput();
    Eigen::Vector4f orientation;
    for (int i =0; i < 4; i++)
    {
        orientation(i) = outputState.orient.arr[i];
    }
    REQUIRE( outputState.timestamp.microseconds == sourceCloud->metadata.timeStamp.microseconds );
    REQUIRE( outputState.timestamp.usecPerSec == sourceCloud->metadata.timeStamp.usecPerSec );
    for (int i = 0; i < 3; i++)
    {
        REQUIRE( outputState.cov_position.arr[i].arr[i] >= 0 );
        REQUIRE( outputState.cov_orientation.arr[i].arr[i] >= 0 );
        REQUIRE( outputState.cov_velocity.arr[i].arr[i] >= 0 );
        REQUIRE( outputState.cov_angular_velocity.arr[i].arr[i] >= 0 );
    }
    REQUIRE( orientation.norm() == 1 );

    // Cleanup
    delete(sourceCloud);
    delete(trackingDFN);
}

TEST_CASE( "Call to configure (LidarBasedTracking)", "[configure]" )
{
/*    // Instantiate DFN
    CDFF::DFN::LidarBasedTracking::LidarBasedTracking trackingDFN;

    // Setup DFN
    trackingDFN.setConfigurationFile("../tests/ConfigurationFiles/DFNs/LidarBasedTracking/LidarBasedTracking_Conf.yaml");
    trackingDFN.configure();
*/}

/** @} */
