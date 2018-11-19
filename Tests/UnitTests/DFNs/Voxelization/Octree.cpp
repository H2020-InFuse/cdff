/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file Octree.cpp
 * @date 17/09/2018
 * @author Irene Sanz
 */

/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Voxelization.
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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <Voxelization/Octree.hpp>
#include <Converters/OctreeToPclOctreeConverter.hpp>

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

TEST_CASE( "Call to process (Octree)", "[process]" )
{
    // Prepare input data
    cv::Mat img = imread("../tests/Data/Images/depth_img.jpg", cv::IMREAD_GRAYSCALE);

    int scale = 5;
    cv::resize(img, img, cv::Size(img.cols/scale,img.rows/scale));
    Converters::MatToFrameConverter matToFrame;
    FrameWrapper::FrameConstPtr inputFrame = matToFrame.Convert(img);

    // Instantiate DFN
    std::unique_ptr<CDFF::DFN::Voxelization::Octree> octree(new CDFF::DFN::Voxelization::Octree());

    // Send input data to DFN
    octree->depthInput(*inputFrame);

    // Run DFN
    octree->process();

    // Query output data from DFN
    const asn1SccOctree & output = octree->octreeOutput();
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> output_octree = Converters::OctreeToPclOctreeConverter().Convert(output);

    // Check the output
    CHECK( output_octree.getResolution() == 1.0 );
    CHECK( output_octree.getInputCloud()->points.empty() == false );
}

/** @} */