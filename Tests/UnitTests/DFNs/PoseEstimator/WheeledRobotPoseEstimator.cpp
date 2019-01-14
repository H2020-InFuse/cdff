/*!
 * @addtogroup DFNsTest
 *
 * Unit Test for the DFN Pose Estimator.
 *
 *
 * @{
 */

#include <catch.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Converters/MatToFrameConverter.hpp>
#include <PoseEstimator/WheeledRobotPoseEstimator.hpp>


using namespace CDFF::DFN::PoseEstimator;
using namespace FrameWrapper;
using namespace Converters;

TEST_CASE( "Call to process (WheeledRobotPoseEstimator)", "[process]" )
{
    // Prepare input data
    cv::Mat rgb = cv::imread("../tests/Data/Images/AlgeriaDesert.jpg", cv::IMREAD_COLOR);
    cv::Mat gray_cv_img;
    cv::cvtColor(rgb, gray_cv_img, cv::COLOR_RGB2GRAY);

    const asn1SccFrame* inImage = MatToFrameConverter().Convert(gray_cv_img);
    const asn1SccFrame* inDepth = MatToFrameConverter().Convert(gray_cv_img);
    const asn1SccFrame* frame_without_background = MatToFrameConverter().Convert(gray_cv_img);


    asn1SccVectorXdSequence* inPrimitives = new asn1SccVectorXdSequence();
    asn1SccVectorXdSequence_Initialize(inPrimitives);

    // fill in with 2 x wheel (x,y,rad)
    // TODO : extract from a urdf file
    inPrimitives->nCount = 2;
    inPrimitives->arr[0].nCount = 3;
    inPrimitives->arr[0].arr[0] = 100;
    inPrimitives->arr[0].arr[1] = 100;
    inPrimitives->arr[0].arr[2] = 0.5;
    inPrimitives->arr[1].arr[0] = 150;
    inPrimitives->arr[1].arr[1] = 100;
    inPrimitives->arr[1].arr[2] = 0.5;

    // Instantiate DFN
    std::unique_ptr<WheeledRobotPoseEstimator> m_pose_estimator (new WheeledRobotPoseEstimator);

    // Send input data to DFN
    m_pose_estimator->imageInput(*inImage);
    m_pose_estimator->depthInput(*inDepth);
    m_pose_estimator->primitivesInput(*inPrimitives);
    m_pose_estimator->imageInput(*frame_without_background);

    // Run DFN more than the number of stabilizing frames (default = 4)
    for(unsigned int i = 0; i < 5; i++)
    {
        m_pose_estimator->process();
    }

    // Query output data from DFN
    const asn1SccPosesSequence & output = m_pose_estimator->posesOutput();

    delete inPrimitives;
    delete inImage;
    delete inDepth;
    delete frame_without_background;
}

TEST_CASE( "Call to configure (WheeledRobotPoseEstimator) ", "[configure]" )
{
    // Instantiate DFN
    std::unique_ptr<WheeledRobotPoseEstimator> m_pose_estimator (new WheeledRobotPoseEstimator);

    // Setup DFN
    m_pose_estimator->setConfigurationFile("../tests/ConfigurationFiles/DFNs/PoseEstimator/WheeledRobotPoseEstimator.yaml");
    m_pose_estimator->configure();

}

/** @} */
