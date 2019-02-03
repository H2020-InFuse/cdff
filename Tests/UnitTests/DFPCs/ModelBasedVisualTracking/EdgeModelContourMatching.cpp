/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFPCsTest
 * @{
 */

#include <catch.hpp>

#include <ModelBasedVisualTracking/EdgeModelContourMatching.hpp>
#include <Types/C/Time.h>
#include <Types/C/RigidBodyState.h>
#include <Types/CPP/Frame.hpp>
#include <Converters/MatToFrameConverter.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include <iostream>

using namespace CDFF::DFPC::ModelBasedVisualTracking;

// Initial pose and initial velocity
void initPose(double* guessT0, double* velocity0)
{
	// Write initial pose in guessT0
	double guessT00[16] = {
		0.014377, -0.997371, 0.071024, 1207.899488,
		0.433600, 0.070224, 0.898365, 296.456254,
		-0.900991, 0.017880, 0.433469, 1202.087847,
		0.000000, 0.000000, 0.000000, 1.000000};
	double TAdapt[16] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0};
	matrixProduct444(guessT00, TAdapt, guessT0);

	// Write initial velocity in velocity0
	double startVelocity[6] =
		{0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
	double unitToRadPerSecond = 1;
	double unitToMillimeterPerSecond = 1;
	int i = 0;
	while (i < 3)
	{
		velocity0[i] = startVelocity[i]*unitToRadPerSecond;
		i++;
	}
	while (i < 6)
	{
		velocity0[i] = startVelocity[i]*unitToMillimeterPerSecond;
		i++;
	}
}

/* Commented off b/c done in next test case, otherwise requires large memory pre-allocation
TEST_CASE( "Setup DFPC ModelBasedVisualTracking::EdgeModelContourMatching", "[DLRTrackerSetup]" )
{
	// Create DFPC instance
	EdgeModelContourMatching* contourMatching = new EdgeModelContourMatching();

	// Setup DFPC
	contourMatching->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking");
	contourMatching->setup();

	// Cleanup
	delete contourMatching;
}
*/

TEST_CASE( "Run DFPC ModelBasedVisualTracking::EdgeModelContourMatching", "[DLRTrackerRun]" )
{
	std::cout << "Unit test for CDFF::DFPC::ModelBasedVisualTracking::EdgeModelContourMatching: start" << std::endl;

	// Create DFPC instance
	EdgeModelContourMatching* contourMatching = new EdgeModelContourMatching();

	// Setup DFPC
	contourMatching->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking");
	contourMatching->setup();

	// Prepare input data: images (files -> cv::Mat -> ASN.1 Frame)
	cv::Mat inputImageLeft = cv::imread("../tests/Data/Images/DLR_OOS_camL0000.pgm", 0);
	cv::Mat inputImageRight = cv::imread("../tests/Data/Images/DLR_OOS_camR0000.pgm", 0);

	Converters::MatToFrameConverter matToFrame;
	FrameWrapper::FrameConstPtr inputFrameLeft = matToFrame.Convert(inputImageLeft);
	FrameWrapper::FrameConstPtr inputFrameRight = matToFrame.Convert(inputImageRight);

	// Send input data: images
	contourMatching->imageLeftInput(*inputFrameLeft);
	contourMatching->imageRightInput(*inputFrameRight);

	// Prepare input data: image timestamp (hardcoded values -> ASN.1 Time)
	int frameCounter = 1; // just one frame
	double dtImages = 0.33; // frame rate 3Hz
	double timeImages = frameCounter * dtImages;

	asn1SccTime imageAcquisitionTime;
	imageAcquisitionTime.microseconds = timeImages * 1000000;

	// Send input data: image timestamp
	contourMatching->imageTimeInput(imageAcquisitionTime);

	// Prepare input data: initial state (hardcoded values -> ASN.1 RigidBodyState)
	double guessT0[16];
	double velocity0[6];
	initPose(guessT0, velocity0);

	double rotationTranslation[6];
	AngleAxisFromT(guessT0, rotationTranslation);

	asn1SccRigidBodyState initState;
	initState.orient.arr[0] = rotationTranslation[0];
	initState.orient.arr[1] = rotationTranslation[1];
	initState.orient.arr[2] = rotationTranslation[2];
	initState.pos.arr[0] = rotationTranslation[3];
	initState.pos.arr[1] = rotationTranslation[4];
	initState.pos.arr[2] = rotationTranslation[5];
	initState.angular_velocity.arr[0] = velocity0[0];
	initState.angular_velocity.arr[1] = velocity0[1];
	initState.angular_velocity.arr[2] = velocity0[2];
	initState.velocity.arr[0] = velocity0[3];
	initState.velocity.arr[1] = velocity0[4];
	initState.velocity.arr[2] = velocity0[5];

	// Send input data: initial state
	contourMatching->initInput(initState);

	// Prepare input data: ego-motion (hardcoded values -> ASN.1 RigidBodyState)
	asn1SccRigidBodyState egoMotion;
	egoMotion.orient.arr[0] = 0.0;
	egoMotion.orient.arr[1] = 0.0;
	egoMotion.orient.arr[2] = 0.0;
	egoMotion.pos.arr[0] = 0.0;
	egoMotion.pos.arr[1] = 0.0;
	egoMotion.pos.arr[2] = 0.0;

	// Send input data: ego-motion
	contourMatching->egoMotionInput(egoMotion);

	// Run DFPC
	contourMatching->run();

	// Query and check output data: estimated state
	asn1SccRigidBodyState estimatedState = contourMatching->stateOutput();
	REQUIRE(estimatedState.pos.arr[0] == 0);
	REQUIRE(estimatedState.pos.arr[1] == 0);
	REQUIRE(estimatedState.pos.arr[2] == 0);

	// Query output data: success flag
	bool success = contourMatching->successOutput();

	// Cleanup
	delete contourMatching;

	std::cout << "Unit test for CDFF::DFPC::ModelBasedVisualTracking::EdgeModelContourMatching: done" << std::endl;
}

/** @} */
