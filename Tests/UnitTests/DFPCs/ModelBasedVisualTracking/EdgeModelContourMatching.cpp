/**
 * @author Nassir W. Oumer
 */

/**
 * @addtogroup DFPCsTest
 * @{
 */

#include <catch.hpp>

#include <ModelBasedVisualTracking/EdgeModelContourMatching.hpp>
#include <Types/C/RigidBodyState.h>
#include <Types/C/Time.h>
#include <Converters/MatToFrameConverter.hpp>

using namespace CDFF::DFPC::ModelBasedVisualTracking;
using namespace Converters;
using namespace FrameWrapper;

using namespace DLRtracker;

// mock external DFPC, to initialize
void initPose(double* guessT0, double* velocity0)
{
	double startVelocity[6] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
	double unitToRadPerSecond = 1;
	double unitToMillimeterPerSecond = 1;
	for (int i = 0; i < 6; i++)
	{
		if (i < 3)
		{
			velocity0[i] = startVelocity[i]*unitToRadPerSecond;
		}
		else
		{
			velocity0[i] = startVelocity[i]*unitToMillimeterPerSecond;
		}
	}

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
}

/* Commented off b/c done in [processDLRTracker] TEST CASE, otherwise requires large memory pre-allocation
TEST_CASE( "Call to Setup (EdgeModelContourMatching)", "[configureDLRTracker]" )
{
	EdgeModelContourMatching* contourMatching = new EdgeModelContourMatching();

	contourMatching->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking");
	contourMatching->setup();

	delete contourMatching;
}
*/

TEST_CASE( "Call to Run (EdgeModelContourMatching)", "[processDLRTracker]" )
{
	// image type: ASN frame, cv::Mat --> ASN Frame
	// cv:Mat image type
	cv::Mat inputImageLeft = cv::imread("../tests/Data/Images/DLR_OOS_camL0000.pgm",0);
	cv::Mat inputImageRight = cv::imread("../tests/Data/Images/DLR_OOS_camR0000.pgm",0);
	MatToFrameConverter matToFrame;
	// ASN Frame type
	FrameConstPtr inputFrameLeft = matToFrame.Convert(inputImageLeft);
	FrameConstPtr inputFrameRight = matToFrame.Convert(inputImageRight);

	EdgeModelContourMatching* contourMatching = new EdgeModelContourMatching();

	contourMatching->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking");
	contourMatching->setup();

	// Image
	contourMatching->imageLeftInput(*inputFrameLeft);
	contourMatching->imageRightInput(*inputFrameRight);

	// Time
	int frameCounter = 1; // just one frame
	double dtImages = 0.33; // frame rate 3Hz
	double timeImages = frameCounter * dtImages;

	asn1SccTime imageAcquisitionTime;
	imageAcquisitionTime.microseconds = timeImages*1000000;

	contourMatching->imageTimeInput(imageAcquisitionTime);

	// Initial pose
	double guessT0[16];
	double velocity0[6];
	double rotationTranslation[6];
	initPose(guessT0, velocity0);
	AngleAxisFromT(guessT0, rotationTranslation);

	asn1SccRigidBodyState initState;
	initState.orient.arr[0] = rotationTranslation[0];
	initState.orient.arr[1] = rotationTranslation[1];
	initState.orient.arr[2] = rotationTranslation[2];

	initState.pos.arr[0] = rotationTranslation[3];
	initState.pos.arr[1] = rotationTranslation[4];
	initState.pos.arr[2] = rotationTranslation[5];

	// Initial velocity
	initState.angular_velocity.arr[0] = velocity0[0];
	initState.angular_velocity.arr[1] = velocity0[1];
	initState.angular_velocity.arr[2] = velocity0[2];

	initState.velocity.arr[0] = velocity0[3];
	initState.velocity.arr[1] = velocity0[4];
	initState.velocity.arr[2] = velocity0[5];

	contourMatching->initInput(initState);

	// EgoMotion
	asn1SccRigidBodyState egoMotion;
	egoMotion.orient.arr[0] = 0.0;
	egoMotion.orient.arr[1] = 0.0;
	egoMotion.orient.arr[2] = 0.0;

	egoMotion.pos.arr[0] = 0.0;
	egoMotion.pos.arr[1] = 0.0;
	egoMotion.pos.arr[2] = 0.0;

	contourMatching->egoMotionInput(egoMotion);

	// Run DFPC
	contourMatching->run();

	// Output ASN type
	bool success = contourMatching->successOutput();
	asn1SccRigidBodyState estimatedState = contourMatching->stateOutput();

	REQUIRE(estimatedState.pos.arr[0] == 0);
	REQUIRE(estimatedState.pos.arr[1] == 0);
	REQUIRE(estimatedState.pos.arr[2] == 0);

	delete contourMatching;
}

/** @} */
