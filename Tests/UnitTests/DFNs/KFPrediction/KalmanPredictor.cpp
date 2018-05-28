/**
 * @author Nassir W.Oumer
 */

/**
 * @addtogroup DFNsTest
 *
 * Unit testing the simple Kalman predictor.
 *
 * @{
 */

#include <catch.hpp>

#include <KFPrediction/KalmanPredictor.hpp>

#include <Errors/Assert.hpp>
#include <iostream>

using namespace dfn_ci;

enum states{ORIENTATION, POSITION, ANGULAR_VELOCITY, VELOCITY};

void printState(asn1SccRigidBodyState output, states filterState)
{
	if (filterState == ORIENTATION)
	{
		std::cout << "orientation: ";
		for (int i = 0; i < 3; i++)
		{
			std::cout << output.orient.arr[i] << "\t";
		}
		std::cout << std::endl;
	}

	else if (filterState == POSITION)
	{
		std::cout << "position: ";
		for (int i = 0; i < 3; i++)
		{
			std::cout << output.pos.arr[i] << "\t";
		}
		std::cout << std::endl;
	}

	else if (filterState == ANGULAR_VELOCITY)
	{
		std::cout << "angular velocity: ";
		for (int i = 0; i < 3; i++)
		{
			std::cout << output.angular_velocity.arr[i] << "\t";
		}
		std::cout << std::endl;
	}

	else if (filterState == VELOCITY)
	{
		std::cout << "velocity: ";
		for (int i = 0; i < 3; i++)
		{
			std::cout << output.velocity.arr[i] << "\t";
		}
		std::cout << std::endl;
	}

	std::cout << "Predited state plausible for given motion model\n";
}

TEST_CASE( "Call to process (Kalman prediction)", "[process]" )
{
	// Prepare input data
	cv::Mat pose(6,1,CV_32F);
	cv::Mat vel0(6,1,CV_32F);
	float time0 = 0;       // initialization timestamp
	float currentTime = 2; // image timestamp

	for (int i = 0; i < 6; i++)
	{
		pose.at<float>(i,0) = 2*i;
		vel0.at<float>(i,0) = 0.8*i;
	}

	// Prepare input data (in a ASN.1 data type)
	asn1SccRigidBodyState input;
	for (int i = 0; i < 3; i++)
	{
		input.orient.arr[i] = pose.at<float>(i,0);
		input.pos.arr[i] = pose.at<float>(i+3,0);
		input.angular_velocity.arr[i] = vel0.at<float>(i,0);
		input.velocity.arr[i] = vel0.at<float>(i+3,0);
	}
	input.timestamp.microseconds = currentTime;

	// Instantiate DFN
	KalmanPredictor simplePredictor;

	// Send input data to DFN
	simplePredictor.previousStateInput(input);
	simplePredictor.currentTimeInput(input.timestamp);

	// Run DFN
	simplePredictor.process();

	// Query output data from DFN
	const asn1SccRigidBodyState& outputStatePre = simplePredictor.predictedStateOutput();
	const asn1SccRigidBodyState& outputerrCovPre = simplePredictor.predictedStateCovarianceOutput();

	printState(outputStatePre, ORIENTATION);
	printState(outputStatePre, POSITION);
}

TEST_CASE( "Call to configure (Kalman prediction)", "[configure]" )
{
	KalmanPredictor simplePredictor;
	simplePredictor.setConfigurationFile("../tests/ConfigurationFiles/DFNs/KFPrediction/KalmanPredictor_Conf.yaml");
	simplePredictor.configure();
}

/** @} */
