/**
 * @author Nassir W.Oumer
 */

/**
 * Unit tests for the DFN KalmanCorrector
 */

/**
 * @addtogroup DFNsTest
 * @{
 */

#include <catch.hpp>
#include <KFCorrection/KalmanCorrector.hpp>
#include <iostream>

using namespace dfn_ci;

enum correctedState {ORIENTATION, POSITION};
void printUpdated(const asn1SccRigidBodyState& output, correctedState state)
{
	if (state == ORIENTATION)
	{
		std::cout << "Updated orientation: ";
		for(int i=0;i<3;i++)
		{
			std::cout << output.orient.arr[i] << "\t";
		}
		std::cout << std::endl;
	}
	else if (state == POSITION)
	{
		std::cout << "Updated position: ";
		for(int i=0;i<3;i++)
		{
			std::cout << output.pos.arr[i] << "\t";
		}
		std::cout << std::endl;
	}
}

TEST_CASE( "Call to process (Kalman correction)", "[process]" )
{
	cv::Mat vel0(6, 1, CV_32F);

	// Prepare input data for port "predictedState"
	asn1SccRigidBodyState inputPredicted;
	for(int i=0;i<3;i++)
	{
		inputPredicted.orient.arr[i]           = 1.6*i;
		inputPredicted.pos.arr[i]              = 4*(i+1);
		inputPredicted.angular_velocity.arr[i] = 1;
		inputPredicted.velocity.arr[i]         = 1;
	}

	// Prepare input data for port "measurement"
	cv::Mat measurement(6, 1, CV_32F);
	for(int i=0;i<3;i++)
	{
		measurement.at<float>(i,0)   = 2*i;
		measurement.at<float>(i+3,0) = 3*(i+1);
	}

	asn1SccRigidBodyState inputMeasure;
	for(int i=0;i<3;i++)
	{
		inputMeasure.orient.arr[i] = measurement.at<float>(i,0);
		inputMeasure.pos.arr[i]    = measurement.at<float>(i+3,0);
	}

	// Prepare input data for port "predictedStateCovariance"
	asn1SccRigidBodyState inputPredictedCov;
	for(int row=0;row<3;row++)
	{
		for(int col=0;col<3;col++)
		{
			inputPredictedCov.cov_orientation.arr[row].arr[col]      = 0.1;
			inputPredictedCov.cov_position.arr[row].arr[col]         = 0.1;
			inputPredictedCov.cov_angular_velocity.arr[row].arr[col] = 0.2;
			inputPredictedCov.cov_velocity.arr[row].arr[col]         = 0.5;
		}
	}

	// Instantiate DFN
	KalmanCorrector simpleCorrector;

	// Send input data to DFN
	simpleCorrector.predictedStateInput(inputPredicted);
	simpleCorrector.measurementInput(inputMeasure);
	simpleCorrector.predictedStateCovarianceInput(inputPredictedCov);

	// Run DFN
	simpleCorrector.process();

	// Query output data from DFN
	const asn1SccRigidBodyState& outputStatePost = simpleCorrector.correctedStateOutput();
	const asn1SccRigidBodyState& outputerrCovPre = simpleCorrector.stateCovarianceOutput();

	// Print output data
	printUpdated(outputStatePost, ORIENTATION);
	printUpdated(outputStatePost, POSITION);
}

TEST_CASE( "Call to configure (Kalman correction)", "[configure]" )
{
	// Instantiate DFN
	KalmanCorrector simpleCorrector;

	// Setup DFN
	simpleCorrector.setConfigurationFile("../tests/ConfigurationFiles/DFNs/KFCorrection/KalmanCorrector_Conf.yaml");
	simpleCorrector.configure();
}

/** @} */
