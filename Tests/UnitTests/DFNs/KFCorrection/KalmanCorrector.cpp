/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file KalmanPredictor.cpp
 * @date 8/05/2018
 * @author Nassir W.Oumer
 */

/*!
 * @addtogroup DFNsTest
 *
 * Testing simple Kalman Corrector.
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
#include <KFCorrection/KalmanCorrector.hpp>
#include <Errors/Assert.hpp>
#include <iostream>

using namespace dfn_ci;
//using namespace Common;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
enum correctedState{ORIENTATION,POSITION};
void printUpdated(asn1SccRigidBodyState output, correctedState state)
{
	if(state==ORIENTATION)
	{
	std::cout<<"update orientation: ";

	for(int i=0;i<3;i++)
		std::cout<<output.orient.arr[i]<<"\t";
	std::cout<<std::endl;

	}
	else if(state==POSITION)
	{
		std::cout<<"updated position: ";
		for(int i=0;i<3;i++)
			std::cout<<output.pos.arr[i]<<"\t";
		std::cout<<std::endl;
	}

}

TEST_CASE( "Call to process (Kalman correction)", "[process]" )
	{

	cv::Mat measurement(6,1,CV_32F);
	cv::Mat vel0(6,1,CV_32F);

	for(int i=0;i<3;i++)
	{
	measurement.at<float>(i,0)= 2*i;
	measurement.at<float>(i+3,0)=3*(i+1);
	}

	//convert intput to ASN format

	 asn1SccRigidBodyState inputMeasure;
	for(int i=0;i<3;i++)
	{
		inputMeasure.orient.arr[i]=measurement.at<float>(i,0);
		inputMeasure.pos.arr[i]=measurement.at<float>(i+3,0);
	}

	asn1SccRigidBodyState inputPredicted;

	for(int i=0;i<3;i++)
	{
		inputPredicted.orient.arr[i]=1.6*i;
		inputPredicted.pos.arr[i]=4*(i+1);
		inputPredicted.angular_velocity.arr[i]=1;
		inputPredicted.velocity.arr[i]=1;
	}


	 asn1SccRigidBodyState inputPredictedCov;

	for(int row=0;row<3;row++)
	{
		for(int col=0;col<3;col++)
		{
		inputPredictedCov.cov_orientation.arr[row].arr[col]= 0.1;
		inputPredictedCov.cov_position.arr[row].arr[col]=0.1;
		inputPredictedCov.cov_angular_velocity.arr[row].arr[col]=0.2;
		inputPredictedCov.cov_velocity.arr[row].arr[col]=0.5;
		}
	}


	KalmanCorrector simpleCorrector;
	simpleCorrector.predictedStateInput(inputPredicted);
	simpleCorrector.measurementInput(inputMeasure);
	simpleCorrector.predictedStateCovarianceInput(inputPredictedCov);

	simpleCorrector.process();
	asn1SccRigidBodyState outputStatePost= simpleCorrector.correctedStateOutput();
	asn1SccRigidBodyState outputerrCovPre = simpleCorrector.stateCovarianceOutput();

	printUpdated(outputStatePost,ORIENTATION);
	printUpdated(outputStatePost,POSITION);

	}


TEST_CASE( "Call to configure (Kalman correction)", "[configure]" )
	{
	KalmanCorrector simpleCorrector;
	simpleCorrector.setConfigurationFile("../tests/ConfigurationFiles/DFNs/KFCorrection/KalmanCorrector_Conf.yaml");
	simpleCorrector.configure();
	}


/** @} */
