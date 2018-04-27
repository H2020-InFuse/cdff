/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file KalmanPredictor.cpp
 * @date 26/04/2018
 * @author Nassir W.Oumer
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Testing simple Kalman predictor.
 * 
 * 
 * @{
 */


/* --------------------------------------------------------------------------
 *
 * Definitions
 * Catch definition must be before the includes, otherwise catch will not compile.
 *
 * --------------------------------------------------------------------------
 */
#define CATCH_CONFIG_MAIN

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include <Catch/catch.h>
#include <KFPrediction/KalmanPredictor.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;
//using namespace Common;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */
TEST_CASE( "Call to process", "[process]" ) 
	{
	
	cv::Mat pose(6,1,CV_32F);
	cv::Mat vel0(6,1,CV_32F);
	float time0=0; //initialization time stamp
	float currentTime=2; //image timestamp
	for(int i=0;i<6;i++)
	{
	pose.at<float>(i,0)= 2*i;
	vel0.at<float>(i,0)= 0.8*i;
	}

	//convert intput to ASN format

	 RigidBodyState input;

	for(int i=0;i<3;i++)
	{
	 	input.orient.arr[i]=pose.at<float>(i,0);
		input.pos.arr[i]=pose.at<float>(i+3,0);
		input.angular_velocity.arr[i]=vel0.at<float>(i,0);
		input.velocity.arr[i]=vel0.at<float>(i+3,0);
	}

	//Time

	input.timestamp.microseconds=currentTime;	

	KalmanPredictor simplePredictor;
	
	simplePredictor.previousStateInput(input);
	simplePredictor.currentTimeInput(input.timestamp);
	
	simplePredictor.process();

	RigidBodyState outputStatePre= simplePredictor.predictedStateOutput();
	RigidBodyState outputerrCovPre = simplePredictor.predictedStateCovarianceOutput();
		
	}


TEST_CASE( "Call to configure", "[configure]" )
	{
	KalmanPredictor simplePredictor;
	simplePredictor.setConfigurationFile("../tests/ConfigurationFiles/DFNs/KFPrediction/KalmanPredictor_Conf.yaml");
	simplePredictor.configure();	
	}


/** @} */
