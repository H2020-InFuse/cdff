/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file FeaturesMatching3D.cpp
 * @date 23/05/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFPCsTest
 * 
 * Unit Test for the DFPCs ContourMatching/pose estimation.
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

#include <ModelBasedVisualTracking/EdgeModelContourMatching.hpp>
#include <catch.hpp>
#include <Errors/Assert.hpp>
#include "RigidBodyState.h"
#include "Time.h"
#include "Eigen.h"
#include <FrameToMatConverter.hpp>
#include <MatToFrameConverter.hpp>
#include <Converters/SupportTypes.hpp>
/*
#include <ConversionCache/ConversionCache.hpp>
#include <Mocks/Common/Converters/FrameToMatConverter.hpp>
#include <Mocks/Common/Converters/MatToFrameConverter.hpp>
#include <Stubs/Common/ConversionCache/CacheHandler.hpp>
*/
#include <iostream>

using namespace CDFF;
using namespace DFPC;

using namespace Converters;
//using namespace Common;
using namespace Converters::SupportTypes;
using namespace FrameWrapper;


/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

// mock external DFPC

void initPose(double* T_guess0, double* vel0 )
{
	// Initialization: IROS 2018-Roberto pro

	 double  vel[6] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};

	  vel0 = &vel[0];


	 //OOS-sim/Test20_2_2018/temp3 
	 double T_guess00[16]= {0.755756, -0.272366, 0.595525, 213.044047,
			 	 	 	 	 -0.008548, 0.905221, 0.424855, -202.064589,
			 	 	 	 	 -0.654798, -0.326178, 0.681798, 519.789040,
	      	 	 	 		0.000000, 0.000000, 0.000000, 1.000000};


	 //from adapt.txt from /DLR_ObjectTracker/DEOS_SIM/parameters>
	double TAdapt[16]= {0.998782, 0.036096, 0.033639, 4.999887,
			-0.034878 ,0.998739, -0.036096, 9.473143,
			-0.034899, 0.034878, 0.998782, 0.348889,
			0.000000, 0.000000, 0.000000, 1.000000};

		// final initial pose. T=T_guess*Tadapt
	//double T_guess0[16];
	matrixProduct444(T_guess00, TAdapt, T_guess0);


}


TEST_CASE( "Success Call to Configure (EdgeModelContourMatching)", "[configureDLRTracker]" ) 
	{
	EdgeModelContourMatching* contourMatching = new EdgeModelContourMatching ;
	contourMatching->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking");
	contourMatching->setup();
	std::cout<< " Tracker DFPC TEST SUCCEEDED-------------!"<<std::endl;
	
	delete contourMatching;
	
	}


TEST_CASE( "Success Call to Process (EdgeModelContourMatching)", "[processDLRTracker]")
	{
	cv::Mat inputImageLeft = cv::imread("../tests/Data/Images/DLR_OOS_sim_camL.png",0);
	cv::Mat inputImageRight = cv::imread("../tests/Data/Images/DLR_OOS_sim_camL.png",0);
	MatToFrameConverter matToFrame;
	
	FrameConstPtr inputFrameLeft = matToFrame.Convert(inputImageLeft);
	FrameConstPtr inputFrameRight = matToFrame.Convert(inputImageRight);
	// Instantiate DFPC
	EdgeModelContourMatching* contourMatching = new EdgeModelContourMatching ;
	//configurationfiles
	contourMatching->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking");
	//configure
	contourMatching->setup();	
	
	// Send input data to DFPC
	//Image:
	contourMatching->imageLeftInput(inputFrameLeft);
	contourMatching->imageRightInput(inputFrameRight);

	//time
	int frame_counter=1; //just one frame
	double dt_images= 0.1; //frame rate 10Hz
	double time_images = (double) frame_counter*dt_images;

	asn1SccTime imageAcquisitionTime;

	imageAcquisitionTime.microseconds = time_images*1000000;
	
	
	contourMatching->imageTimeInput(imageAcquisitionTime);

	//Initial pose:
	double T_guess0[16];
	double vel0[6];
        double rotTrasl[6];
	initPose(T_guess0,vel0);
	AngleAxisFromT(T_guess0, rotTrasl);

	asn1SccRigidBodyState initState;
	initState.orient.arr[0] = rotTrasl[0];
	initState.orient.arr[1] = rotTrasl[1];
 	initState.orient.arr[2] = rotTrasl[2];

	initState.pos.arr[0] = rotTrasl[3];
	initState.pos.arr[1] = rotTrasl[4];
 	initState.pos.arr[2] = rotTrasl[5];
	//Initial vel:
	initState.angular_velocity.arr[0] = vel0[0];
	initState.angular_velocity.arr[1] = vel0[1];
 	initState.angular_velocity.arr[2] = vel0[2];

	initState.velocity.arr[0] = vel0[3];
	initState.velocity.arr[1] = vel0[4];
 	initState.velocity.arr[2] = vel0[5];

	contourMatching->initInput(initState);
		
	contourMatching->run();
        //output to ASN
	bool success = contourMatching->successOutput();
	asn1SccRigidBodyState estimatedState = contourMatching->stateOutput();

	delete contourMatching;
	
	} 



/** @} */
