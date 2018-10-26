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
#include <iostream>

using namespace DLRtracker;
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

void initPose(double* guessT0, double* velocity0 )
{
	 double  velocity[6] = {0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000};
	 velocity0 = &velocity[0];
	 double guessT00[16]= {0.014377, -0.997371, 0.071024, 1207.899488,
				 0.433600, 0.070224, 0.898365, 296.456254,
				 -0.900991, 0.017880, 0.433469, 1202.087847,
	      	 	 	  0.000000, 0.000000, 0.000000, 1.000000};
	double TAdapt[16]= {1.0, 0.0, 0.0, 0.0,
			    0.0, 1.0, 0.0, 0.0,
			    0.0, 0.0, 1.0, 0.0,
			    0.0, 0.0, 0.0, 1.0};

	matrixProduct444(guessT00, TAdapt, guessT0);
}



TEST_CASE( "Success Call to Configure (EdgeModelContourMatching)", "[configureDLRTracker]" ) 
	{
	EdgeModelContourMatching* contourMatching = new EdgeModelContourMatching() ;
	contourMatching->setConfigurationFile("../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking");
	contourMatching->setup();
	std::cout<< " Tracker DFPC- Setup Functionality Test success "<<std::endl;
		
	delete contourMatching;

	
	}



TEST_CASE( "Success Call to Process (EdgeModelContourMatching)", "[processDLRTracker]")
	{
	//source image type- ASN frame, but here cv::Mat. Conversions: cv::Mat --> ASN Frame

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
		
	//Image:
	contourMatching->imageLeftInput(inputFrameLeft);
	contourMatching->imageRightInput(inputFrameRight);

	//time
	int frameCounter = 1; //just one frame
	double dtImages = 0.33; //frame rate 3Hz
	double timeImages = (double) frameCounter*dtImages;

	asn1SccTime imageAcquisitionTime;

	imageAcquisitionTime.microseconds = timeImages*1000000;
		
	contourMatching->imageTimeInput(imageAcquisitionTime);

	//Initial pose:
	double guessT0[16];
	double velocity0[6];
        double rotationTranslation[6];
	initPose(guessT0,velocity0);
	AngleAxisFromT(guessT0, rotationTranslation);

	asn1SccRigidBodyState initState;
	initState.orient.arr[0] = rotationTranslation[0];
	initState.orient.arr[1] = rotationTranslation[1];
 	initState.orient.arr[2] = rotationTranslation[2];

	initState.pos.arr[0] = rotationTranslation[3];
	initState.pos.arr[1] = rotationTranslation[4];
 	initState.pos.arr[2] = rotationTranslation[5];
	//Initial velocity:
	initState.angular_velocity.arr[0] = velocity0[0];
	initState.angular_velocity.arr[1] = velocity0[1];
 	initState.angular_velocity.arr[2] = velocity0[2];

	initState.velocity.arr[0] = velocity0[3];
	initState.velocity.arr[1] = velocity0[4];
 	initState.velocity.arr[2] = velocity0[5];

	contourMatching->initInput(initState);
		
	contourMatching->run();
        //output to ASN
	bool success = contourMatching->successOutput();
	asn1SccRigidBodyState estimatedState = contourMatching->stateOutput();
	std::cout<< " Tracker DFPC- Process Functionality Test success "<<std::endl;
	
	delete contourMatching;
	
	} 


/** @} */
