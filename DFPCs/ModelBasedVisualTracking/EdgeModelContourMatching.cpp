/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* --------------------------------------------------------------------------
*/

/*!
 * @file EdgeModelContourMatching.hpp
 * @date 30/05/2018
 * @author Nassir W. Oumer
 */

/*!
 * @addtogroup DFNs
 * 
 *   
 * Implementation of the EdgeModelContourMatching class.    
 *
 * @{
 */
/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "EdgeModelContourMatching.hpp"
#include "Errors/Assert.hpp"
#include <iostream>

using namespace Converters;
using namespace FrameWrapper;

namespace CDFF
{
namespace DFPC
{


/* --------------------------------------------------------------------------
 *
 * Public Member Functions
 *
 * --------------------------------------------------------------------------
 */
EdgeModelContourMatching::EdgeModelContourMatching()
	{

	img_color = NULL;

	#ifdef _DO_BIG_ALLOCATION
		           //Uchar,   UcharPtr, Double,  Float,   Int, IntPtr,  Short)	
		doBigMalloc(102400000, 1200000, 12000000, 200000, 5600000, 1400000, 3200000); 
	#endif
	matrixIdentity(T_egomotion, 4);
	
	configurationFilePath = "";
	}

EdgeModelContourMatching::~EdgeModelContourMatching()
	{

	 #ifdef _DO_BIG_ALLOCATION
		doBigFree();
	
	 #endif


	#ifdef _USE_OPENCV_DISPLAY
	  destroyAllWindows();
	#endif
	
	}
void EdgeModelContourMatching::allocateImageMemory()
	{
	 numberOfCameras = DLRTracker.getNcams();
	 xrmax = 0;
	 yrmax = 0;
    
	 for(int c=0; c< numberOfCameras; c++)
	 {
	  imgs[c] = myMallocUchar(DLRTracker.getXres(c)*DLRTracker.getYres(c)*sizeof(unsigned char));
	  xrmax = MAX(DLRTracker.getXres(c),xrmax);
	  yrmax = MAX(DLRTracker.getYres(c),yrmax);
	 }

	 img_color = myMallocUchar(3*xrmax*yrmax*sizeof(unsigned char));
	
	
	}


void EdgeModelContourMatching::run() 
	{

		
	ASSERT(numberOfCameras<3,"undefined camera: maximum 2 cameras ");
		
	for(int c=0; c<numberOfCameras; c++)
	 {	if(numberOfCameras == 1)
		{
		   cv::Mat inputLeftImage = frameToMat.Convert(inImageLeft); 
		   ASSERT(inputLeftImage.channels() == 1," unsupported image type: Tracker input is a gray scale image");
		   memcpy(imgs[c], inputLeftImage.data,DLRTracker.getXres(c)*DLRTracker.getYres(c)); 
			
		   break;
		}
		if(numberOfCameras ==2)
		{
		  if(c == 0)
		   {
	            cv::Mat inputLeftImage = frameToMat.Convert(inImageLeft); 
		    ASSERT(inputLeftImage.channels() == 1," unsupported image type: Tracker input is a gray scale image");
		    memcpy(imgs[c], inputLeftImage.data,DLRTracker.getXres(c)*DLRTracker.getYres(c)); 
                    }
		   if(c == 1)
	 	   { 
		    cv::Mat inputRightImage = frameToMat.Convert(inImageRight);
		    ASSERT(inputRightImage.channels() == 1," unsupported image type: Tracker input is a gray scale image");
		    memcpy(imgs[c], inputRightImage.data,DLRTracker.getXres(c)*DLRTracker.getYres(c)); 
		    }
		 }
		
		
	      }
	
	double time_images;
	//asn1Sccseconds in sec   
	 time_images = inImageTime.microseconds*0.000001;	 

	double T_guess0[16];
	double vel0[6];
	double rotTrasl[6];
	double time0;
	//in sec 
	time0 = inInitTime.microseconds*0.000001;	   

	bool useInitialGuess;
	useInitialGuess = inDoInit;

	ConvertAsnStateToState(inInit, rotTrasl, vel0);

	 TfromAngleAxis(rotTrasl, T_guess0);
		
 	double T_est[16];
	double vel_est[6];
	double ErrCov[6*6];
	//outputs for ASN:  outSuccess,outState
        outSuccess = edgeMatching(imgs, time_images, T_egomotion, T_guess0, vel0, time0, useInitialGuess, T_est, vel_est, ErrCov);

	//output: state estiamtes- T_est, vel_est
	AngleAxisFromT(T_est, rotTrasl);

	outState = ConvertStateToAsnState(rotTrasl, vel_est);

	
#ifdef _USE_OPENCV_DISPLAY

	std::string wname;
	int xres = DLRTracker.getXres(0); 
	int yres = DLRTracker.getYres(0);

	for(int c=0; c<numberOfCameras; c++)
	{	
	 DLRTracker.drawResult(T_est, img_color, c, true, 100);
	 sprintf((char*)wname.c_str(),"result%d",c);
	 createWindow(wname,1);
	 showImage(wname,img_color,xres,yres,3);
	}
		 

	char c;
	c = waitKey(100);
	if(c==27)
	 exit(EXIT_SUCCESS);
#endif



	}

void EdgeModelContourMatching::setup()
	{
	
	bool EPOS = false; //Temporary- please remove 
	double setup_global_array[_MAX_PARSE_ARRAY];
	int setup_global_array_counter;

	std::string file_path_string = configurator.configurePath(configurationFilePath);

        const char* pathTOSpecifications = file_path_string.c_str();
	std::cout<< " Path to config"<<file_path_string<<std::endl;
	
/*	if(EPOS)
	{
		if(parser.parseAllFiles(pathTOSpecifications, "camera_parameters_forrost_epos.txt", "tracker_parameters.txt", "theModel_client_Astriumv4.txt", setup_global_array_counter, 				setup_global_array)!=0)
			exit(-1);
		if( DLRTracker.setupFromGlobalArray(setup_global_array_counter, setup_global_array)!=0)
		exit(-1);
		
		DLRTracker.getObjectModel().changeLocalFrame(3); //EPOS
	} */
	//else
	//{

	 if(parser.parseAllFiles(pathTOSpecifications, "camera_parameters.txt", "tracker_parameters.txt", "theModel_client.txt", setup_global_array_counter, 				setup_global_array)!=0)
	  exit(-1);
	
	 if( DLRTracker.setupFromGlobalArray(setup_global_array_counter, setup_global_array)!=0)
	  exit(-1);

	 for(int bb = 3; bb <= 8; bb++) 
	  DLRTracker.getObjectModel().setActiveBody(bb, 0);

	 DLRTracker.getObjectModel().setActiveBody(12, 1); 

	 for(int bb = 13; bb < 19; bb++)
	  DLRTracker.getObjectModel().setActiveBody(bb, 0);

	 DLRTracker.getObjectModel().setActiveBody(13, 1);
	 DLRTracker.getObjectModel().setActiveBody(14, 1);
	 DLRTracker.getObjectModel().setActiveBody(15, 1);
	 DLRTracker.getObjectModel().setActiveBody(16, 1);
	 DLRTracker.getObjectModel().setActiveBody(17, 1);
	 DLRTracker.getObjectModel().setActiveBody(18, 0);

	 DLRTracker.getObjectModel().changeLocalFrame(1);

	//}	
	
    	allocateImageMemory();
	
	dumpMemoryAlloc();
	 
	}
	
bool EdgeModelContourMatching::edgeMatching(unsigned char** imgs, double time_images, double* T_egomotion, double* T_guess0, double* vel0, double time0, bool useInitialGuess, double* T_est, double* vel_est, double* ErrCov)
	{
	bool success;
	double degreesOfFreedom[6] = {1,1,1,1,1,1};
	bool timing=false;
	bool debug_show=false;
	bool diagnostics=false;

	status = DLRTracker.poseEstimation(imgs, time_images, T_egomotion, T_guess0, vel0, time0, T_est, vel_est, ErrCov, degreesOfFreedom, useInitialGuess, timing, debug_show, diagnostics);

	if(status==0)
	 	success = true;
	else
		success = false; 
	return success;
	}
void EdgeModelContourMatching::ConvertAsnStateToState(asn1SccRigidBodyState& state, double* pose, double* velocity)
	{
	 pose[0] = state.orient.arr[0];
	 pose[1] = state.orient.arr[1];
 	 pose[2] = state.orient.arr[2];

	 pose[3] = state.pos.arr[0];
	 pose[4] = state.pos.arr[1];
 	 pose[5] = state.pos.arr[2];
	
	 velocity[0] = state.angular_velocity.arr[0];
	 velocity[1] = state.angular_velocity.arr[1];
 	 velocity[2] = state.angular_velocity.arr[2];
	 velocity[3] = state.velocity.arr[3]; 
	 velocity[4] = state.velocity.arr[4]; 
	 velocity[5] = state.velocity.arr[5]; 
	 	
	}

asn1SccRigidBodyState EdgeModelContourMatching::ConvertStateToAsnState(double* pose, double* velocity)
	{
	asn1SccRigidBodyState asnState; 
	asnState.orient.arr[0] = pose[0];
	asnState.orient.arr[1] = pose[1];
 	asnState.orient.arr[2] = pose[2];

	asnState.pos.arr[0] = pose[3];
	asnState.pos.arr[1] = pose[4];
 	asnState.pos.arr[2] = pose[5];
	
	asnState.angular_velocity.arr[0] = velocity[0];
	asnState.angular_velocity.arr[1] = velocity[1];
 	asnState.angular_velocity.arr[2] = velocity[2];

	asnState.velocity.arr[0] = velocity[3];
	asnState.velocity.arr[1] = velocity[4];
 	asnState.velocity.arr[2] = velocity[5];
		 

	return asnState ;

	}

}
}


/** @} */
