/**
 * @author Nassir W. Oumer
 */

/**
 * Validity Test 4.1? for DFPC implementation EdgeModelContourMatching.
 */

#include "EdgeTrackerExecutor.hpp"

#include <ModelBasedVisualTracking/EdgeModelContourMatching.hpp>
#include <Errors/Assert.hpp>
#include <Types/C/Time.h>
#include <stdlib.h>
#include <boost/make_shared.hpp>
#include <boost/algorithm/string.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace CDFF::DFPC::ModelBasedVisualTracking;

const std::string USAGE = " \n \
The program has two usages depending on avaialbility of ground truth poses: \n \
(1) Without GroundTruth- initial pose should be provided from either initPose method or in the form of (2) to simulate an external DFPC: \n \n \
    (1a) 1st parameter is the configuration file path of the dfpc implementation; \n \
    (1b) 2nd parameter is the directory which connsists of the images list file; \n \
    (1c) 3rd parameter is the image list file name (the file contains three blank lines, and then a triple for each line of the form: time pathToLeftImage \n 		pathToRightImage; \n \
    (1d) 4th parameter is the output pose file path; \n \n \
Example: ./validation_DLR_model_tracker ../../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking ../../tests/Data/Images/Sequences ImagesList.txt ../../tests/Data/Images/Sequences/pose.txt \n \n \n \
(2) With groundtruth. logGroundTruthError must be true. In this case the parameters are defined as: \n \
    (1a) 1st parameter is the configuration file path of the dfpc implementation; \n \
    (1b) 2nd parameter is the directory which connsists of the images list file; \n \
    (1c) 3rd parameter is the image list file name (the file contains three blank lines, and then a triple for each line of the form: timeFloat pathToLeftImage\n  		pathToRightImage; \n \
    (1d) 4rd parameter is the ground truth file path \n \
    (1e) 5th parameter is the ground truth pose list file name (the file contains three blank lines, and then for each line of the form: timeFloat pathToPose ; \n \
    (1f) 6th parameter  is the output pose file path; \n \n \
Example: ./validation_DLR_model_tracker ../../tests/ConfigurationFiles/DFPCs/ModelBasedVisualTracking ../../tests/Data/Images/Sequences ImagesList.txt ../../tests/Data/\n  Images/Sequences  PosesList.txt ../../tests/Data/Images/Sequences/pose.txt \n";

int maindfpcTest(int argc, char* argv[], CDFF::DFPC::ModelBasedVisualTrackingInterface* dfpc)
{
	std::string configurationFilePath;
	std::string inputImagesFolder;
	std::string inputImagesListFileName;
	std::string outputPoseFilePath;

	std::string inputPosesFolder;
	std::string inputPosesListFileName;
	ASSERT(argc == 5 || argc == 7, USAGE);
	if(argc==5)
	{
		configurationFilePath = argv[1];
		inputImagesFolder = argv[2];
		inputImagesListFileName = argv[3];
		outputPoseFilePath = argv[4];
	}
	if(argc==7)
	{
		configurationFilePath = argv[1];
		inputImagesFolder = argv[2];
		inputImagesListFileName = argv[3];
		inputPosesFolder = argv[4];
		inputPosesListFileName = argv[5];
		outputPoseFilePath = argv[6];
	}

	EdgeTrackerExecutor* myexecutor = new EdgeTrackerExecutor;
	myexecutor->dtImages = 0.33; //sampling time
	myexecutor->SetDfpc(configurationFilePath, dfpc);
	//init tracker or log grondtruth error, by turning logGroundTruthError flag off or on
	if(argc==7)
	{
		myexecutor->logGroundTruthError = false;
		myexecutor->SetInputFilesPaths(inputImagesFolder, inputImagesListFileName, inputPosesFolder, inputPosesListFileName);
	}
	else
		myexecutor->SetInputFilesPaths(inputImagesFolder, inputImagesListFileName);

	myexecutor->SetOutputFilePath(outputPoseFilePath);
	myexecutor->ExecuteDfpc();
	delete myexecutor;
	return 0;
}

int main(int argc, char* argv[])
{
	EdgeModelContourMatching* modelTracker = new EdgeModelContourMatching();

	return maindfpcTest(argc,argv,modelTracker);
	std::cout << " Testing performance ... \n";
	delete modelTracker;

	return 0;
}
