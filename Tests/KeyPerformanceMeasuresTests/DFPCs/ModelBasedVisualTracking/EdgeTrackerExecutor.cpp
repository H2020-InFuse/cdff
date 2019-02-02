#include "EdgeTrackerExecutor.hpp"
#include "EdgeTrackerUtility.hpp"
#include <ctime>
#include <typeinfo>

#include <ModelBasedVisualTracking/ModelBasedVisualTrackingInterface.hpp>

using namespace Converters;
using namespace FrameWrapper;
using namespace EdgeTrackerHelper;

#define DELETE_IF_NOT_NULL(pointer) \
	if (pointer != NULL) \
		{ \
		delete(pointer); \
		}

EdgeTrackerExecutor::EdgeTrackerExecutor() :
	dtImages(1.0),
	logGroundTruthError(false),
	frameConverter(),
	outputPose()
{
	inputLeftFrame = NULL;
	inputRightFrame = NULL;
	dfpc = NULL;

	inputImagesWereLoaded = false;
	outputPoseWasLoaded = false;
	dfpcExecuted = false;
	dfpcWasLoaded = false;
	outputSuccess =false;
}

EdgeTrackerExecutor::~EdgeTrackerExecutor()
{
	DELETE_IF_NOT_NULL(inputLeftFrame);
	DELETE_IF_NOT_NULL(inputRightFrame);
}

void EdgeTrackerExecutor::SetDfpc(std::string configurationFilePath, CDFF::DFPC::ModelBasedVisualTrackingInterface* dfpc)
{
	this->configurationFilePath = configurationFilePath;
	this->dfpc = dfpc;

	ConfigureDfpc();
	dfpcWasLoaded = true;
}

void EdgeTrackerExecutor::SetInputFilesPaths(std::string inputImagesFolder, std::string inputImagesListFileName, std::string inputPosesFolder, std::string inputPosesListFileName)
{
	this->inputImagesFolder = inputImagesFolder;
	this->inputImagesListFileName = inputImagesListFileName;
	this->inputPosesFolder = inputPosesFolder ;
	this->inputPosesListFileName = inputPosesListFileName;

	LoadInputImagesList();
	if (!inputPosesFolder.empty())
	{
		LoadInputPosesList();
	}
	inputImagesWereLoaded = true;
}

void EdgeTrackerExecutor::SetOutputFilePath(std::string outputPoseFilePath)
{
	this->outputPoseFilePath = outputPoseFilePath;
	outputPoseWasLoaded = true;
}

void EdgeTrackerExecutor::initPose(double* guessT0)
{
	// Add initial transf. matrix for your sequence if you dont use from file, e.g initial pose for InFuse sequence 20180913_163111
	double guessT00[16]= {
		0.014369, -0.997374, 0.070982, 1207.778952,
		0.433550, 0.070184, 0.898392, 296.516388,
		-0.901015, 0.017865, 0.433420, 1202.125120,
		0.000000, 0.000000, 0.000000, 1.000000};
	double AdaptT[16]= {
		1.0, 0.0, 0.0, 0.0,
		0.0 ,1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0};
	matrixProduct444(guessT00, AdaptT, guessT0);
}

void EdgeTrackerExecutor::initVelocity(double* velocity0)
{
	//  starts at  Rest
	double startVelocity[6] = {0.00, 0.00, 0.00, 0.00, 0.00, 0.00};
	double unitToradPerSecond=1;
	double unitTomillimeterPerSecond=1;
	for (int i = 0; i < 6; i++)
	{
		if(i < 3)
			velocity0[i] = startVelocity[i]*unitToradPerSecond;
		else
			velocity0[i] = startVelocity[i]*unitTomillimeterPerSecond;
	}

	}

void EdgeTrackerExecutor::ExecuteDfpc()
{
	ASSERT(inputImagesWereLoaded && dfpcWasLoaded, "Cannot execute DFPC, the input images or the DFPC itself are not loaded");
	ASSERT(leftImageFileNamesList.size() == rightImageFileNamesList.size(), "Left images list and right images list do not have same dimensions");

	double timeImages;
	asn1SccTime imageAcquisitionTime;
	//Input: initial time- usually set to 0;
	double initTime = 0;
	asn1SccTime initialTime;
	initialTime.microseconds = initTime;
	dfpc->initTimeInput(initialTime);

	//Input: Initial pose

	double vel0[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double orientationPosition[6];
	double guessT0[16];

	initPose(guessT0); //use from file instead

	double distanceInitial = fabs(guessT0[3])+fabs(guessT0[7])+fabs(guessT0[11]);
	ASSERT(distanceInitial > 500, "Initial position is greater than 500 mm ");
	AngleAxisFromT(guessT0, orientationPosition); //deg

	asn1SccRigidBodyState initState;

	initState = ConvertStateToAsnState(orientationPosition, vel0);
	// call an external DFPC here to initialize the tracker
	dfpc->initInput(initState);

	// EgoMotion of a Manipulator- zero if not available
	asn1SccRigidBodyState egoMotion;
	egoMotion.orient.arr[0] = 0.0;
	egoMotion.orient.arr[1] = 0.0;
	egoMotion.orient.arr[2] = 0.0;

	egoMotion.pos.arr[0] = 0.0;
	egoMotion.pos.arr[1] = 0.0;
	egoMotion.pos.arr[2] = 0.0;
	dfpc->egoMotionInput(egoMotion);

	int successCounter = 0;
	float processingTime = 0;

	if(isFileExist(outputPoseFilePath))
	{
		if(remove(outputPoseFilePath.c_str()) != 0 )
			perror( "Error deleting file" );
	}

	std::ofstream writer;
	double T_true[16];

	for(int imageIndex = 0; imageIndex < (int)leftImageFileNamesList.size(); imageIndex++)
	{
		std::stringstream leftImageFilePath, rightImageFilePath;
		leftImageFilePath << inputImagesFolder << "/" << leftImageFileNamesList.at(imageIndex);
		rightImageFilePath << inputImagesFolder << "/" << rightImageFileNamesList.at(imageIndex);

		LoadInputImage(leftImageFilePath.str(), inputLeftFrame);
		LoadInputImage(rightImageFilePath.str(), inputRightFrame);

		dfpc->imageLeftInput(*inputLeftFrame);
		dfpc->imageRightInput(*inputRightFrame);

		if(logGroundTruthError)
		{
			//------------------- Ground truth pose----------------------
			std::stringstream poseFilePath;
			poseFilePath << inputPosesFolder << "/" << poseFileNamesList.at(imageIndex);
			LoadInputPose(poseFilePath.str(),T_true);

			printMatrix(" ## ground truth matrix read from file: ",T_true,4,4);

			//init tracker
			AngleAxisFromT(T_true, orientationPosition); //deg, mm
			initState = ConvertStateToAsnState(orientationPosition, vel0);
			dfpc->initInput(initState);
		}
		else
		{
			//init tracker from row-ordered transforamtion matrix from file
			if(imageIndex == 0)
			{
				std::stringstream poseFilePath;
				poseFilePath << inputPosesFolder << "/" << poseFileNamesList.at(imageIndex);
				LoadInputPose(poseFilePath.str(),T_true);

				printMatrix(" ## Initial pose matrix from file: ",T_true,4,4);

				AngleAxisFromT(T_true, orientationPosition); //deg, mm
				initState = ConvertStateToAsnState(orientationPosition, vel0);
				dfpc->initInput(initState);
			}
		}

		/// Input: image time (image timestamp) ///
		timeImages = (double) imageIndex*dtImages;
		imageAcquisitionTime.microseconds = timeImages*1000000;
		dfpc->imageTimeInput(imageAcquisitionTime);

		if(imageIndex==0)
			dfpc->doInitInput(true);
			//useInitialGuess = true;

		if(imageIndex>0)
			dfpc->doInitInput(false);
			//useInitialGuess = false;

		clock_t startTime = clock();

		dfpc->run();

		clock_t endTime = clock();
		processingTime = float(endTime - startTime)*1000 / CLOCKS_PER_SEC;

		outputSuccess = dfpc->successOutput();
		if(!outputSuccess)
		{
			std::cout << "########## TRACKING-FRAME "<<imageIndex<<": NOT RELIABLE POSE ESTIMATE ! " << std::endl;
			// dfpc->doInitInput(true);	//if you have pose re-initializer
		}
		else
		{
			if(imageIndex>0)
			{
				// copy last pose, vel0 and time0 to initInPut to re-init in case of loss
				dfpc->initInput(dfpc->stateOutput());
				dfpc->initTimeInput(imageAcquisitionTime);
			}
			std::cout << "########## TRACKING-FRAME "<<imageIndex<<": SUCCESS " << std::endl;
		}

		//----output--
		if(outputSuccess)
			outputPose = dfpc->stateOutput();

		//Log poses to file
		writer.open(outputPoseFilePath.c_str(), std::ios::out | std::ios::app);

		if(logGroundTruthError)
			SaveOutputPose(writer, T_true);
		else
			SaveOutputPose(writer);

		writer.close();

		successCounter = (outputSuccess ? successCounter+1 : successCounter);
		std::cout<<"Processing time (ms): "<<processingTime<<std::endl;
	}

	std::cout<<" The tracking was successful for the given image sequence: "<<successCounter<<std::endl;
	dfpcExecuted = true;
}

void EdgeTrackerExecutor::SaveOutputPose(std::ofstream& writer, double* guessT0)
{
	double orientationPosition[6];
	double velocity[6];

	ConvertAsnStateToState(outputPose, orientationPosition, velocity);

	// Log to print output
	std::cout<<" Output states: [ rx  ry  rz  tx  ty  tz  wx  wy  wz  vx  vy  vz] \n";
	for (int i = 0;i<6;i++)
		std::cout<<orientationPosition[i]<<" ";
	for (int i = 0;i<6;i++)
		std::cout<<velocity[i]<<" ";

	std::cout<<std::endl;

	// Log groundtruth pose erorrs (Orientation [radian])
	if(logGroundTruthError)
	{
		double R_true[9];
		double t_true[3];
		rotTranslFromT(guessT0,  R_true, t_true);
		double R_est[9];
		double t_est[3];
		double r_est[3];
		for(int i = 0;i < 3; i++)
		{
			r_est[i] = orientationPosition[i]*M_PI/180.0;
			t_est[i] = orientationPosition[i+3];
		}

		matrixRvecToRmat(r_est, R_est);

		double R_estT[9];
		matrixTranspose( R_est, R_estT, 3,3);
		double dR[9];
		matrixProduct333(R_true,  R_estT, dR);
		double drho[3];
		matrixRmatToRvec(dR, drho); // [radian]
		double dtranslation[3];

		dtranslation[0] = t_true[0]-t_est[0];
		dtranslation[1] = t_true[1]-t_est[1];
		dtranslation[2] = t_true[2]-t_est[2];
		// Log to print output
		printMatrix(" ## Orientation error [radian] ",drho,3,1);
		printMatrix(" ## Position error [mm] ", dtranslation,3,1);
		// Log to file
		for (int i=0;i<3;i++)
		{
			writer<<drho[i];
			writer<<" ";
		}
		for (int i=0;i<3;i++)
		{
			writer<< dtranslation[i];
			writer<<" ";
		}
		for(int i=0;i<3;i++)
		{
			writer<< t_true[i];
			writer<<" ";
		}
		writer<<" \n ";
	}
	// Log the estimated pose (orientation [deg]) to file
	else
	{
		for (int i=0;i<6;i++)
		{
			writer<<orientationPosition[i]<<" ";
		}
		for (int i=0;i<6;i++)
		{
			writer<<velocity[i]<<" ";
		}
		writer<<"\n";
	}
}

void EdgeTrackerExecutor::LoadInputImage(std::string filePath, FrameWrapper::FrameConstPtr& frame )
{
	cv::Mat src_image = cv::imread(filePath, 0);
	assert(src_image.cols > 0 && src_image.rows >0 && "Error: Loaded input image is empty");
	cv::Mat image(src_image.rows, src_image.cols,CV_8UC1);
	filterMedian(src_image, image, 5);
	DELETE_IF_NOT_NULL(frame);
	frame= frameConverter.Convert(image);
}

void EdgeTrackerExecutor::LoadInputPose(std::string filePath, double* groundTruthT)
{
	std::ifstream pose;
	pose.open((char*)filePath.c_str(),std::ios::in);
	double val;
	for(int i=0;i<12;i++)
	{
		pose>>val;
		groundTruthT[i]= val;
	}

	groundTruthT[12]= 0;
	groundTruthT[13]= 0;
	groundTruthT[14]= 0;
	groundTruthT[15]= 1;

	pose.close();
}

void EdgeTrackerExecutor::LoadInputPosesList()
{
	std::stringstream posesListFilePath;
	posesListFilePath << inputPosesFolder << "/" << inputPosesListFileName;
	std::ifstream posesListFile(posesListFilePath.str().c_str());
	assert(posesListFile.good() && "Error it was not possible to open the poses list file");
	std::string line;
	std::getline(posesListFile, line);
	std::getline(posesListFile, line);
	std::getline(posesListFile, line);
	while (std::getline(posesListFile, line))
	{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));
		if(line.size()>0)
		{
			poseFileNamesList.push_back( std::string(stringsList.at(0)) );
		}
		else
			break;
	}
	posesListFile.close();
}


void EdgeTrackerExecutor::LoadInputImagesList()
{
	std::stringstream imagesListFilePath;
	imagesListFilePath << inputImagesFolder << "/" << inputImagesListFileName;
	std::ifstream imagesListFile(imagesListFilePath.str().c_str());
	assert(imagesListFile.good() && "Error it was not possible to open the images list file");
	std::string line;
	std::getline(imagesListFile, line);
	std::getline(imagesListFile, line);
	std::getline(imagesListFile, line);
	while (std::getline(imagesListFile, line))
	{
		std::vector<std::string> stringsList;
		boost::split(stringsList, line, boost::is_any_of(" "));

		if(line.size()>0)
		{
			leftImageFileNamesList.push_back( std::string(stringsList.at(1)) );
			rightImageFileNamesList.push_back( std::string(stringsList.at(2)) );
		}
		else
			break;
	}

	imagesListFile.close();
}

void EdgeTrackerExecutor::ConfigureDfpc()
{
	dfpc->setConfigurationFile(configurationFilePath);
	dfpc->setup();
}

void EdgeTrackerExecutor::filterMedian(cv::Mat& image, cv::Mat& filteredImage, int apertureSize)
{
	cv::medianBlur(image, filteredImage, apertureSize);
}

inline bool EdgeTrackerExecutor::isFileExist(const std::string& name)
{
	std::ifstream infile(name.c_str());
	return infile.good();
}

void EdgeTrackerExecutor::setState(asn1SccRigidBodyState& state, const double value)
{
	state.orient.arr[0] = 0.0;
	state.orient.arr[1] = 0.0;
	state.orient.arr[2] = 0.0;

	state.pos.arr[0] = 0.0;
	state.pos.arr[1] = 0.0;
	state.pos.arr[2] = 0.0;

	state.angular_velocity.arr[0] = value;
	state.angular_velocity.arr[1] = value;
	state.angular_velocity.arr[2] = value;
	state.velocity.arr[0] = value;
	state.velocity.arr[1] = value;
	state.velocity.arr[2] = value;
}

void EdgeTrackerExecutor::ConvertAsnStateToState(asn1SccRigidBodyState& state, double* pose, double* velocity)
{
	pose[0] = state.orient.arr[0];
	pose[1] = state.orient.arr[1];
	pose[2] = state.orient.arr[2];

	pose[3] = state.pos.arr[0];
	pose[4] = state.pos.arr[1];
	pose[5] = state.pos.arr[2];

	if(velocity != NULL)
	{
		velocity[0] = state.angular_velocity.arr[0];
		velocity[1] = state.angular_velocity.arr[1];
		velocity[2] = state.angular_velocity.arr[2];
		velocity[3] = state.velocity.arr[0];
		velocity[4] = state.velocity.arr[1];
		velocity[5] = state.velocity.arr[2];
	}
}

asn1SccRigidBodyState EdgeTrackerExecutor::ConvertStateToAsnState(double* pose, double* velocity)
{
	asn1SccRigidBodyState asnState;
	asnState.orient.arr[0] = pose[0];
	asnState.orient.arr[1] = pose[1];
	asnState.orient.arr[2] = pose[2];

	asnState.pos.arr[0] = pose[3];
	asnState.pos.arr[1] = pose[4];
	asnState.pos.arr[2] = pose[5];

	if(velocity != NULL)
	{
		asnState.angular_velocity.arr[0] = velocity[0];
		asnState.angular_velocity.arr[1] = velocity[1];
		asnState.angular_velocity.arr[2] = velocity[2];

		asnState.velocity.arr[0] = velocity[3];
		asnState.velocity.arr[1] = velocity[4];
		asnState.velocity.arr[2] = velocity[5];
	}

	return asnState ;
}
