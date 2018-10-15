/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file SimulateStereoCloud.cpp
 * @date 12/10/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This is the main program for simulating a stero cloud.
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
#include "StereoCloudSimulator.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>
#include <pcl/io/ply_io.h>
#include <fstream>

using namespace DataGenerators;
using namespace PoseWrapper;

const std::string USAGE =
" \n \
This program requires four inputs: \n \
1. a positive index of the model point cloud to use; \n \
2. the path to the file containing a list of imformation from extracting a point cloud: (each line has form 'x y z qx qy qz qw imagePlanDistance imagePlaneResolution imagePlaneSize displacementErrorMean displacementErrorStandardDeviation missingPatchErrorMean missingPatchErrorStandardDeviation'); \n \
3. the path to the output folder; \
4. the output file name (in the output folder) that lists point cloud path and poses.  \n \n \
Example usage: ./simulate_stereo_cloud 0 ../tests/Data/PointClouds/ViewPoses.txt ../tests/Data/PointClouds/ \n \n";

int ExtractCloudIndex(char* argument)
	{
	static const std::string errorMessage = "The 1st parameter cloudIndex has to be a positive index";
	float cloudIndex;

	try 
		{
		cloudIndex = std::stoi(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(cloudIndex >= 0, errorMessage);

	return cloudIndex;
	}

void RunSimulatorOnce(StereoCloudSimulator& simulator, std::ifstream& inputFile, std::ofstream& outputFile, pcl::PLYWriter& writer, int cloudIndex, const std::string& outputFolder)
	{
	static int counter = 0;
	double x, y, z, qx, qy, qz, qw, roll, pitch, yaw;
	double imagePlanDistance, imagePlaneResolution, imagePlaneSize;
	double displacementErrorMean, displacementErrorStandardDeviation;
	double missingPatchErrorMean, missingPatchErrorStandardDeviation;
	double viewPositionErrorMean, viewPositionErrorStandardDeviation, viewOrientationErrorMean, viewOrientationStandardDeviation;

	inputFile >> x;
	inputFile >> y;
	inputFile >> z;
	inputFile >> roll;
	inputFile >> pitch;
	inputFile >> yaw;
	inputFile >> imagePlanDistance;
	inputFile >> imagePlaneResolution;
	inputFile >> imagePlaneSize;
	inputFile >> displacementErrorMean;
	inputFile >> displacementErrorStandardDeviation;
	inputFile >> missingPatchErrorMean;
	inputFile >> missingPatchErrorStandardDeviation;
	inputFile >> viewPositionErrorMean;
	inputFile >> viewPositionErrorStandardDeviation;
	inputFile >> viewOrientationErrorMean;
	inputFile >> viewOrientationStandardDeviation;	
	if (!inputFile.good())
		{
		return;
		}

	StereoCloudSimulator::EulerAnglesToQuaternion(roll, pitch, yaw, qx, qy, qz, qw);
	Pose3D viewPose;
	SetPosition(viewPose, x, y, z);
	SetOrientation(viewPose, qx, qy, qz, qw);
	simulator.SetCamera(viewPose, imagePlanDistance, imagePlaneResolution, imagePlaneSize);
	simulator.SetDisplacementNoiseModel(displacementErrorMean, displacementErrorStandardDeviation);
	simulator.SetMissingPatchNoiseModel(missingPatchErrorMean, missingPatchErrorStandardDeviation);

	std::cout << "Computing first cloud with data (" << x <<", " << y <<", " << z <<") (" << qx << ", " << qy << ", " << qz << ", " << qw << ")" << std::endl;  
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloud = simulator.ComputePointCloud();

	std::string outputFileName = "cloud_" + std::to_string( cloudIndex ) + "_" + std::to_string(counter) + ".ply";
	std::string outputFilePath = outputFolder + "/" + outputFileName;
	counter++;
	writer.write(outputFilePath, *pointCloud, true);		

	outputFile << outputFileName << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl; //There is a pose as required by the PointAssemblyTest.
	}

void PrintViewPoses(std::string outputFile)
	{
	const double resolution = 0.1;
	const double radius = 2;
	std::ofstream file(outputFile.c_str());

	const double planeDistance = 1;
	const double planeResolution = 0.001;
	const double planSize = 0.1;
	const double displacementErrorMean = 0;
	const double displacementErrorStandardDeviation = 0;
	const double missingPatchErrorMean = 0;
	const double missingPatchErrorStandardDeviation = 0;
	const double viewPositionErrorMean = 0;
	const double viewPositionErrorStandardDeviation = 0;
	const double viewOrientationErrorMean = 0;
	const double viewOrientationStandardDeviation = 0;
	
	for(double angle = 0; angle < 2*M_PI; angle += resolution)
		{
		double x = radius*std::cos(angle);
		double z = radius*std::sin(angle);
		double y = 0.5;

		double roll = std::atan2(y, radius);
		double pitch = M_PI/2 + angle;
		double yaw = 0;

		file << x << " ";
		file << y << " ";
		file << z << " ";
		file << std::setprecision(13);
		file << roll << " ";
		file << pitch << " ";
		file << std::setprecision(5);
		file << yaw << " ";
		file << planeDistance << " ";
		file << planeResolution << " ";
		file << planSize << " ";
		file << displacementErrorMean << " ";
		file << displacementErrorStandardDeviation << " ";
		file << missingPatchErrorMean << " ";
		file << missingPatchErrorStandardDeviation << " ";
		file << viewPositionErrorMean << " ";
		file << viewPositionErrorStandardDeviation << " ";
		file << viewOrientationErrorMean << " ";
		file << viewOrientationStandardDeviation << " ";
		file << std::endl;
		}

	file.close();
	}

int main(int argc, char** argv)
	{
	PrintViewPoses("/Agridrive1/DLR/Synthetic/ViewPoses.txt");
	ASSERT(argc >= 5, USAGE);
	int cloudIndex = ExtractCloudIndex(argv[1]);
	std::string poseFile = argv[2];
	std::string outputFolder = argv[3];
	std::string outputFileName = argv[4];
	std::string cloudFile = outputFolder + "/" + outputFileName;
	
	StereoCloudSimulator simulator(cloudIndex);
	std::ifstream inputFile(poseFile.c_str());
	std::ofstream outputFile(cloudFile.c_str());
	pcl::PLYWriter writer;
	while(inputFile.good())
		{
		RunSimulatorOnce(simulator, inputFile, outputFile, writer, cloudIndex, outputFolder);
		}


	inputFile.close();
	outputFile.close();
	return 0;
	}

/** @} */
