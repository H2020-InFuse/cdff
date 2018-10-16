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
This program has two modes of operations, generate a camera poses file and generate a set of points clouds given by a model as viewed by a set of camera poses. The mode are selected by the first \
parameter which can be either CameraPoseGeneration or PointCloudGeneration. \n \n \
In camera generation mode, two parameters are needed: \n \
2. The index of the generated path; \n \
3. The path of the output file. \n \n \
Currently available indeces: \n \
0: the camera moves along a circle in the xz plane, and points toward the center of the coordinate system. \n \n \
In point cloud generation mode, four parameters are needed: \n \
2. a positive index of the model point cloud to use; \n \
3. the path to the file containing a list of imformation from extracting a point cloud: (each line has form 'x y z qx qy qz qw imagePlanDistance imagePlaneResolution imagePlaneSize displacementErrorMean displacementErrorStandardDeviation missingPatchErrorMean missingPatchErrorStandardDeviation viewPositionErrorMean viewPositionErrorStandardDeviation viewOrientationErrorMean viewOrientationStandardDeviation'). All noise errors are modelled as gaussian random variables; \n \
4. the path to the output folder; \
5. the output file name (in the output folder) that lists point cloud path and poses.  \n \n \
Example usage: ./simulate_stereo_cloud 0 ../tests/Data/PointClouds/ViewPoses.txt ../tests/Data/PointClouds/ output_clouds.txt\n \n \
Currently available indeces: \n \
0: the base point cloud is a cube rotated by 90 degrees around the x axis. \n \n ";

int ExtractIndex(char* argument)
	{
	static const std::string errorMessage = "The 1st parameter cloudIndex has to be a positive index";
	float index;

	try 
		{
		index = std::stoi(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(index >= 0, errorMessage);

	return index;
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

	outputFile << outputFileName << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl; //At the end of line, there is a pose for PointAssemblyTest.
	}

int main(int argc, char** argv)
	{
	ASSERT(argc >= 4, USAGE)
	std::string mode = argv[1];
	int index = ExtractIndex(argv[2]);

	if (mode == "CameraPoseGeneration")
		{
		StereoCloudSimulator::CreateCameraFile(index, argv[3]);
		return 0;
		}

	ASSERT(argc >= 6, USAGE);
	ASSERT( mode == "PointCloudGeneration", USAGE);
	std::string poseFile = argv[3];
	std::string outputFolder = argv[4];
	std::string outputFileName = argv[5];
	std::string cloudFile = outputFolder + "/" + outputFileName;
	
	StereoCloudSimulator simulator(index);
	std::ifstream inputFile(poseFile.c_str());
	std::ofstream outputFile(cloudFile.c_str());
	pcl::PLYWriter writer;
	while(inputFile.good())
		{
		RunSimulatorOnce(simulator, inputFile, outputFile, writer, index, outputFolder);
		}


	inputFile.close();
	outputFile.close();
	return 0;
	}

/** @} */
