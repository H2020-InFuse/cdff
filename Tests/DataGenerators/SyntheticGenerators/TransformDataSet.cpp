/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file TransformDataSet.cpp
 * @date 24/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * This program is used to convert Dlr Dataset into a format usable by the performance tests.
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
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>

const std::string IMAGE_EXTENSION = ".png";
const std::string POSE_EXTENSION = ".coords";
const std::string LEFT_EXTENSION = ".0";
const std::string RIGHT_EXTENSION = ".1";
const std::string BASE_IMAGE_NAME = "pic";
const std::string IMAGES_LIST_FILE_NAME = "ImagesList.txt";
const std::string POSES_LIST_FILE_NAME ="PosesList.txt";

std::string NumberToString(unsigned number)
	{
	std::stringstream numberStream;
	if (number < 100)
		{
		numberStream << "0";
		}
	if (number < 10)
		{
		numberStream << "0";
		}
	numberStream << number;
	return numberStream.str();
	}

void SavePose(std::ifstream& leftPoseFile, unsigned number, std::ofstream& posesListFile)
	{
	float rotation[3][3];
	float translation[3];

	leftPoseFile >> rotation[0][0];
	leftPoseFile >> rotation[0][1];
	leftPoseFile >> rotation[0][2];
	leftPoseFile >> translation[0];
	leftPoseFile >> rotation[1][0];
	leftPoseFile >> rotation[1][1];
	leftPoseFile >> rotation[1][2];
	leftPoseFile >> translation[1];
	leftPoseFile >> rotation[2][0];
	leftPoseFile >> rotation[2][1];
	leftPoseFile >> rotation[2][2];
	leftPoseFile >> translation[2];

	float quaternionW = std::sqrt(1.0 + rotation[0][0] + rotation[1][1] + rotation[2][2]) / 2.0;
	float quaternionX = (rotation[2][1] - rotation[1][2]) / (4*quaternionW);
	float quaternionY = (rotation[0][2] - rotation[2][0]) / (4*quaternionW);
	float quaternionZ = (rotation[1][0] - rotation[0][1]) / (4*quaternionW);

	posesListFile << (0.1 * (float)number) << " ";
	posesListFile << (translation[0] * 1e-3) << " " << (translation[1] * 1e-3) << " " << (translation[2] * 1e-3) << " ";
	posesListFile << quaternionX << " " << quaternionY << " " << quaternionZ << " " << quaternionW << std::endl;
	}

void SaveImage(std::string innerImageFolder, unsigned number, std::ofstream& imagesListFile)
	{
	imagesListFile << (0.1 * (float)number) << " " << std::setprecision(7);
	imagesListFile << innerImageFolder << "/" << BASE_IMAGE_NAME << NumberToString(number) << LEFT_EXTENSION << IMAGE_EXTENSION << " ";
	imagesListFile << innerImageFolder << "/" << BASE_IMAGE_NAME << NumberToString(number) << RIGHT_EXTENSION << IMAGE_EXTENSION << std::endl;
	}

int main(int argc, char** argv)
	{
	ASSERT(argc == 3, "Error: 2 parameters are required, datasetPath and innerImageFolder");
	std::string datasetPath = argv[1];
	std::string innerImageFolder = argv[2];

	std::stringstream imagesListFilePath, posesListFilePath;
	imagesListFilePath << datasetPath << "/" << IMAGES_LIST_FILE_NAME;
	posesListFilePath << datasetPath << "/" << POSES_LIST_FILE_NAME;
	
	std::ofstream posesListFile(posesListFilePath.str().c_str());
	std::ofstream imagesListFile(imagesListFilePath.str().c_str());

	ASSERT(posesListFile.good(), "posesListFile could not be opened");
	ASSERT(imagesListFile.good(), "imagesListFile could not be opened");
	imagesListFile << "#\n#\n#\n";
	posesListFile << "#\n#\n#\n";

	bool moreFilesAvailable = true;
	unsigned number = 0;

	while (moreFilesAvailable)
		{
		std::stringstream leftPoseFilePath, rightPoseFilePath;
		leftPoseFilePath << datasetPath << "/" << innerImageFolder << "/" << BASE_IMAGE_NAME << NumberToString(number) << LEFT_EXTENSION << POSE_EXTENSION;
		rightPoseFilePath << datasetPath << "/" << innerImageFolder << "/" << BASE_IMAGE_NAME << NumberToString(number) << RIGHT_EXTENSION << POSE_EXTENSION;

		std::ifstream leftPoseFile(leftPoseFilePath.str().c_str());
		std::ifstream rightPoseFile(rightPoseFilePath.str().c_str());
		moreFilesAvailable = leftPoseFile.good() && rightPoseFile.good();

		if (moreFilesAvailable)
			{
			SavePose(leftPoseFile, number, posesListFile);
			SaveImage(innerImageFolder, number, imagesListFile);

			leftPoseFile.close();
			rightPoseFile.close();
			number++;
			}
		}
	

	posesListFile << (0.1 * (float)number) << " ";
	posesListFile << 0 << " " << 0 << " " << 0 << " ";
	posesListFile << 0 << " " << 0 << " " << 0 << " " << 1 << std::endl;

	posesListFile.close();
	imagesListFile.close();
	
	return 0;
	}

/** @} */
