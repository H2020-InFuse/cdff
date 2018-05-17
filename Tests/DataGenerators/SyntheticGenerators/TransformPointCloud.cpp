/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file TransformPointCloud.cpp
 * @date 23/04/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DataGenerators
 * 
 * Main files for the transformation of point clouds.
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
#include "PointCloudTransformer.hpp"
#include <Errors/Assert.hpp>
#include <stdlib.h>
#include <iostream>

using namespace DataGenerators;

int main(int argc, char** argv)
	{
	std::string pointCloudFilePath = "../tests/Data/PointClouds/bunny0.ply";
	if (argc >= 2)
		{
		pointCloudFilePath = argv[1];
		}

	PointCloudTransformer pointCloudTransformer;
	PRINT_TO_LOG("Loading Point Cloud", "");
	pointCloudTransformer.LoadPointCloud(pointCloudFilePath);

	std::string command = "";
	while( command != "quit" )
		{
		PRINT_TO_LOG("Insert Command: ", "");
		std::cin >> command;
		if (command == "resize")
			{
			unsigned minIndex, maxIndex;
			std::cin >> minIndex;
			std::cin >> maxIndex;
			PRINT_TO_LOG("Resizing Point Cloud", minIndex);
			pointCloudTransformer.Resize(minIndex, maxIndex);			
			}
		else if (command == "transform")
			{
			float positionX, positionY, positionZ, rotationX, rotationY, rotationZ, rotationW;
			std::cin >> positionX;
			std::cin >> positionY;
			std::cin >> positionZ;
			std::cin >> rotationX;
			std::cin >> rotationY;
			std::cin >> rotationZ;
			std::cin >> rotationW;	
			PRINT_TO_LOG("Applying Transform", positionX)
			pointCloudTransformer.TransformCloud(positionX, positionY, positionZ, rotationX, rotationY, rotationZ, rotationW);		
			}
		else if (command == "camera_transform")
			{
			float positionX, positionY, positionZ, rotationX, rotationY, rotationZ, rotationW;
			std::cin >> positionX;
			std::cin >> positionY;
			std::cin >> positionZ;
			std::cin >> rotationX;
			std::cin >> rotationY;
			std::cin >> rotationZ;
			std::cin >> rotationW;	
			PRINT_TO_LOG("Applying Transform", positionX)
			pointCloudTransformer.TransformCamera(positionX, positionY, positionZ, rotationX, rotationY, rotationZ, rotationW);
			}
		else if (command == "noise")
			{
			float mean, standardDeviation;
			std::cin >> mean;
			std::cin >> standardDeviation;
			PRINT_TO_LOG("Adding Noise", "");
			pointCloudTransformer.AddGaussianNoise(mean, standardDeviation);
			}
		else if (command == "save")
			{
			std::string outputFile;
			std::cin >> outputFile;
			PRINT_TO_LOG("Saving Point Cloud", "");
			pointCloudTransformer.SavePointCloud(outputFile);			
			}
		else if (command == "view")
			{
			pointCloudTransformer.ViewPointCloud();
			}
		}

	return 0;
	}

/** @} */
