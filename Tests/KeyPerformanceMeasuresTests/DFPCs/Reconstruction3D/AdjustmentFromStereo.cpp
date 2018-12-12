/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file AdjustmentFromStereo.cpp
 * @date 29/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 *
 * Validity Test 4.1.1.6 for DFPC implementation AdjustmentFromStereo.
 * "Resulting point cloud should be within the expected bounds of error described in D5.2", and
 * "Expected performance is no more than 10% outliers as estimated by a human inspecting the point cloud", and
 * "position estimation less than 1% of R, where R is the maximum operational distance of the camera/sensor", and
 * "90% similarity in shape to the object viewed with less than 10% error in dimensional analysis (only for components larger than 10% of the total size of the object)"
 *
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "ReconstructionExecutor.hpp"
#include <Reconstruction3D/AdjustmentFromStereo.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFPC;
using namespace CDFF::DFPC::Reconstruction3D;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
" \n \
The program has four usages depending on the first parameter: \n \
(i) 1st parameter is the mode of operation it can be one of (ComputePointCloud, EvaluateOutliers, EvaluateDistanceToCamera, EvaluateDimensions). The modes are explained one by one: \n \n \n \
Mode (a. ComputePointCloud) in this mode the stereo reconstruction dfn is executed and its result is saved to a ply file, you need 4 additional parameters: \n \
(a.ii) 2nd parameter is the configuration file path of the dfpc implementation RegistrationFromStereo; \n \
(a.iii) 3rd parameter is the folder containing the images list file; \n \
(a.iv) 4th is the image list file name (the file contains three blank lines, and then a triple for each line of the form: timeFloat pathToLeftImage pathToRightImage; \n \
(a.v) 5th is the output cloud file path, in ply format; \n \n \
Example usage: ./quality_adjustment_from_stereo ComputePointCloud ../tests/ConfigurationFiles/DFPCs/Reconstruction3D/DfpcRegistrationFromStereo_DevonIsland.yaml \
../tests/Data/Images ImagesList.txt ../tests/Data/PointClouds/Road.ply \n \n \n \
After you run (a), you should open the result with the tool DataGenerator/detect_outliers. Using the tool, you should detect those cloud points which should not appear in the scene. \
The result is used by the second step of the program to assess the quality of the point cloud. \n \n \
Example call: ./detect_outliers ../tests/Data/PointClouds/Road.ply ../tests/Data/PointClouds/RoadOutliers.xml \n \n \n \
Mode (b. EvaluateOutliers) the program will assess whether the data meets the outliers required quality. In this case the parameters have the following meaning: \n \
(b.ii) 2nd parameter is the point cloud file produced by the application of the dfn, it is in ply format. \n \
(b.iii) 3rd parameter is the file containing the outliers in xml format \n \
Optionally you can add one more parameter: \n \
(b.iv) 4th optional parameter is outliers percentage threshold for the quality check. It must be a number between 0 and 1; The default is 0.10. \n \n \
Example usage: ./quality_adjustment_from_stereo EvaluateOutliers ../tests/Data/PointClouds/Road.ply ../tests/Data/PointClouds/RoadOutliers.xml \n \n \n \
Mode (c. EvaluateDistanceToCamera) the program will assess whether the data meets the required quality in terms of distances to the camera. In this case the parameters have the following meaning: \n \
(c.ii) 2nd parameter is the point cloud file produced by the application of the dfn, it is in ply format. \n \
(c.iii) 3rd parameter is the file containing the measures of the distances in xml format \n \
(c.iv) 4th parameter is the camera maximum operational distance expressed in meters \n \
Optionally you can add one more parameter: \n \
(c.v) 5th optional parameter is the camera distance percentage error with respect to the operating distance. It must be a number betwee 0 and 1; The default is 0.01. \n \n \
Example usage: ./quality_adjustment_from_stereo EvaluateDistanceToCamera ../tests/Data/PointClouds/Road.ply ../tests/Data/PointClouds/RoadMeasures.xml 10 \n \n \n \
Mode (d. EvaluateDimensions) the program will assess whether the data meets the required quality in terms of objects dimensions. In this case the parameters have the following meaning: \n \
(b.ii) 2nd parameter is the point cloud file produced by the application of the dfn, it is in ply format. \n \
(b.iii) 3rd parameter is the file containing the measures of the distances in xml format \n \
Optionally you can add three more parameters: \n \
(b.iv) 4th optional parameter is the shape similarity threshold. It must be a number betwee 0 and 1; The default is 0.90. \n \
(b.v) 5th optional parameter is the dimensional error threshold. It must be a number betwee 0 and 1; The default is 0.10. \n \
(b.vi) 6th optional parameter is the theshold on the object component that determines how large a component should be with respect to the whole object in order to be considered in the evaluation of the dimensional error. It must be a number betwee 0 and 1; The default is 0.10. \n \n \
Example usage: ./quality_adjustment_from_stereo EvaluateDimensions ../tests/Data/PointClouds/Road.ply ../tests/Data/PointClouds/RoadMeasures.xml \n";

float ExtractOutliersPercentageThreshold(char* argument)
	{
	const std::string errorMessage = "The 4th parameter numberPercentageThreshold has to be a floating point number between 0 and 1";
	float outliersPercentageThreshold;

	try
		{
		outliersPercentageThreshold = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(outliersPercentageThreshold >= 0 && outliersPercentageThreshold <= 1, errorMessage);

	return outliersPercentageThreshold;
	}

float ExtractCameraOperationaDistance(char* argument)
	{
	const std::string errorMessage = "The 4th parameter camera maximum operational distance has to be a floating point greater than 0";
	float cameraOperationalDistance;

	try
		{
		cameraOperationalDistance = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(cameraOperationalDistance >= 0, errorMessage);

	return cameraOperationalDistance;
	}

float ExtractCameraDistanceError(char* argument)
	{
	const std::string errorMessage = "The 5th parameter cameraDistanceError has to be a floating point number between 0 and 1";
	float cameraDistanceError;

	try
		{
		cameraDistanceError = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(cameraDistanceError >= 0 && cameraDistanceError <= 1, errorMessage);

	return cameraDistanceError;
	}

float ExtractShapeSimilarityThreshold(char* argument)
	{
	const std::string errorMessage = "The 4th parameter shapeSimilarityThreshold has to be a floating point number between 0 and 1";
	float shapeSimilarityThreshold;

	try
		{
		shapeSimilarityThreshold = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(shapeSimilarityThreshold >= 0 && shapeSimilarityThreshold <= 1, errorMessage);

	return shapeSimilarityThreshold;
	}

float ExtractDimensionalErrorThreshold(char* argument)
	{
	const std::string errorMessage = "The 5th parameter dimensionalErrorThreshold has to be a floating point number between 0 and 1";
	float dimensionalErrorThreshold;

	try
		{
		dimensionalErrorThreshold = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(dimensionalErrorThreshold >= 0 && dimensionalErrorThreshold <= 1, errorMessage);

	return dimensionalErrorThreshold;
	}

float ExtractComponentSizeThreshold(char* argument)
	{
	const std::string errorMessage = "The 6th parameter componentSizeThreshold has to be a floating point number between 0 and 1";
	float componentSizeThreshold;

	try
		{
		componentSizeThreshold = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(componentSizeThreshold >= 0 && componentSizeThreshold <= 1, errorMessage);

	return componentSizeThreshold;
	}

int mainComputePointCloudMode(int argc, char** argv, Reconstruction3DInterface* dfpc)
	{
	std::string configurationFilePath;
	std::string inputImagesFolder;
	std::string inputImagesListFileName;
	std::string outputPointCloudFilePath;

	ASSERT(argc == 6, USAGE);
	configurationFilePath = argv[2];
	inputImagesFolder = argv[3];
	inputImagesListFileName = argv[4];
	outputPointCloudFilePath = argv[5];


	ReconstructionExecutor tester;
	tester.SetDfpc(configurationFilePath, dfpc);
	tester.SetInputFilesPaths(inputImagesFolder, inputImagesListFileName);
	tester.ExecuteDfpc();
	tester.SaveOutputPointCloud(outputPointCloudFilePath);

	return 0;
	}

int mainEvaluateOutliersMode(int argc, char** argv)
	{
	std::string outputPointCloudFilePath;
	std::string outliersReferenceFilePath;
	float outliersPercentageThreshold = 0.10;

	ASSERT(argc == 4 || argc == 5, USAGE);
	outputPointCloudFilePath = argv[2];
	outliersReferenceFilePath = argv[3];
	if (argc == 5)
		{
		outliersPercentageThreshold = ExtractOutliersPercentageThreshold(argv[4]);
		}

	ReconstructionExecutor tester;
	tester.SetOutputFilePath(outputPointCloudFilePath);
	tester.SetOutliersFilePath(outliersReferenceFilePath);
	bool success = tester.IsOutliersQualitySufficient(outliersPercentageThreshold);
	VERIFY_REQUIREMENT(success, "Outliers Quality of reconstructed cloud requirement 4.1.1.6 failed on the input images");

	return 0;
	}

int mainEvaluateDistanceToCamera(int argc, char** argv)
	{
	std::string outputPointCloudFilePath;
	std::string outliersMeasuresFilePath;
	float cameraOperationDistance;
	float cameraDistanceError = 0.01;

	ASSERT(argc == 5 || argc == 6, USAGE);
	outputPointCloudFilePath = argv[2];
	outliersMeasuresFilePath = argv[3];
	cameraOperationDistance = ExtractCameraOperationaDistance(argv[4]);
	if (argc == 6)
		{
		cameraDistanceError = ExtractCameraDistanceError(argv[5]);
		}

	ReconstructionExecutor tester;
	tester.SetOutputFilePath(outputPointCloudFilePath);
	tester.SetMeasuresFilePath(outliersMeasuresFilePath);
	bool success = tester.IsCameraDistanceQualitySufficient(cameraOperationDistance, cameraDistanceError);
	VERIFY_REQUIREMENT(success, "Camera Distance Quality of reconstructed cloud requirement 4.1.1.6 failed on the input images");

	return 0;
	}

int mainEvaluateDimensions(int argc, char** argv)
	{
	std::string outputPointCloudFilePath;
	std::string outliersMeasuresFilePath;
	float shapeSimilarityThreshold = 0.90;
	float dimensionalErrorThreshold = 0.10;
	float componentSizeThreshold = 0.10;

	ASSERT(argc >= 4 && argc <= 7, USAGE);
	outputPointCloudFilePath = argv[2];
	outliersMeasuresFilePath = argv[3];
	if (argc == 5)
		{
		shapeSimilarityThreshold = ExtractShapeSimilarityThreshold(argv[4]);
		}
	if (argc == 6)
		{
		dimensionalErrorThreshold = ExtractDimensionalErrorThreshold(argv[5]);
		}
	if (argc == 7)
		{
		componentSizeThreshold = ExtractComponentSizeThreshold(argv[6]);
		}

	ReconstructionExecutor tester;
	tester.SetOutputFilePath(outputPointCloudFilePath);
	tester.SetMeasuresFilePath(outliersMeasuresFilePath);
	bool success = tester.IsDimensionsQualitySufficient(shapeSimilarityThreshold, dimensionalErrorThreshold, componentSizeThreshold);
	VERIFY_REQUIREMENT(success, "Dimensions Quality of reconstructed cloud requirement 4.1.1.6 failed on the input images");

	return 0;
	}

int main(int argc, char** argv)
	{
	ASSERT(argc >= 2, USAGE);

	std::string mode = argv[1];
	if (mode == "ComputePointCloud")
		{
		AdjustmentFromStereo* registrationFromStereo = new AdjustmentFromStereo();
		return mainComputePointCloudMode(argc, argv, registrationFromStereo);
		}
	else if (mode == "EvaluateOutliers")
		{
		return mainEvaluateOutliersMode(argc, argv);
		}
	else if (mode == "EvaluateDistanceToCamera")
		{
		return mainEvaluateDistanceToCamera(argc, argv);
		}
	else if (mode == "EvaluateDimensions")
		{
		return mainEvaluateDimensions(argc, argv);
		}

	ASSERT(false, USAGE);
	return 0;
	}


/** @} */
