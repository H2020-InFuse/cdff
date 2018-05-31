/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file IcpCC.cpp
 * @date 31/05/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Correctness Test for DFN implementation Registration3D IcpCC.
 * " The matched model within the scene should be located and oriented within 10% of the size and relative orientation of the model. " and
 * " Distance between ground truth pose and estimate pose should not be higher than 5% of the operating distance."
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "CorrectLocalizationTester.hpp"
#include <Registration3D/IcpCC.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
" \n \
You should provide at least five parameters: \n \
(i) the configuration file path of Icp 3D; \n \
(ii) the scene input cloud file path, the input cloud file should be in ply format; \n \
(iii) the model input cloud file path, the input cloud file should be in ply format; \n \
(iv) the text file containing the ground truth pose on one line with format x y z qx qy qz qw \n \
(v) the operating distance expressed in meters; \n \
Optionally you can add up to four parameters: \n \
(i) the text file containing the initual guess pose on one line with format x y z qx qy qz qw \n \
(ii) relativeLocationError: the maximum position error expressed as ratio to the size of the model, it needs to be in the interval [0,1]. The default is 0.10; \n \
(iii) relativeOrientationError: the maximum orientation error expressed as ratio to the size of the model, it needs to be in the interval [0,1]. The default is 0.10; \n \
(iv) errorRatioComparedToOperatingDistance: the maximum position error expressed as ratio to the operating distance. The default is 0.05. \n \n \
Example Usage: ./registration_3d_icp_cc_test ../tests/ConfigurationFiles/DFNs/Registration3D/IcpCC_DevonIsland.yaml ../tests/Data/PointClouds/DevonIslandRoad.ply ../tests/Data/PointClouds/DevonIslandRoadTransformed.ply ../tests/Data/PointClouds/DevonIslandRoadTransform.txt 20 ../tests/Data/PointClouds/DevonIslandRoadTransformGuess.txt\n \n";


float ExtractOperatingDistance(char* argument)
	{
	static const std::string errorMessage = "The 5th parameter operatingDistance has to be a positive floating point number";
	float operatingDistance;

	try 
		{
		operatingDistance = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(operatingDistance > 0, errorMessage);

	return operatingDistance;
	}

float ExtractRelativeLocationError(char* argument)
	{
	static const std::string errorMessage = "The 7th parameter relativeLocationError has to be a positive floating between 0 and 1";
	float relativeLocationError;

	try 
		{
		relativeLocationError = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(relativeLocationError >= 0 && relativeLocationError <= 1, errorMessage);

	return relativeLocationError;
	}

float ExtractRelativeOrientationError(char* argument)
	{
	static const std::string errorMessage = "The 8th parameter relativeOrientationError has to be a positive floating between 0 and 1";
	float relativeOrientationError;

	try 
		{
		relativeOrientationError = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(relativeOrientationError >= 0 && relativeOrientationError <= 1, errorMessage);

	return relativeOrientationError;
	}

float ExtractErrorRatio(char* argument)
	{
	static const std::string errorMessage = "The 9th parameter errorRatioComparedToOperatingDistance has to be a positive floating between 0 and 1";
	float errorRatioComparedToOperatingDistance;

	try 
		{
		errorRatioComparedToOperatingDistance = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(errorRatioComparedToOperatingDistance >= 0 && errorRatioComparedToOperatingDistance <= 1, errorMessage);

	return errorRatioComparedToOperatingDistance;
	}

int main(int argc, char** argv)
	{
	std::string icpConfigurationFilePath;
	std::string sceneFilePath, modelFilePath, groundTruthPoseFilePath;
	std::string guessPoseFilePath;
	bool useGuessPose;
	float operatingDistance;
	float relativeLocationError = 0.10;
	float relativeOrientationError = 0.10;
	float errorRatioComparedToOperatingDistance = 0.05;

	ASSERT(argc >= 6, USAGE);
	icpConfigurationFilePath = argv[1];
	sceneFilePath = argv[2];
	modelFilePath = argv[3];
	groundTruthPoseFilePath = argv[4];
	operatingDistance = ExtractOperatingDistance(argv[5]);
	useGuessPose = (argc >= 7);

	if (argc >= 7)
		{
		guessPoseFilePath = argv[6];
		}
	if (argc >= 8)
		{
		relativeLocationError = ExtractRelativeLocationError(argv[7]);
		}
	if (argc >= 9)
		{
		relativeLocationError = ExtractRelativeOrientationError(argv[8]);
		}
	if (argc >= 10)
		{
		relativeLocationError = ExtractErrorRatio(argv[9]);
		}
	 
	IcpCC* icp = new IcpCC();

	CorrectLocalizationTester tester(icpConfigurationFilePath, icp);
	tester.SetInputClouds(sceneFilePath, modelFilePath, groundTruthPoseFilePath);
	if (useGuessPose)
		{
		tester.SetGuessModelPoseInScene(guessPoseFilePath);
		}

	tester.ExecuteDfn();
	float absoluteLocationError = operatingDistance * errorRatioComparedToOperatingDistance;
	bool success = tester.IsOutputCorrect(relativeLocationError, relativeOrientationError, absoluteLocationError);

	VERIFY_REQUIREMENT(success, "Correctness requirement 4.1.1.9 failed on the input scene and model point cloud");
	return 0;
	}


/** @} */
