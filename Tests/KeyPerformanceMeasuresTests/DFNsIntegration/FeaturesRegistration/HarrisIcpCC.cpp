/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisIcpCC.cpp
 * @date 05/06/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Correctness Test for DFN implementation Registration3D IcpCC on the features identified by DFN implementation FeaturesExtraction3D HarrisDetector3D.
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
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
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
You should provide at least six parameters: \n \
(i) the configuration file path of Harris 3D: \n \
(ii) the configuration file path of Icp 3D; \n \
(iii) the scene input cloud file path, the input cloud file should be in ply format; \n \
(iv) the model input cloud file path, the input cloud file should be in ply format; \n \
(v) the text file containing the ground truth pose on one line with format x y z qx qy qz qw \n \
(vi) the operating distance expressed in meters; \n \
Optionally you can add up to four parameters: \n \
(i) the text file containing the initual guess pose on one line with format x y z qx qy qz qw \n \
(ii) relativeLocationError: the maximum position error expressed as ratio to the size of the model, it needs to be in the interval [0,1]. The default is 0.10; \n \
(iii) relativeOrientationError: the maximum orientation error expressed as ratio to the size of the model, it needs to be in the interval [0,1]. The default is 0.10; \n \
(iv) errorRatioComparedToOperatingDistance: the maximum position error expressed as ratio to the operating distance. The default is 0.05. \n \n \
Example Usage: ./harris_icp_cc_test ../tests/ConfigurationFiles/DFNs/FeaturesExtraction/HarrisDetector3D_DevonIsland.yaml ../tests/ConfigurationFiles/DFNs/Registration3D/IcpCC_DevonIsland.yaml ../tests/Data/PointClouds/DevonIslandRoad.ply ../tests/Data/PointClouds/DevonIslandRoadTransformed.ply ../tests/Data/PointClouds/DevonIslandRoadTransform.txt 20 ../tests/Data/PointClouds/DevonIslandRoadTransformGuess.txt\n \n";


float ExtractOperatingDistance(char* argument)
	{
	static const std::string errorMessage = "The 6th parameter operatingDistance has to be a positive floating point number";
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
	static const std::string errorMessage = "The 8th parameter relativeLocationError has to be a positive floating between 0 and 1";
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
	static const std::string errorMessage = "The 9th parameter relativeOrientationError has to be a positive floating between 0 and 1";
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
	static const std::string errorMessage = "The 10th parameter errorRatioComparedToOperatingDistance has to be a positive floating between 0 and 1";
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
	std::string harrisConfigurationFilePath, icpConfigurationFilePath;
	std::string sceneFilePath, modelFilePath, groundTruthPoseFilePath;
	std::string guessPoseFilePath;
	bool useGuessPose;
	float operatingDistance;
	float relativeLocationError = 0.10;
	float relativeOrientationError = 0.10;
	float errorRatioComparedToOperatingDistance = 0.05;

	ASSERT(argc >= 6, USAGE);
	harrisConfigurationFilePath = argv[1];
	icpConfigurationFilePath = argv[2];
	sceneFilePath = argv[3];
	modelFilePath = argv[4];
	groundTruthPoseFilePath = argv[5];
	operatingDistance = ExtractOperatingDistance(argv[6]);
	useGuessPose = (argc >= 8);

	if (argc >= 8)
		{
		guessPoseFilePath = argv[7];
		}
	if (argc >= 9)
		{
		relativeLocationError = ExtractRelativeLocationError(argv[8]);
		}
	if (argc >= 10)
		{
		relativeLocationError = ExtractRelativeOrientationError(argv[9]);
		}
	if (argc >= 11)
		{
		relativeLocationError = ExtractErrorRatio(argv[10]);
		}
	
	HarrisDetector3D* harris = new HarrisDetector3D(); 
	IcpCC* icp = new IcpCC();

	CorrectLocalizationTester tester(icpConfigurationFilePath, icp, harrisConfigurationFilePath, harris);
	tester.SetInputClouds(sceneFilePath, modelFilePath, groundTruthPoseFilePath);
	if (useGuessPose)
		{
		tester.SetGuessModelPoseInScene(guessPoseFilePath);
		}

	tester.ExecuteDfns();
	float absoluteLocationError = operatingDistance * errorRatioComparedToOperatingDistance;
	bool success = tester.IsOutputCorrect(relativeLocationError, relativeOrientationError, absoluteLocationError);

	VERIFY_REQUIREMENT(success, "Correctness requirement 4.1.1.9 failed on the input scene and model point cloud");
	return 0;
	}


/** @} */
