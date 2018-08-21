/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisShotIcp.cpp
 * @date 03/04/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Correctness Test for DFNs implementation HarrisDetector3D - ShotDescriptor3D - Icp3D.
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
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <FeaturesMatching3D/Icp3D.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFN::FeaturesExtraction3D;
using namespace CDFF::DFN::FeaturesDescription3D;
using namespace CDFF::DFN::FeaturesMatching3D;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
" \n \
You should provide at least seven parameters: \n \
(i) the configuration file path of Harris Detector 3D; \n \
(ii) the configuration file path of Shot Descriptor 3D; \n \
(iii) the configuration file path of Icp 3D; \n \
(iv) the scene input cloud file path, the input cloud file should be in ply format; \n \
(v) the model input cloud file path, the input cloud file should be in ply format; \n \
(vi) the text file containing the ground truth pose on one line with format x y z qx qy qz qw \n \
(vii) the operating distance expressed in meters; \n \
Optionally you can add up to three float parameters: \n \
(i) relativeLocationError: the maximum position error expressed as ratio to the size of the model, it needs to be in the interval [0,1]. The default is 0.10; \n \
(ii) relativeOrientationError: the maximum orientation error expressed as ratio to the size of the model, it needs to be in the interval [0,1]. The default is 0.10; \n \
(iii) errorRatioComparedToOperatingDistance: the maximum position error expressed as ratio to the operating distance. The default is 0.05. \n \n \
Example Usage: ./correctness_harris_shot_icp ../tests/ConfigurationFiles/DFNs/FeaturesExtractor3D/HarrisDetector3D_DevonIsland.yaml ../tests/ConfigurationFiles/DFNs/FeaturesDescriptor3D/ShotDescriptor3D_DevonIsland.yaml ../tests/ConfigurationFiles/DFNs/FeaturesMatcher3D/Icp3D_DevonIsland.yaml ../tests/Data/PointClouds/DevonIslandRoad.ply ../tests/Data/PointClouds/DevonIslandRoadTransformed.ply ../tests/Data/PointClouds/DevonIslandRoadTransform.txt 20 \n \n";


float ExtractOperatingDistance(char* argument)
	{
	static const std::string errorMessage = "The 7th parameter operatingDistance has to be a positive floating point number";
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
	std::string harrisConfigurationFilePath, shotConfigurationFilePath, icpConfigurationFilePath;
	std::string sceneFilePath, modelFilePath, groundTruthPoseFilePath;
	float operatingDistance;
	float relativeLocationError = 0.10;
	float relativeOrientationError = 0.10;
	float errorRatioComparedToOperatingDistance = 0.05;

	ASSERT(argc >= 8, USAGE);
	harrisConfigurationFilePath = argv[1];
	shotConfigurationFilePath = argv[2];
	icpConfigurationFilePath = argv[3];
	sceneFilePath = argv[4];
	modelFilePath = argv[5];
	groundTruthPoseFilePath = argv[6];
	operatingDistance = ExtractOperatingDistance(argv[7]);

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
	ShotDescriptor3D* shot = new ShotDescriptor3D();
	Icp3D* icp = new Icp3D();

	CorrectLocalizationTester tester;
	tester.SetDfns(harris, shot, icp);
	tester.SetConfigurationFiles(harrisConfigurationFilePath, shotConfigurationFilePath, icpConfigurationFilePath);
	tester.SetInputClouds(sceneFilePath, modelFilePath, groundTruthPoseFilePath);

	tester.ExecuteDfns();
	float absoluteLocationError = operatingDistance * errorRatioComparedToOperatingDistance;
	bool success = tester.IsOutputCorrect(relativeLocationError, relativeOrientationError, absoluteLocationError);

	VERIFY_REQUIREMENT(success, "Correctness requirement 4.1.1.9 failed on the input scene and model point cloud");
	return 0;
	}


/** @} */
