/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file CeresAdjustment.cpp
 * @date 19/06/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Position Test 4.1.1.2 for DFN implementation CeresAdjustment.
 * "Point cloud should be located and oriented within 10% of the size and relative orientation of the scene."
 * Requirement 4.1.1.10 lists two more properties, but they will not be tested here as they relate to the reconstruction of a point clouds, which is a problem this DFN does not deal with.
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "PositionTester.hpp"
#include <BundleAdjustment/CeresAdjustment.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFN::BundleAdjustment;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
" \n \
You should provide at least four parameters: \n \
(i) the configuration file path; \n \
(ii) the correspondences file path; \n \
(iii) the reference positions file path; \n \
(iv) the model size expressed in meters; \n \
Optionally you can add up to two other parameters: \n \
(i) relativeLocationError: the maximum position error expressed as ratio to the size of the model, it needs to be in the interval [0,1]. The default is 0.10; \n \
(ii) relativeOrientationError: the maximum orientation error expressed as ratio to the size of the model, it needs to be in the interval [0,1]. The default is 0.10; \n \
Example Usage: ./position_test_ceres ../tests/ConfigurationFiles/DFNs/BundleAdjustment/CeresAdjustment_DevonIsland.yaml \
../tests/Data/Images/devonIslandCorrespondences.xml ../tests/Data/Images/devonIslandCameraPosition.xml \n \n"; 

float ExtractModelSize(char* argument)
	{
	static const std::string errorMessage = "The 4th parameter modelSize has to be a floating number greater than 0";
	float modelSize;

	try 
		{
		modelSize = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(modelSize > 0, errorMessage);

	return modelSize;
	}

float ExtractRelativeLocationError(char* argument)
	{
	static const std::string errorMessage = "The 5th parameter relativeLocationError has to be a positive floating between 0 and 1";
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
	static const std::string errorMessage = "The 6th parameter relativeOrientationError has to be a positive floating between 0 and 1";
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


int main(int argc, char** argv)
	{
	std::string configurationFilePath;
	std::string inputCorrespondenceFilePath;
	std::string positionReferenceFilePath;

	float modelSize;
	float relativeLocationError = 0.10;
	float relativeOrientationError = 0.10;

	ASSERT(argc >= 5, USAGE);
	configurationFilePath = argv[1];
	inputCorrespondenceFilePath = argv[2];
	positionReferenceFilePath = argv[3];
	modelSize = ExtractModelSize(argv[4]);

	if (argc >= 6)
		{
		relativeLocationError = ExtractRelativeLocationError(argv[5]);
		}
	if (argc >= 7)
		{
		relativeOrientationError = ExtractRelativeOrientationError(argv[6]);
		}
	 

	CeresAdjustment* ceres = new CeresAdjustment();
	PositionTester tester(configurationFilePath, ceres);
	tester.SetFilesPaths(inputCorrespondenceFilePath, positionReferenceFilePath);
	tester.ExecuteDfn();
	bool success = tester.ArePositionsCloseToReference(relativeLocationError, relativeOrientationError, modelSize);

	VERIFY_REQUIREMENT(success, "Validity of camera poses 4.1.1.10 failed on the input correspondences set");
	return 0;
	}


/** @} */
