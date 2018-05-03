/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector3D.cpp
 * @date 02/04/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Regularity Test for DFN implementation HarrisDetector3D.
 * " Keypoints should be selected at regular locations throughout the point cloud as viewed by a human observer.  
 *   <Regular> refers to keypoints not exhibiting gaps or clustering exceeding 20% of the average separation between keypoints."
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "RegularityTester.hpp"
#include <FeaturesExtraction3D/HarrisDetector3D.hpp>
#include <Errors/Assert.hpp>

using namespace dfn_ci;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
	"You should provide at least three parameters: \n \
	(i) the configuration file path; \n \
	(ii) the input cloud file path, the input cloud file should be in ply format; \n \
	(iii) the average separation type: one of EveryPair or ClosestNeighbour. \n \
	Optionally you can add as fourth parameter a regularity float number between 0 and 1 \n"; 

RegularityTester::AverageSeparationType FromString(std::string averageSeparationTypeString)
	{
	if (averageSeparationTypeString == "EveryPair")
		{
		return RegularityTester::POINTS_PAIR_DISTANCE;
		}
	else if (averageSeparationTypeString == "ClosestNeighbour")
		{
		return RegularityTester::CLOSEST_NEIGHBOUR_DISTANCE;
		}
	else
		{
		ASSERT(false, "The third parameter should be either EveryPair or ClosestNeighbour, capital letters matter");
		}
	return RegularityTester::POINTS_PAIR_DISTANCE;
	}

int main(int argc, char** argv)
	{
	std::string configurationFilePath;
	std::string inputCloudFilePath;
	std::string averageSeparationTypeString;
	float regularity = 0.20;

	ASSERT(argc >= 4, USAGE);
	configurationFilePath = argv[1];
	inputCloudFilePath = argv[2];
	averageSeparationTypeString = argv[3];

	if (argc >= 5)
		{
		try 
			{
			regularity = std::stof(argv[4]);
			}
		catch (...)
			{
			ASSERT(false, "The fourth parameter regularity has to be a floating point number between 0 and 1");
			}
		ASSERT(regularity >= 0 && regularity <= 1, "The third parameter regularity has to be a floating point number between 0 and 1");
		}
	 

	HarrisDetector3D* harris = new HarrisDetector3D();
	RegularityTester tester(configurationFilePath, inputCloudFilePath, FromString(averageSeparationTypeString), harris);
	tester.ExecuteDfn();
	bool success = tester.IsOutputRegular(regularity);

	ASSERT(success, "Regularity requirement 4.1.1.7 failed on the input point cloud");
	return 0;
	}


/** @} */