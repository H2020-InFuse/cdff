/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file ShotIcp.cpp
 * @date 11/05/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * This program will test requirement 4.1.1.8 of deliverable 5.5 for the implementation SHOT - ICP.
 * 90% of manually defined matches on manually defined features should be detected correctly.  Manually defined matches are considered to be matches between features made by a human 
 * on close inspection of a set of features on a pair of images.  A human will inspect each set of matches and identify those that are incorrect.
 * 
 * @{
 */

/* --------------------------------------------------------------------------
 *
 * Includes
 *
 * --------------------------------------------------------------------------
 */
#include "SelectionTester.hpp"
#include <FeaturesMatching3D/Icp3D.hpp>
#include <FeaturesDescription3D/ShotDescriptor3D.hpp>
#include <Errors/Assert.hpp>

using namespace CDFF::DFN::WHICH-DFN(S)-IF-ANY?;

/* --------------------------------------------------------------------------
 *
 * Test Cases
 *
 * --------------------------------------------------------------------------
 */

const std::string USAGE =
" \n \
You should provide at least five parameters: \n \
(i) the features descriptor dfn configuration file path; \n \
(ii) the features matching dfn configuration file path; \n \
(iii) the source input cloud file path, in ply format; \n \
(iv) the sink input cloud file path, in ply format; \n \
(v) the correspondence map file path in xml format. This is the result of using the tool DataGenerator/match_clouds. \n \n \
Optionally you can add one more parameter: \n \
(i) the minimum percentage of detected correct correspondences. It must be a number between 0 and 1. The default is 0.90. \n \n \
Example Usage: ./validity_shot_icp ../tests/ConfigurationFiles/DFNs/FeaturesDescriptor3D/ShotDescriptor3D_DevonIsland.yaml ../tests/ConfigurationFiles/DFNs/FeaturesMatching3D/Icp3D_DevonIsland.yaml ../tests/Data/PointClouds/DevonIslandRoadLeft.ply ../tests/Data/PointClouds/DevonIslandRoadRight.ply ../tests/Data/PointClouds/DevonIslandRoadMatches.xml \n \n"; 

float ExtractPercentageThreshold(char* argument)
	{
	const std::string errorMessage = "The 6th parameter percentageThreshold has to be a floating point number between 0 and 1";
	float percentageThreshold;

	try 
		{
		percentageThreshold = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(percentageThreshold >= 0 && percentageThreshold <= 1, errorMessage);
	
	return percentageThreshold;
	}


int main(int argc, char** argv)
	{
	std::string featuresDescriptorConfigurationFilePath;
	std::string featuresMatcherConfigurationFilePath;
	std::string sourceCloudFilePath;
	std::string sinkCloudFilePath;
	std::string correspondencesFilePath;
	float percentageThreshold = 0.90;

	ASSERT(argc >= 6, USAGE);
	featuresDescriptorConfigurationFilePath = argv[1];
	featuresMatcherConfigurationFilePath = argv[2];
	sourceCloudFilePath = argv[3];
	sinkCloudFilePath = argv[4];
	correspondencesFilePath = argv[5];

	if (argc >= 7)
		{
		percentageThreshold = ExtractPercentageThreshold(argv[6]);
		}


	SelectionTester tester;
	tester.SetConfigurationFilePaths(featuresDescriptorConfigurationFilePath, featuresMatcherConfigurationFilePath);
	tester.SetInputFilesPaths(sourceCloudFilePath, sinkCloudFilePath, correspondencesFilePath);

	ShotDescriptor3D* shot = new ShotDescriptor3D();
	Icp3D* icp = new Icp3D();
	tester.SetDfns(shot, icp);
	
	tester.ExecuteDfns();
	bool success = tester.AreCorrespondencesValid(percentageThreshold);

	VERIFY_REQUIREMENT(success, "Features Selection Validity requirement 4.1.1.8 failed on the input point clouds");
	return 0;
	}


/** @} */
