/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbFlann.cpp
 * @date 07/05/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * This class will test requirement 4.1.1.3 of deliverable 5.5.
 * "90% of manually defined matches on manually defined features should be detected correctly.  Manually defined matches are considered to be matches between features made by a 
 * human on close inspection of a set of features on a pair of images.  A human will inspect each set of matches and identify those that are incorrect."
 *
 * Requirement 4.1.1.3 has also another statement, but this one is the one that should be verified by the Requirement 4.1.1.2 since it referes to the detection function only.
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
#include <FeaturesMatching2D/FlannMatcher.hpp>
#include <FeaturesDescription2D/OrbDescriptor.hpp>
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
(i) the features descriptor dfn configuration file path; \n \
(ii) the features matching dfn configuration file path; \n \
(iii) the source input image file path; \n \
(iv) the sink input image file path; \n \
(v) the correspondence map file path in xml format. This is the result of using the tool DataGenerator/match_images. \n \n \
Optionally you can add one more parameter: \n \
(i) the minimum percentage of detected correct correspondences. It must be a number between 0 and 1. The default is 0.90. \n \n \
Example Usage: ./validity_orb_flann ../tests/ConfigurationFiles/DFNs/FeaturesDescriptor2D/OrbDescriptor_DevonIsland.yaml ../tests/ConfigurationFiles/DFNs/FeaturesMatching2D/FlannMatcher_DevonIsland.yaml ../tests/Data/Images/DevonIslandRoadLeft.yaml ../tests/Data/Images/DevonIslandRoadRight.yaml ../tests/Data/Images/DevonIslandRoadMatches.xml \n \n"; 

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
	std::string sourceImageFilePath;
	std::string sinkImageFilePath;
	std::string correspondencesImageFilePath;
	float percentageThreshold = 0.90;

	ASSERT(argc >= 6, USAGE);
	featuresDescriptorConfigurationFilePath = argv[1];
	featuresMatcherConfigurationFilePath = argv[2];
	sourceImageFilePath = argv[3];
	sinkImageFilePath = argv[4];
	correspondencesImageFilePath = argv[5];

	if (argc >= 7)
		{
		percentageThreshold = ExtractPercentageThreshold(argv[6]);
		}


	SelectionTester tester;
	tester.SetConfigurationFilePaths(featuresDescriptorConfigurationFilePath, featuresMatcherConfigurationFilePath);
	tester.SetInputFilesPaths(sourceImageFilePath, sinkImageFilePath, correspondencesImageFilePath);

	OrbDescriptor* orb = new OrbDescriptor();
	FlannMatcher* flann = new FlannMatcher();
	tester.SetDfns(orb, flann);
	
	tester.ExecuteDfns();
	bool success = tester.AreCorrespondencesValid(percentageThreshold);

	VERIFY_REQUIREMENT(success, "Features Selection Validity requirement 4.1.1.3 failed on the input images");
	return 0;
	}


/** @} */
