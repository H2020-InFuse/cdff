/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file OrbDetectorDescriptorFlannMatcher.cpp
 * @date 22/03/2018
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Performance Test for the integration of Orb extractor descriptor and Flann matcher
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
#include "FeaturesExtractionDescriptionMatching.hpp"

#include <FeaturesExtraction2D/OrbDetectorDescriptor.hpp>
#include <FeaturesMatching2D/FlannMatcher.hpp>

using namespace CDFF::DFN::FeaturesExtraction2D;
using namespace CDFF::DFN::FeaturesMatching2D;

const std::string USAGE =
	" \n \
	This programs requires four parameters: \n \
	(i) the folder path containing the configuration files of each DFN \n \
	(ii) the name of the OrbDetectorDescriptor Configuration file \n \
	(iii) the name of the Flann Configuration file \n \
	(iv) the name of the output performance file (that will be containing in the configuration folder) \n \
	(v) the left image file path; \n \
	(vi) the right image file path; \n \
	Example Usage: ./orb_detector_descriptor_flann ../tests/ConfigurationFiles/DFNsIntegration/FeaturesExtractionDescriptionMatching OrbExtractorDescriptor_PerformanceTest_1.yaml \
	FlannMatcher_PerformanceTest_1.yaml OrbExtractorDescriptor_FlannMatcher.txt ../tests/Data/Images/RectifiedChair40Left.png .../tests/Data/Images/RectifiedChair40Right.png \n \n";

int main(int argc, char** argv)
	{
	ASSERT(argc >= 7, USAGE);
	
	std::string configurationFolderPath, orbConfigurationFileName, flannConfigurationFileName, outputFileName;
	std::string leftImageFilePath, rightImageFilePath;
	configurationFolderPath = argv[1];
	orbConfigurationFileName = argv[2];
	flannConfigurationFileName = argv[3];
	outputFileName = argv[4];
	leftImageFilePath = argv[5];
	rightImageFilePath = argv[6];

	std::vector<std::string> baseConfigurationFiles =
		{
		orbConfigurationFileName,
		flannConfigurationFileName
		};
	FeaturesExtractionDescriptionMatching* interface = new FeaturesExtractionDescriptionMatching(
		configurationFolderPath, 
		baseConfigurationFiles, 
		outputFileName
		);

	OrbDetectorDescriptor* orb = new OrbDetectorDescriptor();
	FlannMatcher* flann = new FlannMatcher();

	interface->SetDfns(orb, NULL, flann);
	interface->SetInputFiles(leftImageFilePath, rightImageFilePath);
	interface->Run();

	delete(interface);
	delete(orb);
	delete(flann);
	};

/** @} */
