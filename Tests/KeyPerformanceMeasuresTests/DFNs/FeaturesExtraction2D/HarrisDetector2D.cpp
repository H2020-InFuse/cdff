/* --------------------------------------------------------------------------
*
* (C) Copyright …
*
* ---------------------------------------------------------------------------
*/

/*!
 * @file HarrisDetector2D.cpp
 * @date 04/05/2017
 * @author Alessandro Bianco
 */

/*!
 * @addtogroup DFNsTest
 * 
 * Validity Test 4.1.1.2 for DFN implementation HarrisDetector2D.
 * "The number of features detected should be within 10% of the number of features in a reference case for feature detection obtained by a standard detection algorithm (open source and in OpenCV)
 * implemented for a given descriptor.  These open-source descriptor implementations are considered to be industry standard, only parameters are expected to cause differences between implementations
 * of the same descriptor on the same data." and 
 * "90% of manually detected features should lay within a 5 pixel distance of the a detected feature.  Manually detected features are considered to be features identified by a human inspecting 
 * a close-up of an image to establish whether a feature is centered correctly on a part of the image.  A human will inspect features to identify any that are misplaced on a part of an image."
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
#include <FeaturesExtraction2D/HarrisDetector2D.hpp>
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
	(ii) the input image file path; \n \
	(iii) the number reference file path containing the number of best keypoints that we would expect to detect. It has to be in opencv xml format. \n \
	(iv) the precision reference file path containing all the admissible keypoints. It has to be in opencv xml format. \n \n \
	Optionally you can add up to three other parameters: \n \
	(i) the percentage within which the number of detected keypoints should be when compared to the number of keypoints detected in a reference case. It must be a number between 0 and 1; \
	The default is 0.10. \n \
	(ii) the distance threshold between a feature and a reference feature beyond which the detected feature is considered incorrect. This is measured in pixels and has to be a non negative integer; \
	The default is 5. \n \
	(iii) the percentage of detected features whose distance from a reference feature should be withing the distance threshold. It must be a number between 0 and 1. The default is 0.90. \n \n"; 

float ExtractNumberPercentageThreshold(char* argument)
	{
	const std::string errorMessage = "The 4th parameter numberPercentageThreshold has to be a floating point number between 0 and 1";
	float numberPercentageThreshold;

	try 
		{
		numberPercentageThreshold = std::stof(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	ASSERT(numberPercentageThreshold >= 0 && numberPercentageThreshold <= 1, errorMessage);
	
	return numberPercentageThreshold;
	}

unsigned ExtractPixelOutlierThreshold(char* argument)
	{
	const std::string errorMessage = "The 5th parameter numberPercentageThreshold has to be a non negative integer.";
	unsigned pixelOutlierThreshold;

	try 
		{
		pixelOutlierThreshold = std::stoul(argument);
		}
	catch (...)
		{
		ASSERT(false, errorMessage);
		}
	
	return pixelOutlierThreshold;
	}

float ExtractOutliersPercentageThreshold(char* argument)
	{
	const std::string errorMessage = "The 6th parameter numberPercentageThreshold has to be a floating point number between 0 and 1";
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


int main(int argc, char** argv)
	{
	std::string configurationFilePath;
	std::string inputImageFilePath;
	std::string numberReferenceFilePath;
	std::string precisionReferenceFeaturesVector;
	float numberPercentageThreshold = 0.10;
	unsigned pixelOutlierThreshold = 5;
	float outliersPercentageThreshold = 0.90;

	ASSERT(argc >= 5, USAGE);
	configurationFilePath = argv[1];
	inputImageFilePath = argv[2];
	numberReferenceFilePath = argv[3];
	precisionReferenceFeaturesVector = argv[4];

	if (argc >= 6)
		{
		numberPercentageThreshold = ExtractNumberPercentageThreshold(argv[5]);
		}
	if (argc >= 7)
		{
		pixelOutlierThreshold = ExtractPixelOutlierThreshold(argv[6]);
		}
	if (argc >= 8)
		{
		outliersPercentageThreshold = ExtractOutliersPercentageThreshold(argv[7]);
		}
	 

	HarrisDetector2D* harris = new HarrisDetector2D();
	SelectionTester tester(configurationFilePath, harris);
	tester.SetFilesPaths(inputImageFilePath, numberReferenceFilePath, precisionReferenceFeaturesVector);
	tester.ExecuteDfn();
	bool success = tester.IsSelectionValid(numberPercentageThreshold, pixelOutlierThreshold, outliersPercentageThreshold);

	ASSERT(success, "Regularity requirement 4.1.1.2 failed on the input point cloud");
	return 0;
	}


/** @} */
